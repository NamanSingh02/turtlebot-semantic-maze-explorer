#!/usr/bin/env python3
"""Detecting objects from ROS images and publishing timestamp-aligned Zenoh events."""

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path
from uuid import uuid4

import rclpy
from cv_bridge import CvBridge
from events.core import ROBOT_PATTERN, OdomBuffer, key_for, make_event, transform_matrix
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO

import zenoh


class EventDetector(Node):
    def __init__(self, args):
        super().__init__(
            "maze_event_detector",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.args = args
        self.run_id = str(uuid4())
        self.sequence = 0
        self.odom = OdomBuffer(max_gap_ns=int(args.max_odom_gap * 1e9))
        self.last_odom_ns = None
        self.pending = None
        self.last_inference = -float("inf")
        self.bridge = CvBridge()
        self.model = YOLO(args.model)
        self.transforms = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.transforms, self)
        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", json.dumps([args.connect]))
        self.session = zenoh.open(config)
        directory = Path(args.output)
        directory.mkdir(parents=True, exist_ok=True)
        self.spool = (directory / f"{self.run_id}.jsonl").open("x")
        metadata = dict(
            schema="maze.runmeta.v1",
            run_id=self.run_id,
            robot_id=args.robot_id,
            model=args.model,
            confidence=args.confidence,
            max_fps=args.max_fps,
            ros_distro=os.environ.get("ROS_DISTRO"),
            image_topic=args.image_topic,
            odom_topic=args.odom_topic,
            base_frame=args.base_frame,
            time_alignment="bounded linear interpolation; shortest-angle yaw",
            max_odom_gap_seconds=args.max_odom_gap,
            sha256_scope="ROS image data bytes including row padding",
            stamp_semantics="ROS image time; simulation time is not wall time",
        )
        (directory / f"{self.run_id}.runmeta.json").write_text(
            json.dumps(metadata, indent=2)
        )
        self.session.put(
            f"maze/{args.robot_id}/{self.run_id}/runmeta/v1", json.dumps(metadata)
        )
        self.create_subscription(
            Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, args.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.create_timer(0.02, self.process_pending)
        self.get_logger().info(
            f"Run {self.run_id}; writing events to {self.spool.name}"
        )

    def on_odom(self, message):
        timestamp = Time.from_msg(message.header.stamp).nanoseconds
        if (
            self.last_odom_ns is not None
            and timestamp < self.last_odom_ns - 500_000_000
        ):
            self.odom.samples.clear()
            self.get_logger().warning(
                "Simulation time reset; clearing odometry history"
            )
        self.last_odom_ns = timestamp
        pose, twist = message.pose.pose, message.twist.twist
        q = pose.orientation
        self.odom.add(
            dict(
                time_ns=timestamp,
                frame_id=message.header.frame_id,
                child_frame_id=message.child_frame_id,
                x=pose.position.x,
                y=pose.position.y,
                yaw=math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                ),
                vx=twist.linear.x,
                vy=twist.linear.y,
                wz=twist.angular.z,
            )
        )

    def on_image(self, message):
        if (
            self.pending is None
            and time.monotonic() - self.last_inference >= 1 / self.args.max_fps
        ):
            self.pending = (message, time.monotonic())

    def process_pending(self):
        if self.pending is None:
            return
        message, arrived = self.pending
        timestamp = Time.from_msg(message.header.stamp).nanoseconds
        state = self.odom.at(timestamp)
        if state is None:
            if time.monotonic() - arrived > self.args.state_wait:
                self.get_logger().warning(
                    "Dropping image without timestamp-aligned odometry"
                )
                self.pending = None
            return
        self.pending = None
        self.last_inference = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            image = dict(
                topic=self.args.image_topic,
                stamp=dict(
                    sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec
                ),
                frame_id=message.header.frame_id,
                width=message.width,
                height=message.height,
                encoding=message.encoding,
                sha256=hashlib.sha256(bytes(message.data)).hexdigest(),
            )
            odometry = {
                field: state[field]
                for field in ("frame_id", "x", "y", "yaw", "vx", "vy", "wz")
            }
            odometry["topic"] = self.args.odom_topic
            camera_frame = self.args.camera_frame or message.header.frame_id
            tf = dict(
                base_frame=self.args.base_frame,
                camera_frame=camera_frame,
                t_base_camera=None,
                tf_ok=False,
            )
            try:
                transform = self.transforms.lookup_transform(
                    self.args.base_frame,
                    camera_frame,
                    Time.from_msg(message.header.stamp),
                )
                t, q = transform.transform.translation, transform.transform.rotation
                tf["t_base_camera"] = transform_matrix(
                    (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
                )
                tf["tf_ok"] = True
            except TransformException:
                pass
            detections = []
            for result in self.model.predict(
                frame, conf=self.args.confidence, device=self.args.device, verbose=False
            ):
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                    bbox = [
                        max(0.0, x1),
                        max(0.0, y1),
                        min(float(message.width), x2),
                        min(float(message.height), y2),
                    ]
                    if bbox[0] >= bbox[2] or bbox[1] >= bbox[3]:
                        continue
                    detections.append(
                        dict(
                            det_id=str(uuid4()),
                            class_id=class_id,
                            class_name=self.model.names[class_id],
                            confidence=float(box.conf[0]),
                            bbox_xyxy=bbox,
                        )
                    )
            event = make_event(
                self.run_id,
                self.args.robot_id,
                self.sequence,
                image,
                odometry,
                tf,
                detections,
            )
            payload = json.dumps(event, allow_nan=False, separators=(",", ":"))
            self.spool.write(payload + "\n")
            self.spool.flush()
            os.fsync(self.spool.fileno())
            self.sequence += 1
            try:
                self.session.put(key_for(event), payload)
            except Exception as error:
                self.get_logger().error(
                    f"Publish failed; event remains in spool: {error}"
                )
            self.get_logger().info(
                f"Event {event['sequence']}: {len(detections)} detections; tf_ok={tf['tf_ok']}"
            )
        except Exception as error:
            self.get_logger().error(f"Frame processing failed: {error}")

    def close(self):
        self.spool.close()
        self.session.close()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default="tcp/localhost:7447")
    parser.add_argument("--robot-id", default="tb3_sim")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--camera-frame", default="")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--device", default="0")
    parser.add_argument("--max-fps", type=float, default=5)
    parser.add_argument("--max-odom-gap", type=float, default=0.2)
    parser.add_argument("--state-wait", type=float, default=0.3)
    parser.add_argument("--output", default="/data/events")
    args, ros_args = parser.parse_known_args()
    if (
        not ROBOT_PATTERN.fullmatch(args.robot_id)
        or args.max_fps <= 0
        or args.max_odom_gap <= 0
        or args.state_wait <= 0
        or not 0 <= args.confidence <= 1
    ):
        parser.error("Invalid robot ID, timing limit, or confidence")
    rclpy.init(args=ros_args)
    node = EventDetector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

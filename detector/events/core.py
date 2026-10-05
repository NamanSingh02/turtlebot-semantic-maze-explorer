"""Validating events and aligning odometry to image time without ROS dependencies."""

import bisect
import math
import re
from uuid import UUID, uuid4


SCHEMA = "maze.detection.v1"
ROBOT_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")


def stamp_ns(stamp):
    sec, nano = stamp["sec"], stamp["nanosec"]
    if type(sec) is not int or type(nano) is not int or not 0 <= nano < 1_000_000_000:
        raise ValueError("Invalid image stamp")
    return sec * 1_000_000_000 + nano


def key_for(event):
    return (
        f"maze/{event['robot_id']}/{event['run_id']}/detections/v1/{event['event_id']}"
    )


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate(event, key=None):
    if not isinstance(event, dict) or event.get("schema") != SCHEMA:
        raise ValueError("Unsupported event schema")
    for field in ("event_id", "run_id"):
        if str(UUID(event[field])) != event[field]:
            raise ValueError(f"Noncanonical {field}")
    if not ROBOT_PATTERN.fullmatch(event["robot_id"]):
        raise ValueError("Invalid robot identifier")
    if type(event["sequence"]) is not int or event["sequence"] < 0:
        raise ValueError("Invalid sequence")
    image = event["image"]
    stamp_ns(image["stamp"])
    for field in ("width", "height"):
        if type(image[field]) is not int or image[field] <= 0:
            raise ValueError("Invalid image dimensions")
    for field in ("topic", "frame_id", "encoding"):
        if not isinstance(image[field], str) or not image[field]:
            raise ValueError(f"Missing image {field}")
    if not re.fullmatch(r"[0-9a-f]{64}", image["sha256"]):
        raise ValueError("Invalid pixel SHA256")
    odom = event["odometry"]
    if not odom["topic"] or not odom["frame_id"]:
        raise ValueError("Missing odometry frame/topic")
    if not all(finite(odom[f]) for f in ("x", "y", "yaw", "vx", "vy", "wz")):
        raise ValueError("Nonfinite odometry")
    tf = event["tf"]
    if not tf["base_frame"] or not tf["camera_frame"] or type(tf["tf_ok"]) is not bool:
        raise ValueError("Invalid transform metadata")
    matrix = tf["t_base_camera"]
    if tf["tf_ok"]:
        if (
            not isinstance(matrix, list)
            or len(matrix) != 16
            or not all(map(finite, matrix))
        ):
            raise ValueError("Invalid 4x4 transform")
        if matrix[12:] != [0.0, 0.0, 0.0, 1.0]:
            raise ValueError("Invalid homogeneous transform row")
    elif matrix is not None:
        raise ValueError("Unavailable transform must be null")
    if not isinstance(event["detections"], list):
        raise ValueError("Invalid detections")
    seen = set()
    for detection in event["detections"]:
        identifier = str(UUID(detection["det_id"]))
        if identifier != detection["det_id"] or identifier in seen:
            raise ValueError("Invalid or repeated detection identifier")
        seen.add(identifier)
        if (
            type(detection["class_id"]) is not int
            or not 0 <= detection["class_id"] < 80
        ):
            raise ValueError("Invalid COCO class identifier")
        if not isinstance(detection["class_name"], str) or not detection["class_name"]:
            raise ValueError("Missing class name")
        confidence = detection["confidence"]
        if not finite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("Invalid confidence")
        box = detection["bbox_xyxy"]
        if not isinstance(box, list) or len(box) != 4 or not all(map(finite, box)):
            raise ValueError("Invalid bounding box")
        x1, y1, x2, y2 = box
        if not 0 <= x1 < x2 <= image["width"] or not 0 <= y1 < y2 <= image["height"]:
            raise ValueError("Bounding box outside image")
    if key is not None and key != key_for(event):
        raise ValueError("Key does not match event identity")
    return event


class OdomBuffer:
    """Interpolating planar state only between nearby samples in the same frames."""

    def __init__(self, max_gap_ns=200_000_000, capacity=500):
        self.samples = []
        self.max_gap_ns = max_gap_ns
        self.capacity = capacity

    def add(self, sample):
        times = [s["time_ns"] for s in self.samples]
        index = bisect.bisect_left(times, sample["time_ns"])
        if index < len(times) and times[index] == sample["time_ns"]:
            self.samples[index] = sample
        else:
            self.samples.insert(index, sample)
        self.samples = self.samples[-self.capacity :]

    def at(self, time_ns):
        times = [s["time_ns"] for s in self.samples]
        index = bisect.bisect_left(times, time_ns)
        if index < len(times) and times[index] == time_ns:
            return dict(self.samples[index])
        if index == 0 or index == len(times):
            return None
        before, after = self.samples[index - 1 : index + 1]
        gap = after["time_ns"] - before["time_ns"]
        if gap > self.max_gap_ns or any(
            before[f] != after[f] for f in ("frame_id", "child_frame_id")
        ):
            return None
        ratio = (time_ns - before["time_ns"]) / gap
        result = dict(before, time_ns=time_ns)
        for field in ("x", "y", "vx", "vy", "wz"):
            result[field] = before[field] + ratio * (after[field] - before[field])
        delta = math.atan2(
            math.sin(after["yaw"] - before["yaw"]),
            math.cos(after["yaw"] - before["yaw"]),
        )
        result["yaw"] = math.atan2(
            math.sin(before["yaw"] + ratio * delta),
            math.cos(before["yaw"] + ratio * delta),
        )
        return result


def transform_matrix(translation, quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("Zero quaternion")
    x, y, z, w = (v / norm for v in quaternion)
    tx, ty, tz = translation
    return [
        1 - 2 * (y * y + z * z),
        2 * (x * y - z * w),
        2 * (x * z + y * w),
        tx,
        2 * (x * y + z * w),
        1 - 2 * (x * x + z * z),
        2 * (y * z - x * w),
        ty,
        2 * (x * z - y * w),
        2 * (y * z + x * w),
        1 - 2 * (x * x + y * y),
        tz,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def make_event(run_id, robot_id, sequence, image, odometry, tf, detections):
    event = dict(
        schema=SCHEMA,
        event_id=str(uuid4()),
        run_id=run_id,
        robot_id=robot_id,
        sequence=sequence,
        image=image,
        odometry=odometry,
        tf=tf,
        detections=detections,
    )
    return validate(event)

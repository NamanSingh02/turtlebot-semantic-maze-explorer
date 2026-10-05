"""Writing a validated frame and its detections atomically."""

from datetime import datetime, timedelta, timezone

from psycopg2.extras import Json

from events.core import validate


class IdentityConflict(ValueError):
    pass


def insert_event(connection, event, key=None):
    validate(event, key)
    image, odom, tf = event["image"], event["odometry"], event["tf"]
    stamp = datetime(1970, 1, 1, tzinfo=timezone.utc) + timedelta(
        seconds=image["stamp"]["sec"], microseconds=image["stamp"]["nanosec"] // 1000
    )
    with connection:
        with connection.cursor() as cursor:
            cursor.execute(
                """
                INSERT INTO detection_events
                (event_id, run_id, robot_id, sequence, stamp, image_frame_id,
                 image_sha256, width, height, encoding, x, y, yaw, vx, vy, wz,
                 tf_ok, t_base_camera, raw_event)
                VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
                ON CONFLICT DO NOTHING RETURNING event_id
            """,
                (
                    event["event_id"],
                    event["run_id"],
                    event["robot_id"],
                    event["sequence"],
                    stamp,
                    image["frame_id"],
                    image["sha256"],
                    image["width"],
                    image["height"],
                    image["encoding"],
                    *(odom[f] for f in ("x", "y", "yaw", "vx", "vy", "wz")),
                    tf["tf_ok"],
                    tf["t_base_camera"],
                    Json(event),
                ),
            )
            if cursor.fetchone() is None:
                cursor.execute(
                    "SELECT raw_event FROM detection_events WHERE event_id=%s",
                    (event["event_id"],),
                )
                stored = cursor.fetchone()
                if stored is None or stored[0] != event:
                    raise IdentityConflict(
                        "Conflicting event ID or run sequence; refusing altered replay"
                    )
                return False
            for detection in event["detections"]:
                cursor.execute(
                    """
                    INSERT INTO detections
                    (event_id, det_id, class_id, class_name, confidence, x1, y1, x2, y2)
                    VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s)
                    ON CONFLICT (event_id, det_id) DO NOTHING
                """,
                    (
                        event["event_id"],
                        detection["det_id"],
                        detection["class_id"],
                        detection["class_name"],
                        detection["confidence"],
                        *detection["bbox_xyxy"],
                    ),
                )
    return True

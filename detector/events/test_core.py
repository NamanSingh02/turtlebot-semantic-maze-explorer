"""Checking timestamp alignment, event identity, and validation boundaries."""

import copy
import math
from uuid import uuid4

import pytest

from events.core import OdomBuffer, key_for, make_event, transform_matrix, validate


def sample(time_ns, x=0.0, yaw=0.0, frame="odom"):
    return dict(
        time_ns=time_ns,
        frame_id=frame,
        child_frame_id="base_footprint",
        x=x,
        y=0.0,
        yaw=yaw,
        vx=1.0,
        vy=0.0,
        wz=0.0,
    )


def event():
    image = dict(
        topic="/camera/image_raw",
        stamp=dict(sec=0, nanosec=50),
        frame_id="camera_link",
        width=320,
        height=240,
        encoding="rgb8",
        sha256="a" * 64,
    )
    odom = dict(
        topic="/odom", frame_id="odom", x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0
    )
    tf = dict(
        base_frame="base_footprint",
        camera_frame="camera_link",
        t_base_camera=None,
        tf_ok=False,
    )
    detection = dict(
        det_id=str(uuid4()),
        class_id=41,
        class_name="cup",
        confidence=0.8,
        bbox_xyxy=[1, 2, 30, 40],
    )
    return make_event(str(uuid4()), "tb3_sim", 0, image, odom, tf, [detection])


def test_interpolation_and_yaw_wrap():
    buffer = OdomBuffer(max_gap_ns=100)
    buffer.add(sample(100, 2, math.radians(-179)))
    buffer.add(sample(0, 0, math.radians(179)))
    state = buffer.at(50)
    assert state["x"] == 1
    assert abs(abs(state["yaw"]) - math.pi) < 1e-9
    assert state["time_ns"] == 50


def test_rejecting_extrapolation_and_stale_brackets():
    buffer = OdomBuffer(max_gap_ns=10)
    buffer.add(sample(0))
    buffer.add(sample(100))
    assert buffer.at(-1) is None
    assert buffer.at(101) is None
    assert buffer.at(50) is None
    assert buffer.at(100)["time_ns"] == 100


def test_rejecting_frame_change():
    buffer = OdomBuffer()
    buffer.add(sample(0))
    buffer.add(sample(10, frame="map"))
    assert buffer.at(5) is None


def test_transform_direction_and_rotation():
    matrix = transform_matrix((1, 2, 3), (0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    assert matrix[3::4] == [1, 2, 3, 1]
    assert matrix[0] == pytest.approx(0)
    assert matrix[1] == pytest.approx(-1)
    assert matrix[4] == pytest.approx(1)


def test_key_identity_and_empty_frame():
    item = event()
    assert validate(item, key_for(item)) == item
    item["detections"] = []
    validate(item)
    with pytest.raises(ValueError):
        validate(item, key_for(item) + "bad")


@pytest.mark.parametrize(
    "field,value",
    [
        ("confidence", float("nan")),
        ("confidence", 1.1),
        ("bbox_xyxy", [-1, 2, 30, 40]),
        ("bbox_xyxy", [1, 2, 400, 40]),
        ("class_id", 80),
    ],
)
def test_rejecting_invalid_detection(field, value):
    item = event()
    item["detections"][0][field] = value
    with pytest.raises(ValueError):
        validate(item)


def test_duplicate_detection_id():
    item = event()
    item["detections"].append(copy.deepcopy(item["detections"][0]))
    with pytest.raises(ValueError):
        validate(item)


def test_invalid_time_and_transform():
    item = event()
    item["image"]["stamp"]["nanosec"] = 1_000_000_000
    with pytest.raises(ValueError):
        validate(item)
    item = event()
    item["tf"]["t_base_camera"] = [0] * 16
    with pytest.raises(ValueError):
        validate(item)

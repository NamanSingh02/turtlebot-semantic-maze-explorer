"""Checking real PostgreSQL atomicity, identity conflicts, and report counts."""

import copy
import os
from pathlib import Path
from uuid import uuid4

import psycopg2
import pytest
from event_ingest import ingest_record
from event_report import report

from events.core import key_for
from events.store import IdentityConflict, insert_event
from events.test_core import event


@pytest.fixture
def connection():
    dsn = os.environ.get("TEST_EVENT_DATABASE_URL")
    if not dsn:
        pytest.skip("Setting TEST_EVENT_DATABASE_URL enables PostgreSQL integration")
    conn = psycopg2.connect(dsn)
    schema = "test_" + uuid4().hex
    with conn, conn.cursor() as cursor:
        cursor.execute(f"CREATE SCHEMA {schema}")
        cursor.execute(f"SET search_path TO {schema}")
        cursor.execute(
            (
                Path(__file__).resolve().parents[2] / "db/detection_events.sql"
            ).read_text()
        )
    try:
        yield conn
    finally:
        conn.rollback()
        with conn, conn.cursor() as cursor:
            cursor.execute(f"DROP SCHEMA {schema} CASCADE")
        conn.close()


def test_duplicate_conflict_and_report(connection):
    frame = event()
    assert insert_event(connection, frame, key_for(frame))
    assert not insert_event(connection, frame, key_for(frame))
    altered = copy.deepcopy(frame)
    altered["image"]["sha256"] = "b" * 64
    with pytest.raises(IdentityConflict):
        insert_event(connection, altered)
    collision = copy.deepcopy(frame)
    collision["event_id"] = str(uuid4())
    with pytest.raises(IdentityConflict):
        insert_event(connection, collision)
    result = report(connection, frame["run_id"])
    assert result["robots"][0]["frames"] == 1
    assert result["robots"][0]["missing_sequences"] == 0
    assert result["class_histogram"] == {"cup": 1}
    with connection.cursor() as cursor:
        cursor.execute("SELECT raw_event FROM detection_events")
        assert cursor.fetchone()[0] == frame


def test_rolling_back_frame_when_child_insert_fails(connection):
    with connection, connection.cursor() as cursor:
        cursor.execute(
            "ALTER TABLE detections ADD CONSTRAINT rejecting_test_class CHECK (class_name <> 'cup')"
        )
    with pytest.raises(psycopg2.IntegrityError):
        insert_event(connection, event())
    with connection.cursor() as cursor:
        cursor.execute("SELECT count(*) FROM detection_events")
        assert cursor.fetchone()[0] == 0
        cursor.execute("SELECT count(*) FROM detections")
        assert cursor.fetchone()[0] == 0


def test_empty_frame_and_metadata(connection):
    frame = event()
    frame["detections"] = []
    metadata = dict(
        schema="maze.runmeta.v1", run_id=frame["run_id"], robot_id=frame["robot_id"]
    )
    key = f"maze/{frame['robot_id']}/{frame['run_id']}/runmeta/v1"
    ingest_record(connection, key, metadata)
    ingest_record(connection, key, metadata)
    insert_event(connection, frame)
    result = report(connection, frame["run_id"])
    assert result["detections"] == 0
    assert result["run_metadata"] == [metadata]


def test_zenoh_delivery_into_database(connection):
    import json
    import socket
    import threading

    import zenoh

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = zenoh.Config()
    config.insert_json5("listen/endpoints", json.dumps([f"tcp/127.0.0.1:{port}"]))
    config.insert_json5("scouting/multicast/enabled", "false")
    receiver = zenoh.open(config)
    signal = threading.Event()
    received = []

    def receive(sample):
        received.append((str(sample.key_expr), json.loads(sample.payload.to_bytes())))
        signal.set()

    subscriber = receiver.declare_subscriber("maze/**/detections/v1/*", receive)
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([f"tcp/127.0.0.1:{port}"]))
    sender = zenoh.open(config)
    try:
        frame = event()
        sender.put(key_for(frame), json.dumps(frame))
        assert signal.wait(5), "Zenoh event did not arrive"
        assert ingest_record(connection, *received[0])
        assert report(connection, frame["run_id"])["class_histogram"] == {"cup": 1}
    finally:
        subscriber.undeclare()
        sender.close()
        receiver.close()

#!/usr/bin/env python3
"""Ingesting Zenoh frame events with atomic writes and recoverable failure records."""

import argparse
import json
import logging
import os
import queue
import sys
import threading
import time
from pathlib import Path
from uuid import UUID

import psycopg2
from events.core import ROBOT_PATTERN, key_for, validate
from events.store import insert_event
from psycopg2.extras import Json

import zenoh

LOG = logging.getLogger("maze_ingest")


def ingest_record(connection, key, record):
    if "/runmeta/" not in key:
        return insert_event(connection, record, key)
    if record.get("schema") != "maze.runmeta.v1":
        raise ValueError("Invalid run metadata schema")
    if str(UUID(record["run_id"])) != record["run_id"]:
        raise ValueError("Invalid metadata run ID")
    if not ROBOT_PATTERN.fullmatch(record["robot_id"]):
        raise ValueError("Invalid metadata robot ID")
    expected = f"maze/{record['robot_id']}/{record['run_id']}/runmeta/v1"
    if key != expected:
        raise ValueError("Run metadata key mismatch")
    with connection, connection.cursor() as cursor:
        cursor.execute(
            """INSERT INTO detection_runs(run_id,robot_id,raw_metadata)
                          VALUES (%s,%s,%s) ON CONFLICT DO NOTHING""",
            (record["run_id"], record["robot_id"], Json(record)),
        )
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default="tcp/localhost:7447")
    parser.add_argument("--dsn", default=os.environ.get("EVENT_DATABASE_URL", ""))
    parser.add_argument("--failures", default="/data/events/ingest_failures.jsonl")
    parser.add_argument("--replay", type=Path)
    args = parser.parse_args()
    if not args.dsn:
        parser.error("Provide EVENT_DATABASE_URL or --dsn")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    failures = Path(args.failures)
    failures.parent.mkdir(parents=True, exist_ok=True)
    failure_lock = threading.Lock()
    connection = None

    def failed(key, payload, error):
        LOG.error("Rejected %s: %s", key, error)
        with failure_lock, failures.open("a") as file:
            file.write(
                json.dumps(dict(key=key, payload=payload, error=str(error))) + "\n"
            )
            file.flush()
            os.fsync(file.fileno())

    def store(key, payload):
        nonlocal connection
        try:
            record = json.loads(payload)
            if "/runmeta/" not in key:
                validate(record, key)
            for attempt in range(3):
                try:
                    if connection is None or connection.closed:
                        connection = psycopg2.connect(args.dsn, connect_timeout=5)
                    inserted = ingest_record(connection, key, record)
                    LOG.info("%s %s", "Stored" if inserted else "Duplicate", key)
                    return True
                except (psycopg2.OperationalError, psycopg2.InterfaceError):
                    if connection is not None:
                        connection.close()
                    connection = None
                    if attempt == 2:
                        raise
                    time.sleep(0.5 * (attempt + 1))
        except Exception as error:
            failed(key, payload, error)
            return False

    if args.replay:
        count = 0
        rejected = 0
        with args.replay.open() as file:
            for line in file:
                try:
                    event = json.loads(line)
                    if store(key_for(event), line):
                        count += 1
                    else:
                        rejected += 1
                except Exception as error:
                    failed("invalid-replay", line, error)
                    rejected += 1
        if connection is not None:
            connection.close()
        LOG.info("Processed %s replay records; inspecting failures is required", count)
        if rejected:
            sys.exit(1)
        return

    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([args.connect]))
    session = zenoh.open(config)
    pending = queue.Queue(maxsize=1000)

    def receive(sample):
        key = str(sample.key_expr)
        payload = sample.payload.to_bytes().decode("utf-8", errors="replace")
        try:
            pending.put_nowait((key, payload))
        except queue.Full:
            failed(key, payload, "Ingest queue full; saved for investigation")

    subscribers = [
        session.declare_subscriber("maze/**/detections/v1/*", receive),
        session.declare_subscriber("maze/**/runmeta/v1", receive),
    ]
    LOG.info("Subscribed; ready for detector startup")
    try:
        while True:
            try:
                key, payload = pending.get(timeout=0.2)
            except queue.Empty:
                continue
            store(key, payload)
    except KeyboardInterrupt:
        for subscriber in subscribers:
            subscriber.undeclare()
        while not pending.empty():
            store(*pending.get_nowait())
    finally:
        session.close()
        if connection is not None:
            connection.close()


if __name__ == "__main__":
    main()

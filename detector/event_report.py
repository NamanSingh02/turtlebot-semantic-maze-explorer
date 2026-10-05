#!/usr/bin/env python3
"""Reporting actual stored run counts and class histograms without invented metrics."""

import argparse
import json
import os
from pathlib import Path
from uuid import UUID

import psycopg2


def report(connection, run_id):
    with connection.cursor() as cursor:
        cursor.execute(
            """SELECT robot_id, count(*), min(sequence), max(sequence),
                          count(*) FILTER (WHERE tf_ok),
                          count(*) FILTER (WHERE NOT tf_ok)
                          FROM detection_events WHERE run_id=%s GROUP BY robot_id""",
            (run_id,),
        )
        runs = [
            dict(
                robot_id=row[0],
                frames=row[1],
                min_sequence=row[2],
                max_sequence=row[3],
                tf_available=row[4],
                tf_unavailable=row[5],
                missing_sequences=row[3] + 1 - row[1],
            )
            for row in cursor.fetchall()
        ]
        cursor.execute(
            """SELECT class_name,count(*) FROM detections
                          JOIN detection_events USING(event_id) WHERE run_id=%s
                          GROUP BY class_name ORDER BY count(*) DESC,class_name""",
            (run_id,),
        )
        histogram = dict(cursor.fetchall())
        cursor.execute(
            "SELECT raw_metadata FROM detection_runs WHERE run_id=%s", (run_id,)
        )
        metadata = [row[0] for row in cursor.fetchall()]
    return dict(
        run_id=run_id,
        robots=runs,
        detections=sum(histogram.values()),
        class_histogram=histogram,
        run_metadata=metadata,
        timestamp_note="stamp represents ROS image time; simulation time is stored relative to the Unix epoch",
        metric_note="Counts are observations across frames, not unique objects or accuracy estimates",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--dsn", default=os.environ.get("EVENT_DATABASE_URL", ""))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    UUID(args.run_id)
    with psycopg2.connect(args.dsn) as connection:
        result = report(connection, args.run_id)
    if not result["robots"]:
        parser.error("No stored frames for this run")
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n")
    else:
        print(text)


if __name__ == "__main__":
    main()

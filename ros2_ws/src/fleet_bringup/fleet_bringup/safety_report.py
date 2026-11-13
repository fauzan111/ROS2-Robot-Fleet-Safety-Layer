"""Score a safety log against ground truth and print a summary.

    ros2 run fleet_bringup safety_report /ws/results/safety_log_crossing.jsonl \
        --people /ws/src/fleet_bringup/config/people_crossing.yaml

Use --people none for a run without people (then every stop is a false stop).
Writes <log>.report.json next to the log.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from fleet_bringup.safety_eval import evaluate
from fleet_ml.perception import rectangle_zone


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log")
    ap.add_argument("--people", default="none")
    ap.add_argument("--stop-zone", type=float, nargs=2, default=[1.0, 0.4],
                    metavar=("LENGTH", "HALF_WIDTH"))
    ap.add_argument("--window", type=float, nargs=2, metavar=("START", "END"),
                    help="score only records with START <= t <= END (sim seconds)")
    args, _ = ap.parse_known_args()

    records = [json.loads(line) for line in Path(args.log).read_text().splitlines() if line]
    if args.window:
        records = [r for r in records if args.window[0] <= r["t"] <= args.window[1]]
    people = {}
    if args.people != "none":
        people = {n: p["waypoints"]
                  for n, p in yaml.safe_load(Path(args.people).read_text())["people"].items()}
    report = evaluate(records, people, rectangle_zone(*args.stop_zone))
    out = Path(args.log).with_suffix(".report.json")
    out.write_text(json.dumps(report, indent=2))

    f = report["fleet"]
    lat, halt = f["stop_command_latency_s"], f["halt_latency_s"]
    print(f"operating time: {f['operating_robot_hours'] * 60:.1f} robot-minutes")
    print(f"true stop-zone entries: {f['true_stop_zone_entries']}, "
          f"missed: {f['missed_entries']}")
    print(f"stop command latency: median {lat['median']} s, p95 {lat['p95']} s (n={lat['n']})")
    print(f"halt latency: median {halt['median']} s, p95 {halt['p95']} s (n={halt['n']})")
    print(f"false stops: {f['false_stops']} ({f['false_stops_per_robot_hour']} per robot-hour)")
    print(f"localisation error: {f['localisation_error_m']}")
    for name, r in report.items():
        if name != "fleet":
            print(f"{name}: recall by distance {r['recall_by_distance']}")
    print(f"Report: {out}")


if __name__ == "__main__":
    main()

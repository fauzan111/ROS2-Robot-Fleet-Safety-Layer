"""Merge one fleet run's results into results/fleet_<timestamp>.json and print a table.

    ros2 run fleet_bringup fleet_report /ws/results --since <unix seconds>

Picks, per robot, the newest missions_<robot>_*.json written since the run started,
plus fleet_monitor.json.
"""

from __future__ import annotations

import argparse
import json
import re
from datetime import datetime
from pathlib import Path

from fleet_bringup.fleet_metrics import markdown_table, summarize_fleet

_MISSION_FILE = re.compile(r"missions_(?P<robot>[A-Za-z]\w*?)_\d{8}_\d{6}\.json$")


def latest_missions(results_dir: Path, since: float = 0.0) -> dict[str, dict]:
    newest: dict[str, Path] = {}
    for path in results_dir.glob("missions_*_*.json"):
        m = _MISSION_FILE.search(path.name)
        if not m or path.stat().st_mtime < since:
            continue
        robot = m["robot"]
        if robot not in newest or path.stat().st_mtime > newest[robot].stat().st_mtime:
            newest[robot] = path
    return {robot: json.loads(p.read_text()) for robot, p in newest.items()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir", nargs="?", default="/ws/results")
    parser.add_argument("--since", type=float, default=0.0)
    args, _ = parser.parse_known_args()  # tolerate --ros-args from ros2 run

    results_dir = Path(args.results_dir)
    missions = latest_missions(results_dir, args.since)
    if not missions:
        raise SystemExit(f"no per-robot mission reports in {results_dir}")
    monitor_path = results_dir / "fleet_monitor.json"
    monitor = json.loads(monitor_path.read_text()) if monitor_path.exists() else None

    summary = summarize_fleet(missions, monitor)
    if monitor:
        summary["near_miss_events"] = monitor.get("near_miss_events", [])
    out = results_dir / f"fleet_{datetime.now():%Y%m%d_%H%M%S}.json"
    out.write_text(json.dumps(summary, indent=2))
    print(markdown_table(summary))
    print(f"\nReport: {out}")


if __name__ == "__main__":
    main()

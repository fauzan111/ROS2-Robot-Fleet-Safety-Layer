"""Fleet metrics with no ROS dependency, so they can be unit-tested anywhere.

- odom_to_world: Gazebo's diff-drive odometry starts at zero at the spawn pose.
- PathLength: distance driven, ignoring sub-centimetre jitter.
- NearMissCounter: pairwise separation with hysteresis, so one close pass is one event.
- summarize_fleet / markdown_table: merge per-robot mission reports with monitor output.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import combinations


def odom_to_world(spawn: dict, x: float, y: float) -> tuple[float, float]:
    """Transform an odom-frame position into the world frame given the spawn pose."""
    yaw = float(spawn.get("yaw", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    return spawn["x"] + c * x - s * y, spawn["y"] + s * x + c * y


@dataclass
class PathLength:
    min_step_m: float = 0.005
    total_m: float = 0.0
    _last: tuple[float, float] | None = None

    def update(self, x: float, y: float) -> None:
        if self._last is None:
            self._last = (x, y)
            return
        step = math.dist(self._last, (x, y))
        if step >= self.min_step_m:  # below this it's odometry jitter, keep the anchor
            self.total_m += step
            self._last = (x, y)


@dataclass
class NearMissCounter:
    """Count close passes between robots (centre-to-centre distance).

    A pair enters a near miss below threshold_m and must separate beyond clear_m
    before a new event can be counted.
    """

    threshold_m: float = 0.5
    clear_m: float = 0.6
    events: list[dict] = field(default_factory=list)
    min_separation_m: dict[str, float] = field(default_factory=dict)
    _active: dict[str, int] = field(default_factory=dict)  # pair -> index into events

    def update(self, t: float, positions: dict[str, tuple[float, float]]) -> None:
        for a, b in combinations(sorted(positions), 2):
            pair = f"{a}-{b}"
            d = math.dist(positions[a], positions[b])
            self.min_separation_m[pair] = min(self.min_separation_m.get(pair, math.inf), d)
            if pair in self._active:
                event = self.events[self._active[pair]]
                event["min_distance_m"] = min(event["min_distance_m"], round(d, 3))
                if d > self.clear_m:
                    del self._active[pair]
            elif d < self.threshold_m:
                self._active[pair] = len(self.events)
                self.events.append({"t": round(t, 1), "pair": pair, "min_distance_m": round(d, 3)})


def summarize_fleet(missions: dict[str, dict], monitor: dict | None) -> dict:
    """Merge per-robot mission reports (from mission_runner) with fleet_monitor output."""
    monitor = monitor or {}
    distance = monitor.get("distance_m", {})
    robots = {}
    for name, rep in sorted(missions.items()):
        robots[name] = {
            "succeeded": rep["succeeded"],
            "total": rep["total"],
            "mission_sim_s": round(rep["end_sim_s"] - rep["start_sim_s"], 1),
            "distance_m": distance.get(name),
            "failed_goals": [g["goal"] for g in rep["goals"] if g["result"] != "SUCCEEDED"],
        }
    starts = [rep["start_sim_s"] for rep in missions.values()]
    ends = [rep["end_sim_s"] for rep in missions.values()]
    makespan = round(max(ends) - min(starts), 1) if missions else 0.0
    succeeded = sum(r["succeeded"] for r in robots.values())
    return {
        "robots": robots,
        "succeeded": succeeded,
        "total": sum(r["total"] for r in robots.values()),
        "makespan_sim_s": makespan,
        "goals_per_min": round(60.0 * succeeded / makespan, 2) if makespan > 0 else 0.0,
        "near_misses": len(monitor.get("near_miss_events", [])),
        "near_miss_threshold_m": monitor.get("near_miss_threshold_m"),
        "min_separation_m": monitor.get("min_separation_m", {}),
    }


def markdown_table(summary: dict) -> str:
    lines = ["| Robot | Goals | Mission time (sim s) | Distance (m) |", "|---|---|---|---|"]
    for name, r in summary["robots"].items():
        dist = "n/a" if r["distance_m"] is None else f"{r['distance_m']:.1f}"
        lines.append(f"| {name} | {r['succeeded']}/{r['total']} | {r['mission_sim_s']} | {dist} |")
    seps = summary["min_separation_m"]
    closest = f"{min(seps.values()):.2f} m" if seps else "n/a"
    lines += [
        "",
        f"Fleet: {summary['succeeded']}/{summary['total']} goals, makespan "
        f"{summary['makespan_sim_s']} s sim, {summary['goals_per_min']} goals/min, "
        f"{summary['near_misses']} near misses (< {summary['near_miss_threshold_m']} m), "
        f"closest approach {closest}.",
    ]
    return "\n".join(lines)

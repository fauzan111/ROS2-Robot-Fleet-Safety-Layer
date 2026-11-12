"""Score the camera safety layer against simulation ground truth (no ROS; unit-tested).

Inputs: a safety log (records written by safety_recorder) and the people file. The robots'
poses come from Gazebo ground truth; each person's position follows exactly from their
scripted waypoints (sim_assets.person_position), so "was someone really in the stop zone?"
has an exact answer at every instant.

Record kinds (dicts with t in sim seconds and r = robot):
  pose:    x, y, yaw         ground-truth robot pose
  state:   s                 safety state (normal/slow/stop/camera_timeout), on change
  persons: pts [[x, y], ...] detector's people in base_link, one record per processed frame

Metrics per robot:
- stop latency: a person truly enters the stop zone -> first STOP onset (command) and ->
  robot speed below 0.02 m/s (halt). Entries while already stopped are counted separately.
- missed: a true stop-zone entry with no STOP before the person leaves the zone (+0.5 s).
- false stops: STOP onsets with no person truly within the stop zone + margin around then.
- recall by distance and localisation error, per processed frame.
"""

from __future__ import annotations

import bisect
import math
from collections import defaultdict

from fleet_bringup.sim_assets import person_position
from fleet_ml.zone import point_in_polygon


def to_robot_frame(pose: tuple[float, float, float], p: tuple[float, float]):
    x, y, yaw = pose
    dx, dy = p[0] - x, p[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def expand_zone(zone, margin):
    xs, ys = [p[0] for p in zone], [p[1] for p in zone]
    return [(min(xs), min(ys) - margin), (max(xs) + margin, min(ys) - margin),
            (max(xs) + margin, max(ys) + margin), (min(xs), max(ys) + margin)]


def _percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    k = (len(v) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return round(v[lo] + (v[hi] - v[lo]) * (k - lo), 3)


def _summary(values):
    return {"n": len(values), "median": _percentile(values, 50), "p95": _percentile(values, 95),
            "max": round(max(values), 3) if values else None}


class _Robot:
    def __init__(self, records, people):
        self.poses = sorted((r["t"], r["x"], r["y"], r["yaw"]) for r in records
                            if r["k"] == "pose")
        self.times = [p[0] for p in self.poses]
        self.states = sorted((r["t"], r["s"]) for r in records if r["k"] == "state")
        self.frames = sorted((r["t"], r["pts"]) for r in records if r["k"] == "persons")
        self.people = people

    def pose_at(self, t):
        i = min(max(bisect.bisect_left(self.times, t), 0), len(self.times) - 1)
        if i > 0 and abs(self.times[i - 1] - t) < abs(self.times[i] - t):
            i -= 1
        return self.poses[i][1:]

    def people_rel(self, t, offset=0.0):
        pose = self.pose_at(t)
        return [to_robot_frame(pose, person_position(w, t + offset)) for w in self.people]

    def state_at(self, t):
        i = bisect.bisect_right([s[0] for s in self.states], t) - 1
        return self.states[i][1] if i >= 0 else None

    def speed_series(self, window_s=0.2):
        out, j = [], 0
        for i, (t, x, y, _) in enumerate(self.poses):
            while self.poses[j][0] < t - window_s:
                j += 1
            t0, x0, y0, _ = self.poses[j]
            out.append((t, math.hypot(x - x0, y - y0) / (t - t0) if t > t0 else 0.0))
        return out


def evaluate(records, people: dict[str, list[dict]], stop_zone, hfov=1.047, max_range=4.0,
             match_m=0.5, false_stop_margin=0.3, halted_mps=0.02) -> dict:
    by_robot = defaultdict(list)
    for r in records:
        by_robot[r["r"]].append(r)
    waypoints = list(people.values())
    near_zone = expand_zone(stop_zone, false_stop_margin)
    result, totals = {}, defaultdict(list)
    for name, recs in sorted(by_robot.items()):
        rb = _Robot(recs, waypoints)
        if not rb.poses:
            continue
        onsets = [t for (t, s), prev in zip(rb.states, [(None, None)] + rb.states[:-1])
                  if s == "stop" and prev[1] != "stop"]
        speeds = rb.speed_series()

        # true stop-zone intervals from ground truth, sampled at pose times
        intervals, start = [], None
        for t, *_ in rb.poses:
            inside = bool(waypoints) and any(point_in_polygon(p, stop_zone)
                                             for p in rb.people_rel(t))
            if inside and start is None:
                start = t
            elif not inside and start is not None:
                intervals.append((start, t))
                start = None
        if start is not None:
            intervals.append((start, rb.poses[-1][0]))

        cmd_lat, halt_lat, missed, already = [], [], [], 0
        for t_in, t_out in intervals:
            if rb.state_at(t_in) == "stop":
                already += 1
                continue
            onset = next((o for o in onsets if t_in - 1.0 <= o <= t_out + 0.5), None)
            if onset is None:
                missed.append(round(t_in, 1))
                continue
            cmd_lat.append(onset - t_in)
            halt = next((t for t, v in speeds if t >= onset and v < halted_mps), None)
            if halt is not None:
                halt_lat.append(halt - t_in)

        false_stops = []
        for o in onsets:
            window = [t for t in rb.times if o - 1.0 <= t <= o + 0.5] or [o]
            if not any(point_in_polygon(p, near_zone) for t in window
                       for p in (rb.people_rel(t) if waypoints else [])):
                false_stops.append(round(o, 1))

        # per-frame recall by distance and localisation error
        seen, total, errors = defaultdict(int), defaultdict(int), []
        for t, pts in rb.frames:
            for gx, gy in (rb.people_rel(t) if waypoints else []):
                d = math.hypot(gx, gy)
                if gx <= 0 or abs(math.atan2(gy, gx)) > hfov / 2 or not 0.3 < d <= max_range:
                    continue
                b = f"{int(d)}-{int(d) + 1} m"
                total[b] += 1
                near = [math.hypot(px - gx, py - gy) for px, py in pts]
                if near and min(near) <= match_m:
                    seen[b] += 1
                    errors.append(min(near))

        hours = (rb.poses[-1][0] - rb.poses[0][0]) / 3600.0
        result[name] = {
            "operating_s": round(hours * 3600, 1),
            "true_stop_zone_entries": len(intervals),
            "entries_while_already_stopped": already,
            "stop_command_latency_s": _summary(cmd_lat),
            "halt_latency_s": _summary(halt_lat),
            "missed_entries_t": missed,
            "false_stops_t": false_stops,
            "recall_by_distance": {b: f"{seen[b]}/{total[b]}" for b in sorted(total)},
            "localisation_error_m": _summary(errors),
        }
        for k, v in (("cmd", cmd_lat), ("halt", halt_lat), ("err", errors)):
            totals[k] += v
        totals["missed"] += missed
        totals["false"] += false_stops
        totals["entries"].append(len(intervals))
        totals["hours"].append(hours)
    hours = sum(totals["hours"])
    result["fleet"] = {
        "operating_robot_hours": round(hours, 4),
        "true_stop_zone_entries": sum(totals["entries"]),
        "stop_command_latency_s": _summary(totals["cmd"]),
        "halt_latency_s": _summary(totals["halt"]),
        "missed_entries": len(totals["missed"]),
        "false_stops": len(totals["false"]),
        "false_stops_per_robot_hour": round(len(totals["false"]) / hours, 1) if hours else None,
        "localisation_error_m": _summary(totals["err"]),
    }
    return result

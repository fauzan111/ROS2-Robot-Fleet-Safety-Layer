"""Safety-layer scoring against ground truth (fleet_bringup.safety_eval)."""

import math

import pytest
from fleet_bringup.safety_eval import evaluate, to_robot_frame

from fleet_ml.perception import rectangle_zone

STOP = rectangle_zone(1.0, 0.4)
# Person walks across 0.8 m in front of a parked robot: up (y -2 -> 2) in 4 s, back in 4 s.
PEOPLE = {"p": [{"t": 0, "x": 0.8, "y": -2.0}, {"t": 4, "x": 0.8, "y": 2.0},
                {"t": 8, "x": 0.8, "y": -2.0}]}


def _log(states, frames=()):
    recs = [{"k": "pose", "r": "robot1", "t": round(i * 0.05, 2), "x": 0.0, "y": 0.0, "yaw": 0.0}
            for i in range(181)]  # 0 .. 9 s (entries at 1.6 s and 5.6 s; the next is 9.6 s)
    recs += [{"k": "state", "r": "robot1", "t": t, "s": s} for t, s in states]
    recs += [{"k": "persons", "r": "robot1", "t": t, "pts": pts} for t, pts in frames]
    return recs


def test_to_robot_frame_rotates_into_heading():
    assert to_robot_frame((1.0, 1.0, math.pi / 2), (1.0, 3.0)) == pytest.approx((2.0, 0.0))


def test_latency_missed_and_false_stops():
    # True entries: t=1.6 (up) and t=5.6 (back). Stop 0.3 s after the first; none for the
    # second (missed); a stop at 8.2 with the person >= 1.2 m to the side is false.
    states = [(0.0, "normal"), (1.9, "stop"), (3.5, "normal"), (8.2, "stop"), (9.5, "normal")]
    r = evaluate(_log(states), PEOPLE, STOP)["robot1"]
    assert r["true_stop_zone_entries"] == 2
    assert r["stop_command_latency_s"]["median"] == pytest.approx(0.3, abs=0.06)
    assert r["halt_latency_s"]["median"] == pytest.approx(0.3, abs=0.06)  # parked: halted
    assert r["missed_entries_t"] == [pytest.approx(5.6, abs=0.11)]  # one pose sample + rounding
    assert r["false_stops_t"] == [8.2]


def test_entry_while_already_stopped_is_not_a_latency_sample():
    r = evaluate(_log([(0.0, "stop")]), PEOPLE, STOP)["robot1"]
    assert r["entries_while_already_stopped"] == 2
    assert r["stop_command_latency_s"]["n"] == 0
    assert r["missed_entries_t"] == []


def test_recall_by_distance_and_localisation_error():
    frames = [(1.0, [[0.8, -1.0]]),   # person at 51 deg: outside the 60 deg FOV, not counted
              (2.0, [[0.9, 0.05]]),   # person straight ahead at 0.8 m, seen 0.11 m off
              (6.0, [])]              # person straight ahead again, not seen
    r = evaluate(_log([(0.0, "normal")], frames), PEOPLE, STOP)["robot1"]
    assert r["recall_by_distance"] == {"0-1 m": "1/2"}
    assert r["localisation_error_m"]["median"] == pytest.approx(math.hypot(0.1, 0.05), abs=1e-3)


def test_no_people_means_every_stop_is_false():
    r = evaluate(_log([(0.0, "normal"), (3.0, "stop"), (4.0, "normal")]), {}, STOP)
    assert r["robot1"]["false_stops_t"] == [3.0]
    assert r["fleet"]["false_stops_per_robot_hour"] == pytest.approx(400.0)  # 1 in 9 s

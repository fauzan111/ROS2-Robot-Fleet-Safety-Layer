"""Fleet logic that needs no ROS: params building, metrics, mission files, report merge."""

import math

import pytest
from fleet_bringup.fleet_metrics import (
    NearMissCounter,
    PathLength,
    markdown_table,
    odom_to_world,
    summarize_fleet,
)
from fleet_bringup.missions import mission_for
from fleet_bringup.params import fleet_params

# --- params


MULTI = {
    "amcl": {"ros__parameters": {"scan_topic": "scan"}},
    "collision_monitor": {"ros__parameters": {
        "FootprintApproach": {"footprint_topic": "/local_costmap/published_footprint"},
        "scan": {"topic": "<robot_namespace>/scan"},
    }},
}
SINGLE = {"amcl": {"ros__parameters": {"scan_topic": "/scan"}},
          "route_server": {"ros__parameters": {"radius_to_achieve_node": 2.0}}}


def test_fleet_params_fills_missing_nodes_from_single_robot_file():
    out = fleet_params(MULTI, SINGLE, {}, {"x": 1.0, "y": 2.0, "yaw": 0.5})
    assert out["route_server"] == SINGLE["route_server"]
    assert out["amcl"]["ros__parameters"]["scan_topic"] == "scan"  # multi wins where present


def test_fleet_params_makes_costmap_topics_follow_the_namespace():
    out = fleet_params(MULTI, SINGLE, {}, {"x": 0, "y": 0})
    cm = out["collision_monitor"]["ros__parameters"]
    assert cm["FootprintApproach"]["footprint_topic"] == "local_costmap/published_footprint"
    assert cm["scan"]["topic"] == "<robot_namespace>/scan"  # left for bringup to replace


def test_fleet_params_sets_amcl_initial_pose_per_robot_and_keeps_inputs_intact():
    out = fleet_params(MULTI, SINGLE, {}, {"x": -2, "y": -0.5, "yaw": 1.57})
    amcl = out["amcl"]["ros__parameters"]
    assert amcl["set_initial_pose"] is True
    assert amcl["initial_pose"] == {"x": -2.0, "y": -0.5, "z": 0.0, "yaw": 1.57}
    assert "set_initial_pose" not in MULTI["amcl"]["ros__parameters"]


# --- metrics


def test_odom_to_world_rotates_by_spawn_yaw():
    x, y = odom_to_world({"x": 1.0, "y": 2.0, "yaw": math.pi / 2}, 1.0, 0.0)
    assert x == pytest.approx(1.0) and y == pytest.approx(3.0)


def test_path_length_ignores_jitter_but_accumulates_slow_motion():
    p = PathLength(min_step_m=0.01)
    p.update(0.0, 0.0)
    for _ in range(100):
        p.update(0.0, 0.0005)  # jitter around one point
    assert p.total_m == 0.0
    for i in range(1, 101):
        p.update(i * 0.004, 0.0)  # 4 mm steps, below the threshold one by one
    assert p.total_m == pytest.approx(0.4, abs=0.02)


def test_near_miss_counts_one_event_per_close_pass_with_hysteresis():
    nm = NearMissCounter(threshold_m=0.5, clear_m=0.6)
    for t, d in enumerate([1.0, 0.45, 0.40, 0.55, 0.48, 0.30, 0.65, 0.45]):
        nm.update(float(t), {"r1": (0.0, 0.0), "r2": (d, 0.0)})
    # 0.45 -> enters; 0.55/0.48/0.30 stay inside (not past 0.6); 0.65 clears; 0.45 -> new event
    assert [e["t"] for e in nm.events] == [1.0, 7.0]
    assert nm.events[0]["min_distance_m"] == 0.3
    assert nm.min_separation_m["r1-r2"] == pytest.approx(0.30)


def test_near_miss_tracks_every_pair():
    nm = NearMissCounter()
    nm.update(0.0, {"a": (0, 0), "b": (0.2, 0), "c": (5, 5)})
    assert set(nm.min_separation_m) == {"a-b", "a-c", "b-c"}
    assert [e["pair"] for e in nm.events] == ["a-b"]


# --- missions and report


def test_mission_for_reads_both_file_formats():
    single = {"initial_pose": {"x": 1}, "goals": [{"name": "g"}]}
    fleet = {"robots": {"robot1": {"spawn": {"x": 2}, "goals": [{"name": "h"}]}}}
    assert mission_for(single) == ({"x": 1}, [{"name": "g"}])
    assert mission_for(fleet, "robot1") == ({"x": 2}, [{"name": "h"}])
    with pytest.raises(KeyError, match="robot9"):
        mission_for(fleet, "robot9")


def _report(succeeded, total, start, end, failed=()):
    goals = [{"goal": f"g{i}", "result": "SUCCEEDED"} for i in range(succeeded)]
    goals += [{"goal": name, "result": "FAILED"} for name in failed]
    return {"succeeded": succeeded, "total": total, "start_sim_s": start, "end_sim_s": end,
            "goals": goals}


def test_summarize_fleet_merges_reports_and_monitor():
    missions = {"robot1": _report(5, 5, 10.0, 60.0),
                "robot2": _report(3, 4, 12.0, 90.0, failed=["east"])}
    monitor = {"distance_m": {"robot1": 12.3, "robot2": 9.9}, "near_miss_threshold_m": 0.5,
               "near_miss_events": [{"t": 40.0, "pair": "robot1-robot2"}],
               "min_separation_m": {"robot1-robot2": 0.42}}
    s = summarize_fleet(missions, monitor)
    assert (s["succeeded"], s["total"]) == (8, 9)
    assert s["makespan_sim_s"] == 80.0
    assert s["goals_per_min"] == 6.0
    assert s["near_misses"] == 1
    assert s["robots"]["robot2"]["failed_goals"] == ["east"]
    table = markdown_table(s)
    assert "| robot1 | 5/5 | 50.0 | 12.3 |" in table
    assert "closest approach 0.42 m" in table

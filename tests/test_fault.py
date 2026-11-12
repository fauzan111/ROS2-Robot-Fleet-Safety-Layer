"""Fault-injection scoring and frame prefixing (fleet_bringup.fault)."""

from fleet_bringup.fault import analyze_fault, prefixed


def test_prefixed_is_idempotent_and_strips_leading_slash():
    assert prefixed("base_link", "robot1") == "robot1/base_link"
    assert prefixed("/map", "robot2") == "robot2/map"
    assert prefixed("robot1/odom", "robot1") == "robot1/odom"
    assert prefixed("", "robot1") == ""


def _series(t0, t1, value, dt=0.05):
    n = round((t1 - t0) / dt)
    return [(round(t0 + i * dt, 3), value) for i in range(n)]


def _run(gate_out, target_speed, other_speed=0.2):
    # kill at 10 s, restart at 15 s, end at 20 s
    speeds = {"robot2": target_speed, "robot1": _series(5, 20, other_speed),
              "robot3": _series(5, 20, other_speed)}
    return analyze_fault("robot2", 10.0, 15.0, 20.0, gate_out, speeds)


GOOD_GATE = _series(5, 10.4, 0.2) + _series(10.4, 15, 0.0) + _series(15.3, 20, 0.2)
GOOD_SPEED = _series(5, 10.6, 0.2) + _series(10.6, 15.5, 0.0) + _series(15.5, 20, 0.2)


def test_clean_fail_safe_run_passes():
    r = _run(GOOD_GATE, GOOD_SPEED)
    assert r["passed"], r["checks"]
    assert r["gate_stop_latency_s"] == 0.4
    assert r["robot_halt_latency_s"] == 0.6
    assert r["resume_latency_s"] == 0.3
    assert r["others_distance_during_fault_m"] == {"robot1": 1.0, "robot3": 1.0}  # 0.2 m/s for 5 s


def test_gate_that_keeps_passing_commands_fails():
    leaky = _series(5, 20, 0.2)
    r = _run(leaky, _series(5, 20, 0.2))
    assert not r["checks"]["gate_zeroed_within_timeout"]
    assert not r["checks"]["target_halted"]
    assert not r["passed"]


def test_late_stop_beyond_timeout_fails():
    late = _series(5, 11.0, 0.2) + _series(11.0, 15, 0.0) + _series(15.3, 20, 0.2)
    r = _run(late, GOOD_SPEED)
    assert r["gate_stop_latency_s"] == 1.0
    assert not r["checks"]["gate_zeroed_within_timeout"]


def test_other_robots_stopping_too_fails():
    r = _run(GOOD_GATE, GOOD_SPEED, other_speed=0.0)
    assert not r["checks"]["others_kept_driving"]


def test_no_resume_after_restart_fails():
    stuck = _series(5, 10.4, 0.2) + _series(10.4, 20, 0.0)
    r = _run(stuck, GOOD_SPEED)
    assert r["checks"]["gate_zeroed_within_timeout"]
    assert not r["checks"]["target_resumed_after_restart"]

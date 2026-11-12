"""Fault-injection analysis and frame naming, with no ROS dependency (unit-tested).

The experiment: while the fleet drives, kill one robot's safety_monitor (SIGKILL, like a
crash), watch for a while, restart it, watch again. Pass criteria:

- the target's speed_gate cuts its output to zero within the heartbeat timeout (+ margin)
  and keeps it at zero until the monitor is back;
- the target robot actually halts;
- every other robot keeps driving (its own safety chain is untouched);
- after the restart, the target's gate passes commands again.
"""

from __future__ import annotations

EPS = 1e-3


def prefixed(frame_id: str, robot: str) -> str:
    """robot1 + 'base_link' -> 'robot1/base_link' (idempotent; empty stays empty)."""
    frame = frame_id.lstrip("/")
    if not frame or frame.startswith(f"{robot}/"):
        return frame
    return f"{robot}/{frame}"


def _distance(samples: list[tuple[float, float]], t0: float, t1: float) -> float:
    """Integrate speed samples (t, m/s) over [t0, t1] with a left Riemann sum."""
    window = [(t, v) for t, v in samples if t0 <= t <= t1]
    return sum(v0 * (t1_ - t0_) for (t0_, v0), (t1_, _) in zip(window, window[1:]))


def analyze_fault(
    target: str,
    t_kill: float,
    t_restart: float,
    t_end: float,
    gate_out: list[tuple[float, float]],
    speeds: dict[str, list[tuple[float, float]]],
    timeout_s: float = 0.5,
    margin_s: float = 0.25,
    halted_mps: float = 0.02,
    others_min_m: float = 0.3,
) -> dict:
    """Score one fault-injection run.

    gate_out: (t, |command|) of everything the target's speed_gate published.
    speeds:   robot -> [(t, |velocity| from odometry)].
    """
    dead = [(t, m) for t, m in gate_out if t_kill <= t < t_restart]
    stop_t = next((t for i, (t, _) in enumerate(dead) if all(m < EPS for _, m in dead[i:])),
                  None)
    stop_latency = None if stop_t is None else round(stop_t - t_kill, 3)

    target_speed = speeds.get(target, [])
    halt_t = None
    if stop_t is not None:
        after = [(t, v) for t, v in target_speed if stop_t <= t < t_restart]
        halt_t = next((t for i, (t, _) in enumerate(after)
                       if all(v < halted_mps for _, v in after[i:])), None)

    resumed = next((t for t, m in gate_out if t >= t_restart and m >= EPS), None)
    others = {r: round(_distance(s, t_kill, t_restart), 2)
              for r, s in sorted(speeds.items()) if r != target}

    checks = {
        "gate_zeroed_within_timeout": stop_latency is not None
        and stop_latency <= timeout_s + margin_s,
        "target_halted": halt_t is not None,
        "others_kept_driving": bool(others) and all(d >= others_min_m for d in others.values()),
        "target_resumed_after_restart": resumed is not None and resumed <= t_end,
    }
    return {
        "target": target,
        "heartbeat_timeout_s": timeout_s,
        "fault_window_s": round(t_restart - t_kill, 2),
        "gate_stop_latency_s": stop_latency,
        "robot_halt_latency_s": None if halt_t is None else round(halt_t - t_kill, 3),
        "resume_latency_s": None if resumed is None else round(resumed - t_restart, 3),
        "others_distance_during_fault_m": others,
        "checks": checks,
        "passed": all(checks.values()),
    }

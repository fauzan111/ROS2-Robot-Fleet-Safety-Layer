"""Fault injection: crash one robot's safety_monitor mid-mission and score the fail-safe.

    ros2 run fleet_bringup fault_injection --ros-args -p use_sim_time:=true \
        -p fleet_file:=/ws/src/fleet_bringup/config/fleet.yaml -p target:=robot2

1. Wait until every robot has been driving for 1 s (sim time).
2. SIGKILL the target's safety_monitor (no clean shutdown, like a real crash).
3. Watch for window_s, then restart the monitor and watch for window_s again.
4. Score with fleet_bringup.fault.analyze_fault and write results/fault_injection_<ts>.json.
"""

from __future__ import annotations

import json
import math
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fleet_bringup.fault import analyze_fault


class FaultInjection(Node):
    def __init__(self) -> None:
        super().__init__("fault_injection")
        fleet_file = self.declare_parameter("fleet_file", "").value
        self.target = self.declare_parameter("target", "robot2").value
        self.window_s = float(self.declare_parameter("window_s", 5.0).value)
        self.wait_limit_s = float(self.declare_parameter("wait_limit_s", 300.0).value)
        self.results_dir = Path(self.declare_parameter("results_dir", "/ws/results").value)

        robots = list(yaml.safe_load(Path(fleet_file).read_text())["robots"])
        if self.target not in robots:
            raise ValueError(f"target {self.target} not in fleet {robots}")
        self.speeds: dict[str, list[tuple[float, float]]] = {r: [] for r in robots}
        self.gate_out: list[tuple[float, float]] = []
        for r in robots:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda msg, r=r: self.on_odom(r, msg), 50)
        self.create_subscription(Twist, f"/{self.target}/cmd_vel_safe", self.on_gate, 50)

        self.state = "wait"
        self.wall0 = time.monotonic()
        self.moving_since: float | None = None
        self.t_kill = self.t_restart = math.nan
        self.killed_pids: list[str] = []
        self.monitor_proc: subprocess.Popen | None = None
        self.done = False
        self.create_timer(0.05, self.step)
        self.get_logger().info(f"fault_injection armed: target {self.target}, "
                               f"window {self.window_s} s")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, robot: str, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self.speeds[robot].append((self.now_s(), math.hypot(v.x, v.y)))

    def on_gate(self, msg: Twist) -> None:
        m = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        self.gate_out.append((self.now_s(), m))

    def all_moving(self) -> bool:
        return all(s and s[-1][1] > 0.05 for s in self.speeds.values())

    def step(self) -> None:
        if self.done:
            return
        t = self.now_s()
        if self.state == "wait":
            if time.monotonic() - self.wall0 > self.wait_limit_s:
                self.finish(error="robots were never all moving at once")
            elif self.all_moving():
                self.moving_since = self.moving_since if self.moving_since is not None else t
                if t - self.moving_since >= 1.0:
                    self.kill()
            else:
                self.moving_since = None
        elif self.state == "dead" and t - self.t_kill >= self.window_s:
            self.restart()
        elif self.state == "restarted" and t - self.t_restart >= self.window_s:
            self.finish()

    def kill(self) -> None:
        pattern = f"safety_monitor --ros-args.*__ns:=/{self.target}( |$)"
        self.killed_pids = subprocess.run(["pgrep", "-f", pattern], capture_output=True,
                                          text=True).stdout.split()
        subprocess.run(["pkill", "-KILL", "-f", pattern], check=False)
        self.t_kill = self.now_s()
        self.state = "dead"
        self.get_logger().warn(f"KILLED {self.target}/safety_monitor (pids {self.killed_pids}) "
                               f"at t={self.t_kill:.2f} s")

    def restart(self) -> None:
        self.monitor_proc = subprocess.Popen([
            "ros2", "run", "fleet_safety", "safety_monitor", "--ros-args",
            "-r", f"__ns:=/{self.target}", "-r", "__node:=safety_monitor",
            "-r", "/tf:=tf", "-r", "/tf_static:=tf_static",
            "-p", "use_sim_time:=true", "-p", "perception:=true"])  # same config as launch
        self.t_restart = self.now_s()
        self.state = "restarted"
        self.get_logger().warn(f"restarted {self.target}/safety_monitor at "
                               f"t={self.t_restart:.2f} s")

    def finish(self, error: str | None = None) -> None:
        self.done = True
        if error:
            result = {"target": self.target, "passed": False, "error": error}
        else:
            result = analyze_fault(self.target, self.t_kill, self.t_restart, self.now_s(),
                                   self.gate_out, self.speeds)
            result["killed_pids"] = self.killed_pids
            result["checks"]["monitor_was_killed"] = bool(self.killed_pids)
            result["passed"] = result["passed"] and bool(self.killed_pids)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        out = self.results_dir / f"fault_injection_{datetime.now():%Y%m%d_%H%M%S}.json"
        out.write_text(json.dumps(result, indent=2))
        self.get_logger().info(f"{'PASS' if result['passed'] else 'FAIL'}: {json.dumps(result)}")
        self.get_logger().info(f"Results: {out}")


def main() -> None:
    rclpy.init()
    node = FaultInjection()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node.monitor_proc is not None:
            node.monitor_proc.terminate()
        node.destroy_node()
        rclpy.try_shutdown()

"""Demo director: script the events of the demo video on top of a running fleet.

    ros2 run fleet_bringup demo_director --ros-args -p use_sim_time:=true \
        -p fleet_file:=/ws/src/fleet_bringup/config/demo.yaml

Once every robot has started driving (sim time, t0):
  t0 + kill_at_s     SIGKILL fault_robot's safety_monitor  -> its gate must stop it alone
  + fault_window_s   restart the monitor                   -> it drives on
  t0 + fog_at_s      fog fog_robot's camera (ood_inject)    -> OOD -> SLOW
  + fog_window_s     clear the fog
Each event is published on /demo/event (std_msgs/String, JSON with sim time) so the video
renderer can caption it from the same bag.
"""

from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fleet_bringup.fault_injection import kill_monitor, restart_monitor


class DemoDirector(Node):
    def __init__(self) -> None:
        super().__init__("demo_director")
        p = self.declare_parameter
        self.robots = list(yaml.safe_load(open(p("fleet_file", "").value))["robots"])
        self.fault_robot = p("fault_robot", "robot2").value
        self.fog_robot = p("fog_robot", "robot3").value
        kill_at = float(p("kill_at_s", 12.0).value)
        fault_window = float(p("fault_window_s", 6.0).value)
        fog_at = float(p("fog_at_s", 30.0).value)
        fog_window = float(p("fog_window_s", 18.0).value)
        self.plan = sorted([
            (kill_at, self.kill),
            (kill_at + fault_window, self.restart),
            (fog_at, lambda: self.fog("fog")),
            (fog_at + fog_window, lambda: self.fog("none")),
        ], key=lambda e: e[0])
        self.pub = self.create_publisher(
            String, "/demo/event", QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.speed = {r: 0.0 for r in self.robots}
        for r in self.robots:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda m, r=r: self.speed.__setitem__(
                                         r, abs(m.twist.twist.linear.x)), 10)
        self.t0: float | None = None
        self.started: set[str] = set()
        self.procs: list[subprocess.Popen] = []
        self.wall0 = time.monotonic()
        self.done = False
        self.create_timer(0.1, self.step)
        self.get_logger().info(f"demo_director armed: fault {self.fault_robot}, "
                               f"fog {self.fog_robot}")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def event(self, kind: str, robot: str, **extra) -> None:
        payload = {"t": round(self.now_s(), 2), "kind": kind, "robot": robot, **extra}
        self.pub.publish(String(data=json.dumps(payload)))
        self.get_logger().warn(f"demo event: {payload}")

    def step(self) -> None:
        t = self.now_s()
        if self.t0 is None:
            # Robots pause (people, turns), so wait for each to have driven once, not all at once.
            self.started |= {r for r, v in self.speed.items() if v > 0.05}
            if len(self.started) == len(self.robots):
                self.t0 = t
                self.event("start", "fleet")
            elif time.monotonic() - self.wall0 > 300:
                self.get_logger().error(f"only {sorted(self.started)} ever drove; no demo events")
                self.done = True
            return
        while self.plan and t - self.t0 >= self.plan[0][0]:
            self.plan.pop(0)[1]()
        if not self.plan:
            self.done = True

    def kill(self) -> None:
        pids = kill_monitor(self.fault_robot)
        self.event("monitor_killed", self.fault_robot, pids=pids)

    def restart(self) -> None:
        others = [r for r in self.robots if r != self.fault_robot]
        self.procs.append(restart_monitor(self.fault_robot, others,
                                          Path("/ws/results/demo_restarted_monitor.log")))
        self.event("monitor_restarted", self.fault_robot)

    def fog(self, value: str) -> None:
        self.procs.append(subprocess.Popen(
            ["ros2", "param", "set", f"/{self.fog_robot}/safety_monitor", "ood_inject", value],
            stdout=subprocess.DEVNULL))
        self.event("fog_on" if value != "none" else "fog_off", self.fog_robot)


def main() -> None:
    rclpy.init()
    node = DemoDirector()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Keep spinning (and the restarted monitor alive) until the run script stops us.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for proc in node.procs:
            proc.terminate()
        node.destroy_node()
        rclpy.try_shutdown()

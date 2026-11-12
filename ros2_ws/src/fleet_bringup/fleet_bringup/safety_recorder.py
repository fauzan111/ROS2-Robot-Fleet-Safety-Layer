"""Record what the safety layer saw and did, next to ground truth, for safety_eval.

Writes JSON lines to <results_dir>/safety_log_<tag>.jsonl (records described in
fleet_bringup.safety_eval): ground-truth poses, safety-state changes and every processed
frame's person estimates, for each robot in the fleet file. Runs until stopped.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


class SafetyRecorder(Node):
    def __init__(self) -> None:
        super().__init__("safety_recorder")
        fleet_file = self.declare_parameter("fleet_file", "").value
        tag = self.declare_parameter("tag", "run").value
        results = Path(self.declare_parameter("results_dir", "/ws/results").value)
        results.mkdir(parents=True, exist_ok=True)
        self.path = results / f"safety_log_{tag}.jsonl"
        self.out = self.path.open("w", buffering=1)  # line-buffered: survives a hard stop
        self.last_state: dict[str, str] = {}
        for r in yaml.safe_load(Path(fleet_file).read_text())["robots"]:
            self.create_subscription(PoseStamped, f"/{r}/ground_truth/pose",
                                     lambda m, r=r: self.on_pose(r, m), 20)
            self.create_subscription(String, f"/{r}/safety/state",
                                     lambda m, r=r: self.on_state(r, m), 20)
            self.create_subscription(PoseArray, f"/{r}/safety/persons",
                                     lambda m, r=r: self.on_persons(r, m), 20)
        self.get_logger().info(f"recording to {self.path}")

    def write(self, rec: dict) -> None:
        self.out.write(json.dumps(rec) + "\n")

    def on_pose(self, robot: str, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.write({"k": "pose", "r": robot, "t": round(_stamp(msg), 3),
                    "x": round(msg.pose.position.x, 4), "y": round(msg.pose.position.y, 4),
                    "yaw": round(yaw, 4)})

    def on_state(self, robot: str, msg: String) -> None:
        if self.last_state.get(robot) != msg.data:
            self.last_state[robot] = msg.data
            t = self.get_clock().now().nanoseconds / 1e9
            self.write({"k": "state", "r": robot, "t": round(t, 3), "s": msg.data})

    def on_persons(self, robot: str, msg: PoseArray) -> None:
        pts = [[round(p.position.x, 3), round(p.position.y, 3)] for p in msg.poses]
        self.write({"k": "persons", "r": robot, "t": round(_stamp(msg), 3), "pts": pts})


def main() -> None:
    rclpy.init()
    node = SafetyRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.out.close()
        node.destroy_node()
        rclpy.try_shutdown()

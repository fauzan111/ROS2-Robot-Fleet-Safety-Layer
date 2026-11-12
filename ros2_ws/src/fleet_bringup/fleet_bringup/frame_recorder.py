"""Save camera frames to disk: visual checks now, and the in-distribution dataset for OOD.

    ros2 run fleet_bringup frame_recorder --ros-args -p use_sim_time:=true \
        -p robots:="[robot1, robot2, robot3]" -p count:=20 -p period_s:=2.0 \
        -p out_dir:=/ws/results/frames

Writes <out_dir>/<robot>_<sim seconds>.png (RGB) and, with depth:=true, a matching
_depth.npy (float32 metres). Exits once every robot has `count` frames.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameRecorder(Node):
    def __init__(self) -> None:
        super().__init__("frame_recorder")
        robots = list(self.declare_parameter("robots", ["robot1"]).value)
        self.count = int(self.declare_parameter("count", 10).value)
        self.period_s = float(self.declare_parameter("period_s", 1.0).value)
        self.depth = bool(self.declare_parameter("depth", False).value)
        self.out = Path(self.declare_parameter("out_dir", "/ws/results/frames").value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.saved = {r: 0 for r in robots}
        self.last_t = {r: -1e9 for r in robots}
        self.latest_depth: dict[str, np.ndarray] = {}
        for r in robots:
            self.create_subscription(Image, f"/{r}/camera/image_raw",
                                     lambda m, r=r: self.on_image(r, m), qos_profile_sensor_data)
            if self.depth:
                self.create_subscription(Image, f"/{r}/camera/depth/image_raw",
                                         lambda m, r=r: self.on_depth(r, m),
                                         qos_profile_sensor_data)

    @property
    def done(self) -> bool:
        return all(n >= self.count for n in self.saved.values())

    def on_depth(self, robot: str, msg: Image) -> None:
        self.latest_depth[robot] = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def on_image(self, robot: str, msg: Image) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.saved[robot] >= self.count or t - self.last_t[robot] < self.period_s:
            return
        if self.depth and robot not in self.latest_depth:
            return
        stem = self.out / f"{robot}_{t:09.2f}"
        cv2.imwrite(f"{stem}.png", self.bridge.imgmsg_to_cv2(msg, "bgr8"))
        if self.depth:
            np.save(f"{stem}_depth.npy", self.latest_depth[robot])
        self.last_t[robot] = t
        self.saved[robot] += 1
        self.get_logger().info(f"saved {stem}.png ({self.saved[robot]}/{self.count})")


def main() -> None:
    rclpy.init()
    node = FrameRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

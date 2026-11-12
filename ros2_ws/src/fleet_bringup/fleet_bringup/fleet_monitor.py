"""Watch the whole fleet: distance driven, closest approaches and near misses.

Subscribes to /<robot>/odom for every robot in the fleet file, converts to the world
frame with each spawn pose, and rewrites results/fleet_monitor.json every second
(atomically), so the numbers survive however the run ends.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fleet_bringup.fleet_metrics import NearMissCounter, PathLength, odom_to_world


class FleetMonitor(Node):
    def __init__(self) -> None:
        super().__init__("fleet_monitor")
        fleet_file = self.declare_parameter("fleet_file", "").value
        results_dir = Path(self.declare_parameter("results_dir", "/ws/results").value)
        results_dir.mkdir(parents=True, exist_ok=True)
        self.out = results_dir / "fleet_monitor.json"

        cfg = yaml.safe_load(Path(fleet_file).read_text())
        self.spawn = {name: r["spawn"] for name, r in cfg["robots"].items()}
        threshold = float(cfg.get("near_miss_m", 0.5))
        self.near = NearMissCounter(threshold_m=threshold, clear_m=threshold + 0.1)
        self.paths = {name: PathLength() for name in self.spawn}
        self.pos: dict[str, tuple[float, float]] = {}

        for name in self.spawn:
            self.create_subscription(
                Odometry, f"/{name}/odom", lambda msg, n=name: self.on_odom(n, msg), 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.save)
        self.get_logger().info(f"fleet_monitor watching {sorted(self.spawn)}, "
                               f"near miss < {threshold} m")

    def on_odom(self, name: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        xy = odom_to_world(self.spawn[name], p.x, p.y)
        self.pos[name] = xy
        self.paths[name].update(*xy)

    def tick(self) -> None:
        if len(self.pos) == len(self.spawn):
            n_before = len(self.near.events)
            self.near.update(self.get_clock().now().nanoseconds / 1e9, self.pos)
            for event in self.near.events[n_before:]:
                self.get_logger().warn(f"near miss {event['pair']} at t={event['t']} s")

    def save(self) -> None:
        data = {
            "near_miss_threshold_m": self.near.threshold_m,
            "near_miss_events": self.near.events,
            "min_separation_m": {k: round(v, 3) for k, v in self.near.min_separation_m.items()},
            "distance_m": {k: round(p.total_m, 2) for k, p in self.paths.items()},
        }
        tmp = self.out.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2))
        os.replace(tmp, self.out)


def main() -> None:
    rclpy.init()
    node = FleetMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()

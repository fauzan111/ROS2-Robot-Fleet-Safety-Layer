"""Merge every robot's TF tree into one, so Foxglove can show the whole fleet in one view.

Each robot keeps its own tree on /robotN/tf (Nav2's multi-robot pattern), with plain frame
names (map, odom, base_link). This relay republishes them on the global /tf and /tf_static
with prefixed names (robot1/map, robot1/base_link, ...) under a common `world` frame:

    world -> robot1/map -> robot1/odom -> robot1/base_footprint -> ...
          -> robot2/map -> ...

All robots share one map file, so world -> robotN/map is the identity. Laser scans, planned
paths and maps are relayed to /fleet/robotN/{scan,plan,map} with prefixed frame ids.
The robots' own TF trees are untouched; this node only feeds visualisation.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from fleet_bringup.fault import prefixed


def _latched(depth: int) -> QoSProfile:
    return QoSProfile(depth=depth, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


class FleetTfRelay(Node):
    def __init__(self) -> None:
        super().__init__("fleet_tf_relay")
        fleet_file = self.declare_parameter("fleet_file", "").value
        world = self.declare_parameter("world_frame", "world").value
        robots = list(yaml.safe_load(Path(fleet_file).read_text())["robots"])

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.static_pub = self.create_publisher(TFMessage, "/tf_static", _latched(1))
        self.static: dict[str, TransformStamped] = {}
        self.keep = []  # publishers referenced by relay closures

        for robot in robots:
            anchor = TransformStamped()
            anchor.header.frame_id = world
            anchor.child_frame_id = f"{robot}/map"
            anchor.transform.rotation.w = 1.0
            self.static[anchor.child_frame_id] = anchor

            self.create_subscription(TFMessage, f"/{robot}/tf",
                                     lambda m, r=robot: self.on_tf(r, m), 100)
            self.create_subscription(TFMessage, f"/{robot}/tf_static",
                                     lambda m, r=robot: self.on_tf_static(r, m), _latched(10))
            self.relay(robot, "scan", LaserScan, qos_profile_sensor_data, qos_profile_sensor_data)
            self.relay(robot, "plan", NavPath, 10, 10)
            self.relay(robot, "map", OccupancyGrid, _latched(1), _latched(1))

        self.publish_static()
        self.get_logger().info(f"fleet_tf_relay: {robots} under '{world}'")

    def _prefix_transforms(self, robot: str, msg: TFMessage) -> TFMessage:
        for t in msg.transforms:
            t.header.frame_id = prefixed(t.header.frame_id, robot)
            t.child_frame_id = prefixed(t.child_frame_id, robot)
        return msg

    def on_tf(self, robot: str, msg: TFMessage) -> None:
        self.tf_pub.publish(self._prefix_transforms(robot, msg))

    def on_tf_static(self, robot: str, msg: TFMessage) -> None:
        for t in self._prefix_transforms(robot, msg).transforms:
            self.static[t.child_frame_id] = t
        self.publish_static()

    def publish_static(self) -> None:
        # Latched topics keep only the last message, so always publish the full set.
        stamp = self.get_clock().now().to_msg()
        for t in self.static.values():
            if t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0:
                t.header.stamp = stamp
        self.static_pub.publish(TFMessage(transforms=list(self.static.values())))

    def relay(self, robot: str, topic: str, msg_type, qos_in, qos_out) -> None:
        pub = self.create_publisher(msg_type, f"/fleet/{robot}/{topic}", qos_out)
        self.keep.append(pub)

        def forward(msg, robot=robot, pub=pub):
            msg.header.frame_id = prefixed(msg.header.frame_id, robot)
            for pose in getattr(msg, "poses", []):  # nav_msgs/Path
                pose.header.frame_id = prefixed(pose.header.frame_id, robot)
            pub.publish(msg)

        self.create_subscription(msg_type, f"/{robot}/{topic}", forward, qos_in)


def main() -> None:
    rclpy.init()
    node = FleetTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

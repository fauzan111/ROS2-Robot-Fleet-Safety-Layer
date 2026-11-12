"""Safety monitor node.

Publishes the allowed speed scale on safety/speed_scale (relative, so /robotN/safety/...
in a fleet) as a fixed-rate heartbeat
(the C++ speed_gate stops the robot if the heartbeat goes stale).

Week 1: always 1.0 (no perception yet).
Week 3: the camera callback runs the person detector, zone check and OOD score
        (fleet_ml package) and updates self.scale.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class SafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        rate_hz = self.declare_parameter("rate_hz", 20.0).value
        self.scale = 1.0
        self.pub = self.create_publisher(Float32, "safety/speed_scale", 10)  # relative: per robot
        self.create_subscription(Image, "camera/image_raw", self.on_image, 10)
        self.create_timer(1.0 / rate_hz, self.publish_scale)
        self.get_logger().info(f"safety_monitor started: heartbeat {rate_hz:.0f} Hz, pass-through")

    def on_image(self, msg: Image) -> None:
        pass  # week 3: perception updates self.scale

    def publish_scale(self) -> None:
        self.pub.publish(Float32(data=float(self.scale)))


def main() -> None:
    rclpy.init()
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

"""Safety monitor node.

Subscribes to the camera and publishes the allowed speed scale on /safety/speed_scale.
Week 1: pass-through (always 1.0) so the fleet runs end to end.
Week 3: runs the person detector, zone check and OOD score (fleet_ml package).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class SafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        self.pub = self.create_publisher(Float32, "/safety/speed_scale", 10)
        self.create_subscription(Image, "camera/image_raw", self.on_image, 10)
        self.get_logger().info("safety_monitor started (pass-through mode)")

    def on_image(self, msg: Image) -> None:
        self.pub.publish(Float32(data=1.0))


def main() -> None:
    rclpy.init()
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

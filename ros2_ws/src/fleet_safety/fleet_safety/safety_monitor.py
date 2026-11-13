"""Safety monitor node: camera -> person detector -> protective zones -> speed scale.

Publishes the allowed speed scale on safety/speed_scale (relative, so /robotN/safety/...
in a fleet) as a fixed-rate heartbeat; the C++ speed_gate stops the robot if it goes stale.

With perception:=true (the fleet launch):
- each colour frame runs YOLOv4-tiny (fleet_ml.detector); each person box gets a distance
  from the aligned depth image and is transformed into base_link (fleet_ml.perception);
- a person in the stop zone (default 1.0 m ahead, +-0.4 m) -> STOP; in the warning zone
  (2.0 m, +-0.6 m) -> SLOW (30%); leaving STOP needs 1 s clear (fleet_ml.zone hysteresis);
- near field: any 360 deg lidar return inside the stop zone that is not on the static map
  (and not on another fleet robot, whose poses come from their AMCL) -> STOP. Measured: the
  60 deg camera covers the full stop-zone width only beyond ~0.76 m and missed 4 of 13
  close side entries on its own;
- an out-of-distribution frame (fog, darkness, noise, dirt on the lens: Mahalanobis distance
  on image statistics, fleet_ml/models/ood_stats.json) -> SLOW, since the detector may be
  blind; ood_inject:=<fog|dark|noise|smudge|blur|overexposed> degrades frames for testing;
- no camera frame for camera_timeout_s, or no evaluated scan for lidar_timeout_s -> STOP:
  a blind sensor must not let the robot drive.

With perception:=false (default, the single-robot setup): pass-through scale 1.0.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

COLORS = {"normal": (0, 200, 0), "slow": (0, 200, 255), "stop": (0, 0, 255),
          "camera_timeout": (255, 0, 255), "lidar_timeout": (255, 0, 255)}


class SafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        p = self.declare_parameter
        rate_hz = float(p("rate_hz", 20.0).value)
        self.perception = bool(p("perception", False).value)
        self.scale_pub = self.create_publisher(Float32, "safety/speed_scale", 10)
        self.create_timer(1.0 / rate_hz, self.heartbeat)
        if not self.perception:
            self.get_logger().info(f"safety_monitor: heartbeat {rate_hz:.0f} Hz, pass-through")
            return

        from fleet_ml.detector import PersonDetector
        from fleet_ml.perception import rectangle_zone
        from fleet_ml.zone import SafetyStateMachine

        self.detector = PersonDetector(
            p("model_cfg", "/opt/fleet_models/yolov4-tiny/yolov4-tiny.cfg").value,
            p("model_weights", "/opt/fleet_models/yolov4-tiny/yolov4-tiny.weights").value,
            input_size=int(p("input_size", 416).value),
            conf_threshold=float(p("conf_threshold", 0.3).value),
            threads=int(p("cv_threads", 2).value))
        self.min_period_s = 1.0 / float(p("max_detect_hz", 10.0).value)
        self.last_detect_t = -1e9
        stop = list(p("stop_zone", [1.0, 0.4]).value)      # [length, half width] in metres
        warn = list(p("warning_zone", [2.0, 0.6]).value)
        self.stop_zone, self.warn_zone = rectangle_zone(*stop), rectangle_zone(*warn)
        self.camera_timeout_s = float(p("camera_timeout_s", 0.5).value)
        self.base_frame = p("base_frame", "base_link").value
        self.machine = SafetyStateMachine(clear_for_s=float(p("clear_for_s", 1.0).value))
        import json
        from pathlib import Path as _Path

        from fleet_ml.ood import Mahalanobis
        ood_file = p("ood_model", "/opt/fleet_ml/fleet_ml/models/ood_stats.json").value
        ood_cfg = json.loads(_Path(ood_file).read_text())
        self.ood_model = Mahalanobis.from_dict(ood_cfg["model"])
        self.ood_threshold = float(ood_cfg["threshold"])
        self.ood_inject = p("ood_inject", "").value
        self.ood = False
        self.ood_pub = self.create_publisher(Float32, "safety/ood_score", 10)
        self.lidar = bool(p("lidar_near_field", True).value)
        self.lidar_timeout_s = float(p("lidar_timeout_s", 0.5).value)
        self.lidar_min_points = int(p("lidar_min_points", 3).value)
        self.map_inflate_m = float(p("map_inflate_m", 0.15).value)
        self.exclusion_m = float(p("robot_exclusion_m", 0.35).value)
        self.other_robots = [r for r in p("other_robots", [""]).value if r]
        self.map_frame = p("map_frame", "map").value
        self.static_mask = None
        self.map_info = None
        self.others_xy: dict[str, tuple[float, float]] = {}
        self.lidar_stop = False
        self.last_scan_t: float | None = None

        self.bridge = CvBridge()
        self.k: list[float] | None = None
        self.depth: np.ndarray | None = None
        self.cam_to_base: np.ndarray | None = None
        self.last_frame_t: float | None = None
        self.in_stop = self.in_warn = False
        self.state = "camera_timeout"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_pub = self.create_publisher(String, "safety/state", 10)
        self.annotated_pub = self.create_publisher(Image, "safety/annotated", 2)
        # Every processed frame, even with nobody seen: what the detector believed (base_link).
        self.persons_pub = self.create_publisher(PoseArray, "safety/persons", 10)
        self.create_subscription(CameraInfo, "camera/camera_info", self.on_info, 1)
        self.create_subscription(Image, "camera/depth/image_raw", self.on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "camera/image_raw", self.on_image,
                                 qos_profile_sensor_data)
        if self.lidar:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
            self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
            for other in self.other_robots:
                self.create_subscription(PoseWithCovarianceStamped, f"/{other}/amcl_pose",
                                         lambda m, o=other: self.on_other(o, m), 10)
        self.get_logger().info(f"safety_monitor: heartbeat {rate_hz:.0f} Hz, perception on, "
                               f"stop zone {stop}, warning zone {warn}, lidar near field "
                               f"{'on' if self.lidar else 'off'} (others: {self.other_robots})")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_info(self, msg: CameraInfo) -> None:
        self.k = list(msg.k)

    def on_depth(self, msg: Image) -> None:
        self.depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def on_map(self, msg: OccupancyGrid) -> None:
        from fleet_ml.perception import static_obstacle_mask
        grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.static_mask = static_obstacle_mask(grid, msg.info.resolution, self.map_inflate_m)
        self.map_info = msg.info

    def on_other(self, robot: str, msg: PoseWithCovarianceStamped) -> None:
        self.others_xy[robot] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _lookup(self, target: str, source: str) -> np.ndarray | None:
        from fleet_ml.perception import transform_from_quaternion
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except Exception:  # noqa: BLE001 - not available yet (e.g. AMCL starting)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return transform_from_quaternion((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def on_scan(self, msg: LaserScan) -> None:
        from fleet_ml.perception import scan_to_points, unknown_obstacle_points
        if self.static_mask is None:
            return
        base_scan = self._lookup(self.base_frame, msg.header.frame_id)
        map_base = self._lookup(self.map_frame, self.base_frame)
        if base_scan is None or map_base is None:
            return  # scan not evaluated -> lidar_timeout -> STOP (fail-safe)
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max)
        homo = np.column_stack((pts, np.zeros(len(pts)), np.ones(len(pts))))
        p_base = (base_scan @ homo.T).T
        p_map = (map_base @ p_base.T).T
        o = self.map_info.origin.position
        unknown = unknown_obstacle_points(
            p_base[:, :2], p_map[:, :2], self.stop_zone, self.static_mask, (o.x, o.y),
            self.map_info.resolution, list(self.others_xy.values()), self.exclusion_m)
        self.lidar_stop = len(unknown) >= self.lidar_min_points
        self.last_scan_t = self.now_s()

    def _camera_to_base(self, frame_id: str) -> np.ndarray | None:
        if self.cam_to_base is None:
            from fleet_ml.perception import transform_from_quaternion
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
            except Exception as e:  # noqa: BLE001 - not available yet; retry next frame
                self.get_logger().warn(f"waiting for TF {self.base_frame}<-{frame_id}: {e}",
                                       throttle_duration_sec=5.0)
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            self.cam_to_base = transform_from_quaternion((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        return self.cam_to_base

    def on_image(self, msg: Image) -> None:
        from fleet_ml.perception import person_point_optical, transform_point, zone_decision

        if self.k is None or self.depth is None:
            return
        if self.now_s() - self.last_detect_t < self.min_period_s:
            return
        self.last_detect_t = self.now_s()
        cam_to_base = self._camera_to_base(msg.header.frame_id)
        if cam_to_base is None:
            return
        from fleet_ml.ood import corrupt, image_stats
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.ood_inject:
            bgr = corrupt(bgr, self.ood_inject)
        ood_score = float(self.ood_model.score(image_stats(bgr))[0])
        self.ood = ood_score > self.ood_threshold
        self.ood_pub.publish(Float32(data=ood_score))
        detections = self.detector(bgr)
        located = []
        for det in detections:
            p_opt = person_point_optical(det, self.depth, self.k)
            if p_opt is not None:
                located.append((det, transform_point(cam_to_base, p_opt)))
        decision = zone_decision([pb for _, pb in located], self.stop_zone, self.warn_zone)
        self.in_stop, self.in_warn = decision.in_stop_zone, decision.in_warning_zone
        self.last_frame_t = self.now_s()
        persons = PoseArray()
        persons.header.stamp = msg.header.stamp
        persons.header.frame_id = self.base_frame
        for _, pb in located:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pb
            pose.orientation.w = 1.0
            persons.poses.append(pose)
        self.persons_pub.publish(persons)
        if self.annotated_pub.get_subscription_count() > 0:
            self._publish_annotated(msg, bgr, detections, located)

    def heartbeat(self) -> None:
        if not self.perception:
            self.scale_pub.publish(Float32(data=1.0))
            return
        t = self.now_s()
        if self.last_frame_t is None or t - self.last_frame_t > self.camera_timeout_s:
            scale, state = 0.0, "camera_timeout"
        elif self.lidar and (self.last_scan_t is None
                             or t - self.last_scan_t > self.lidar_timeout_s):
            scale, state = 0.0, "lidar_timeout"
        else:
            lidar_stop = self.lidar and self.lidar_stop
            scale = self.machine.update(t, self.in_stop or lidar_stop,
                                        slow=self.in_warn or self.ood)
            state = str(self.machine.state)
        if state != self.state:
            why = ""
            if state == "stop":
                why = " [" + "+".join(s for s, on in (("camera", self.in_stop),
                                                     ("lidar", self.lidar and self.lidar_stop))
                                      if on) + "]"
            elif state == "slow":
                why = " [" + "+".join(s for s, on in (("person", self.in_warn), ("ood", self.ood))
                                      if on) + "]"
            self.get_logger().info(f"safety state {self.state} -> {state}{why} (scale {scale})")
            self.state = state
        self.scale_pub.publish(Float32(data=float(scale)))
        self.state_pub.publish(String(data=state))

    def _publish_annotated(self, msg: Image, bgr: np.ndarray, detections, located) -> None:
        img = bgr.copy()
        dist = {id(d): pb for d, pb in located}
        for d in detections:
            pb = dist.get(id(d))
            label = f"{d.score:.2f}" + (f" {pb[0]:.1f}m" if pb else " no depth")
            cv2.rectangle(img, (d.x, d.y), (d.x + d.w, d.y + d.h), (0, 0, 255), 2)
            cv2.putText(img, label, (d.x, max(d.y - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 255), 1)
        cv2.putText(img, self.state.upper(), (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    COLORS.get(self.state, (255, 255, 255)), 2)
        out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        out.header = msg.header
        self.annotated_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

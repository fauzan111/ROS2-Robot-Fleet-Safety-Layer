"""Patch the upstream simulation assets (pure text functions, unit-tested).

- with_rgbd_camera: the upstream TurtleBot3 waffle model only has a 5 Hz depth camera on a
  non-namespaced Gazebo topic. The safety layer needs colour (person detector) plus aligned
  depth (distance to the person), per robot. We swap in an RGB-D sensor placed exactly at the
  URDF's camera_rgb_frame, publishing on <namespace>/camera/{image,depth_image,camera_info}.
- actor_sdf / with_actors: walking people (Gazebo actors) on scripted, looping trajectories,
  injected into the world. Scripted motion makes the ground truth exact: where a person is at
  any sim time follows from the waypoints, which is what stop-latency measurement needs.
"""

from __future__ import annotations

import math
import re
from xml.sax.saxutils import escape

_DEPTH_SENSOR = re.compile(
    r'<sensor name="intel_realsense_r200_depth" type="depth">.*?</sensor>', re.DOTALL)


def rgbd_sensor_xml(namespace: str, rate_hz: float = 5.0, width: int = 320,
                    height: int = 240, hfov: float = 1.047, far_m: float = 8.0) -> str:
    topic = f"{namespace}/camera" if namespace else "camera"
    # Link camera_link sits at (0.069, -0.047, 0.107) in the model frame; the URDF's
    # camera_rgb_frame is at (0.069, -0.047, 0.117) (base_link is 1 cm above the footprint).
    return f"""<sensor name="rgbd_camera" type="rgbd_camera">
          <always_on>1</always_on>
          <update_rate>{rate_hz}</update_rate>
          <pose>0 0 0.010 0 0 0</pose>
          <topic>{escape(topic)}</topic>
          <gz_frame_id>camera_rgb_optical_frame</gz_frame_id>
          <camera name="rgbd">
            <horizontal_fov>{hfov}</horizontal_fov>
            <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>{far_m}</far></clip>
          </camera>
        </sensor>"""


def with_rgbd_camera(robot_sdf: str, namespace: str, **kwargs) -> str:
    patched, n = _DEPTH_SENSOR.subn(lambda _: rgbd_sensor_xml(namespace, **kwargs), robot_sdf)
    if n != 1:
        raise ValueError(f"expected one upstream depth sensor to replace, found {n}")
    return patched


_POSE_PUBLISHER = """<plugin filename="gz-sim-pose-publisher-system"
        name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>{rate_hz}</update_frequency>
      </plugin>"""


def with_pose_publisher(robot_sdf: str, rate_hz: float = 20.0) -> str:
    """Publish the model's ground-truth world pose on /model/<name>/pose.

    Metrics (separation, stop latency) need the true pose: wheel odometry drifts as soon as a
    robot is pushed or slips, which is exactly when the numbers matter.
    """
    idx = robot_sdf.rfind("</model>")
    if idx < 0:
        raise ValueError("no </model> tag in robot SDF")
    return robot_sdf[:idx] + _POSE_PUBLISHER.format(rate_hz=rate_hz) + "\n    " + robot_sdf[idx:]


def actor_sdf(name: str, waypoints: list[dict], mesh: str) -> str:
    """A looping walking actor. waypoints: [{t, x, y}] in sim seconds and world metres.

    Heading follows the direction of travel; while standing still the last heading is kept.
    """
    if len(waypoints) < 2:
        raise ValueError(f"actor {name} needs at least 2 waypoints")
    times = [float(w["t"]) for w in waypoints]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f"actor {name}: waypoint times must increase")
    poses, yaw = [], 0.0
    for i, w in enumerate(waypoints):
        nxt = waypoints[i + 1] if i + 1 < len(waypoints) else None
        if nxt and (nxt["x"], nxt["y"]) != (w["x"], w["y"]):
            yaw = math.atan2(nxt["y"] - w["y"], nxt["x"] - w["x"])
        poses.append(f"<waypoint><time>{w['t']}</time>"
                     f"<pose>{w['x']} {w['y']} 1.0 0 0 {yaw:.4f}</pose></waypoint>")
    mesh = escape(mesh)
    return f"""<actor name="{escape(name)}">
      <skin><filename>{mesh}</filename><scale>1.0</scale></skin>
      <animation name="walking"><filename>{mesh}</filename><scale>1.0</scale>
        <interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="walking">{''.join(poses)}</trajectory>
      </script>
    </actor>"""


def with_actors(world_sdf: str, actors_xml: list[str]) -> str:
    idx = world_sdf.rfind("</world>")
    if idx < 0:
        raise ValueError("no </world> tag in world SDF")
    return world_sdf[:idx] + "\n".join(actors_xml) + "\n  " + world_sdf[idx:]


def person_position(waypoints: list[dict], t: float) -> tuple[float, float]:
    """Ground-truth position of a looping scripted actor at sim time t (linear interpolation)."""
    period = float(waypoints[-1]["t"])
    tl = t % period if period > 0 else 0.0
    for a, b in zip(waypoints, waypoints[1:]):
        if a["t"] <= tl <= b["t"]:
            f = 0.0 if b["t"] == a["t"] else (tl - a["t"]) / (b["t"] - a["t"])
            return a["x"] + f * (b["x"] - a["x"]), a["y"] + f * (b["y"] - a["y"])
    return waypoints[-1]["x"], waypoints[-1]["y"]

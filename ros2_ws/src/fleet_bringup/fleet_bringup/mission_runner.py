"""Send a list of goals to Nav2 and record what happened.

    ros2 run fleet_bringup mission_runner --ros-args -p use_sim_time:=true \
        -p missions_file:=/ws/src/fleet_bringup/config/missions_one_robot.yaml
"""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main() -> None:
    rclpy.init()
    nav = BasicNavigator()
    missions_file = nav.declare_parameter("missions_file", "").value
    results_dir = Path(nav.declare_parameter("results_dir", "/ws/results").value)
    cfg = yaml.safe_load(Path(missions_file).read_text())

    ip = cfg["initial_pose"]
    nav.setInitialPose(make_pose(nav, ip["x"], ip["y"], ip.get("yaw", 0.0)))
    nav.waitUntilNav2Active()

    timeout_s = float(cfg.get("timeout_s", 120))
    records = []
    for goal in cfg["goals"]:
        nav.get_logger().info(f"goal '{goal['name']}' -> ({goal['x']}, {goal['y']})")
        sim_t0 = nav.get_clock().now()
        wall_t0 = time.monotonic()
        nav.goToPose(make_pose(nav, goal["x"], goal["y"], goal.get("yaw", 0.0)))
        while not nav.isTaskComplete():
            if time.monotonic() - wall_t0 > timeout_s:
                nav.get_logger().warn(f"goal '{goal['name']}' timed out; cancelling")
                nav.cancelTask()
        result = nav.getResult()
        sim_s = (nav.get_clock().now() - sim_t0).nanoseconds / 1e9
        records.append({
            "goal": goal["name"],
            "result": result.name if isinstance(result, TaskResult) else str(result),
            "sim_time_s": round(sim_s, 1),
            "wall_time_s": round(time.monotonic() - wall_t0, 1),
        })
        nav.get_logger().info(f"  -> {records[-1]}")

    succeeded = sum(r["result"] == "SUCCEEDED" for r in records)
    summary = {
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "missions_file": missions_file,
        "succeeded": succeeded,
        "total": len(records),
        "goals": records,
    }
    results_dir.mkdir(parents=True, exist_ok=True)
    out = results_dir / f"missions_{datetime.now():%Y%m%d_%H%M%S}.json"
    out.write_text(json.dumps(summary, indent=2))
    nav.get_logger().info(f"{succeeded}/{len(records)} goals reached. Results: {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

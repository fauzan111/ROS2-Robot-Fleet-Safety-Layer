"""Send a list of goals to Nav2 and record what happened.

    ros2 run fleet_bringup mission_runner --ros-args -p use_sim_time:=true \
        -p missions_file:=/ws/src/fleet_bringup/config/missions_one_robot.yaml

Fleet: run one per robot in its namespace; the robot's entry is picked from the file.

    ros2 run fleet_bringup mission_runner --ros-args -r __ns:=/robot1 \
        -p use_sim_time:=true -p missions_file:=/ws/src/fleet_bringup/config/fleet.yaml
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
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from fleet_bringup.missions import mission_for


def make_pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _node_active(nav: BasicNavigator, node_name: str, timeout_s: float) -> bool:
    """Bounded version of BasicNavigator's lifecycle wait (which waits forever)."""
    client = nav.create_client(GetState, f"{node_name}/get_state")
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, future, timeout_sec=2.0)
            result = future.result()
            if result is not None and result.current_state.label == "active":
                return True
            time.sleep(0.5)
        return False
    finally:
        nav.destroy_client(client)


def wait_for_nav2(nav: BasicNavigator, initial_pose: PoseStamped, timeout_s: float = 60.0,
                  attempts: int = 3) -> BasicNavigator:
    """Wait for AMCL and the BT navigator; on timeout rebuild the node and retry.

    With several Nav2 stacks starting at once, a client's DDS discovery of a service can get
    stuck (seen in the fleet: amcl logged 'failed to send response' and the runner waited
    forever). A fresh node redoes discovery. Returns the navigator to use from here on.
    """
    for attempt in range(1, attempts + 1):
        nav.setInitialPose(initial_pose)
        if all(_node_active(nav, n, timeout_s) for n in ("amcl", "bt_navigator")):
            return nav
        nav.get_logger().warn(f"Nav2 not reachable after {timeout_s:.0f} s (attempt "
                              f"{attempt}/{attempts}); recreating the navigator node")
        nav.destroy_node()
        nav = BasicNavigator()
    raise RuntimeError(f"Nav2 not reachable after {attempts} attempts")


def main() -> None:
    rclpy.init()
    nav = BasicNavigator()
    missions_file = nav.declare_parameter("missions_file", "").value
    results_dir = Path(nav.declare_parameter("results_dir", "/ws/results").value)
    cfg = yaml.safe_load(Path(missions_file).read_text())
    robot = nav.get_namespace().strip("/")  # '' for the single-robot setup
    ip, goals = mission_for(cfg, robot)

    nav = wait_for_nav2(nav, make_pose(nav, ip["x"], ip["y"], ip.get("yaw", 0.0)))
    nav.get_logger().info("Nav2 is ready")

    def sim_now_s() -> float:
        return nav.get_clock().now().nanoseconds / 1e9

    timeout_s = float(cfg.get("timeout_s", 120))
    records = []
    start_sim_s = sim_now_s()
    for goal in goals:
        nav.get_logger().info(f"goal '{goal['name']}' -> ({goal['x']}, {goal['y']})")
        sim_t0 = nav.get_clock().now()
        wall_t0 = time.monotonic()
        nav.goToPose(make_pose(nav, goal["x"], goal["y"], goal.get("yaw", 0.0)))
        cancelled = False
        while not nav.isTaskComplete():
            if not cancelled and time.monotonic() - wall_t0 > timeout_s:
                nav.get_logger().warn(f"goal '{goal['name']}' timed out; cancelling")
                nav.cancelTask()
                cancelled = True
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
        "robot": robot,
        "succeeded": succeeded,
        "total": len(records),
        "start_sim_s": round(start_sim_s, 1),
        "end_sim_s": round(sim_now_s(), 1),
        "goals": records,
    }
    results_dir.mkdir(parents=True, exist_ok=True)
    tag = f"{robot}_" if robot else ""
    out = results_dir / f"missions_{tag}{datetime.now():%Y%m%d_%H%M%S}.json"
    out.write_text(json.dumps(summary, indent=2))
    nav.get_logger().info(f"{succeeded}/{len(records)} goals reached. Results: {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

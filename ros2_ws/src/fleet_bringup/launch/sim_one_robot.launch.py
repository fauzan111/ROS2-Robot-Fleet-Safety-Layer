"""One TurtleBot in Gazebo with Nav2, guarded by the safety layer.

Single choke point: the simulated robot only listens to /cmd_vel_safe, which only the
speed gate publishes. Every Nav2 velocity publisher (collision monitor, docking server, ...)
writes to /cmd_vel and therefore has to pass through the gate.

  Nav2 (any node) --/cmd_vel--> speed_gate --/cmd_vel_safe--> robot (Gazebo bridge)
                                    ^
                 /safety/speed_scale (safety_monitor, 20 Hz heartbeat)
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from fleet_bringup.params import write_merged_params

SPAWN = {"x": "-2.0", "y": "-0.5", "z": "0.01", "yaw": "0.0"}  # must match nav2_overrides.yaml


def _nav2(context):
    bringup = get_package_share_directory("nav2_bringup")
    ours = get_package_share_directory("fleet_bringup")
    params = write_merged_params(
        os.path.join(bringup, "params", "nav2_params.yaml"),
        os.path.join(ours, "config", "nav2_overrides.yaml"),
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": os.path.join(bringup, "maps", "tb3_sandbox.yaml"),
            "params_file": params,
            "use_sim_time": "True",
            "autostart": "True",
        }.items(),
    )]


def generate_launch_description():
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    foxglove_share = get_package_share_directory("foxglove_bridge")
    foxglove = LaunchConfiguration("foxglove")

    # --- simulator: Gazebo server + robot + bridge, with the robot listening on cmd_vel_safe
    world_sdf = tempfile.mktemp(prefix="fleet_world_", suffix=".sdf")
    world_xacro = ExecuteProcess(cmd=[
        "xacro", "-o", world_sdf, "headless:=True",
        os.path.join(sim_dir, "worlds", "tb3_sandbox.sdf.xacro")])
    gazebo = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world_sdf], output="screen")
    simulator = GroupAction(scoped=True, actions=[
        SetRemap(src="cmd_vel", dst="cmd_vel_safe"),
        world_xacro,
        RegisterEventHandler(OnProcessExit(target_action=world_xacro, on_exit=[gazebo])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_dir, "launch", "spawn_tb3.launch.py")),
            launch_arguments={"use_sim_time": "True", "x_pose": SPAWN["x"],
                              "y_pose": SPAWN["y"], "z_pose": SPAWN["z"],
                              "yaw": SPAWN["yaw"]}.items(),
        ),
    ])
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda _: os.path.exists(world_sdf) and os.remove(world_sdf))]))

    with open(os.path.join(sim_dir, "urdf", "turtlebot3_waffle.urdf")) as f:
        robot_description = f.read()
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        output="screen",
    )

    # --- safety layer
    speed_gate = Node(
        package="speed_gate", executable="speed_gate", name="speed_gate", output="screen",
        parameters=[{"use_sim_time": True, "safety_timeout_s": 0.5}],
        remappings=[("cmd_vel_in", "cmd_vel"), ("cmd_vel_out", "cmd_vel_safe")],
    )
    safety_monitor = Node(
        package="fleet_safety", executable="safety_monitor", name="safety_monitor",
        output="screen", parameters=[{"use_sim_time": True}],
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_share, "launch", "foxglove_bridge_launch.xml")),
        condition=IfCondition(foxglove),
    )

    # Gazebo must find the TurtleBot world and robot models (set before Gazebo starts).
    gz_paths = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(sim_dir)),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("foxglove", default_value="True", description="Start Foxglove bridge"),
        *gz_paths,
        simulator,
        cleanup,
        robot_state_publisher,
        OpaqueFunction(function=_nav2),
        speed_gate,
        safety_monitor,
        foxglove_bridge,
    ])

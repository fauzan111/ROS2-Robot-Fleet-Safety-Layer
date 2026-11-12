"""A fleet of TurtleBots in one Gazebo world, each with its own Nav2 stack and safety chain.

Robots, spawn poses and missions come from config/fleet.yaml. Per robot (namespace robotN):

  Nav2 (any node) --cmd_vel--> speed_gate --cmd_vel_safe--> ros_gz_bridge --> Gazebo wheels
                                   ^
              safety/speed_scale (safety_monitor, 20 Hz heartbeat)

All names are relative, so each robot's chain lives in its own namespace and fails safe on
its own: if robot2's monitor dies, only robot2 stops. The bridge config (tb3_bridge.yaml)
feeds the wheels only from cmd_vel_safe, so no Nav2 node can bypass the gate.
fleet_monitor watches all robots and records separation, near misses and distance.
"""

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

from fleet_bringup.params import write_fleet_params


def _robot(name: str, spawn: dict, shares: dict, robot_description: str) -> GroupAction:
    bringup, ours, sim_dir = shares["nav2_bringup"], shares["fleet_bringup"], shares["sim"]
    params = write_fleet_params(
        os.path.join(bringup, "params", "nav2_multirobot_params_all.yaml"),
        os.path.join(bringup, "params", "nav2_params.yaml"),
        os.path.join(ours, "config", "nav2_fleet_overrides.yaml"),
        spawn,
    )
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    return GroupAction(scoped=True, actions=[
        LogInfo(msg=f"robot {name} at ({spawn['x']}, {spawn['y']}, yaw {spawn.get('yaw', 0.0)})"),
        # --- simulator side: spawn the model and bridge its topics
        Node(
            package="ros_gz_sim", executable="create", namespace=name, output="screen",
            arguments=[
                "-name", name,
                "-string", Command([FindExecutable(name="xacro"), " namespace:=", name, " ",
                                    os.path.join(sim_dir, "urdf", "gz_waffle.sdf.xacro")]),
                "-x", str(spawn["x"]), "-y", str(spawn["y"]), "-z", "0.01",
                "-Y", str(spawn.get("yaw", 0.0)),
            ],
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", namespace=name,
            output="screen",
            parameters=[{"config_file": os.path.join(ours, "config", "tb3_bridge.yaml"),
                         "expand_gz_topic_names": True, "use_sim_time": True}],
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher", namespace=name,
            parameters=[{"use_sim_time": True, "robot_description": robot_description}],
            remappings=tf_remaps, output="screen",
        ),
        # --- navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup_launch.py")),
            launch_arguments={
                "namespace": name,
                "use_namespace": "True",
                "map": os.path.join(bringup, "maps", "tb3_sandbox.yaml"),
                "params_file": params,
                "use_sim_time": "True",
                "autostart": "True",
            }.items(),
        ),
        # --- safety chain
        Node(
            package="speed_gate", executable="speed_gate", name="speed_gate", namespace=name,
            output="screen", parameters=[{"use_sim_time": True, "safety_timeout_s": 0.5}],
            remappings=[("cmd_vel_in", "cmd_vel"), ("cmd_vel_out", "cmd_vel_safe")],
        ),
        Node(
            package="fleet_safety", executable="safety_monitor", name="safety_monitor",
            namespace=name, output="screen", parameters=[{"use_sim_time": True}],
        ),
    ])


def _fleet(context):
    fleet_file = LaunchConfiguration("fleet_file").perform(context)
    results_dir = LaunchConfiguration("results_dir").perform(context)
    cfg = yaml.safe_load(Path(fleet_file).read_text())
    shares = {
        "nav2_bringup": get_package_share_directory("nav2_bringup"),
        "fleet_bringup": get_package_share_directory("fleet_bringup"),
        "sim": get_package_share_directory("nav2_minimal_tb3_sim"),
    }
    with open(os.path.join(shares["sim"], "urdf", "turtlebot3_waffle.urdf")) as f:
        robot_description = f.read()

    actions = [_robot(name, r["spawn"], shares, robot_description)
               for name, r in cfg["robots"].items()]
    # Visualisation only: merge the robots' TF trees under 'world' for Foxglove.
    actions.append(Node(
        package="fleet_bringup", executable="fleet_tf_relay", name="fleet_tf_relay",
        output="screen", parameters=[{"use_sim_time": True, "fleet_file": fleet_file}],
    ))
    actions.append(Node(
        package="fleet_bringup", executable="fleet_monitor", name="fleet_monitor",
        output="screen",
        parameters=[{"use_sim_time": True, "fleet_file": fleet_file,
                     "results_dir": results_dir}],
    ))
    return actions


def generate_launch_description():
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    ours = get_package_share_directory("fleet_bringup")
    foxglove_share = get_package_share_directory("foxglove_bridge")

    world_sdf = tempfile.mktemp(prefix="fleet_world_", suffix=".sdf")
    world_xacro = ExecuteProcess(cmd=[
        "xacro", "-o", world_sdf, "headless:=True",
        os.path.join(sim_dir, "worlds", "tb3_sandbox.sdf.xacro")])
    gazebo = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world_sdf], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("fleet_file",
                              default_value=os.path.join(ours, "config", "fleet.yaml")),
        DeclareLaunchArgument("results_dir", default_value="/ws/results"),
        DeclareLaunchArgument("foxglove", default_value="True",
                              description="Start Foxglove bridge"),
        # Gazebo must find the TurtleBot world and robot models (set before Gazebo starts).
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(sim_dir)),
        world_xacro,
        RegisterEventHandler(OnProcessExit(target_action=world_xacro, on_exit=[gazebo])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: os.path.exists(world_sdf) and os.remove(world_sdf))])),
        # One clock for the whole world (not one per robot bridge).
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen",
        ),
        OpaqueFunction(function=_fleet),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(foxglove_share, "launch", "foxglove_bridge_launch.xml")),
            condition=IfCondition(LaunchConfiguration("foxglove")),
        ),
    ])

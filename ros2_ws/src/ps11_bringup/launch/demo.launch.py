"""Top-level demo bringup launch file (§14.4).

Brings up complete system:
  Simulation (Gazebo Harmonic, vehicle spawn, sensor bridge, robot_state_publisher)
  Vehicle stack (navigation, perception chain, telemetry scheduler)
  Acoustic link emulator
  Surface stack (surface decoder)
  Evaluation metrics
  Foxglove WebSocket bridge
  Optional MCAP bag recording
"""

import os
from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = get_package_share_directory("ps11_gazebo")
    pkg_bringup = get_package_share_directory("ps11_bringup")

    # Launch Configurations
    mode = LaunchConfiguration("mode")
    scenario = LaunchConfiguration("scenario")
    gui = LaunchConfiguration("gui")
    policy = LaunchConfiguration("policy")
    link_profile = LaunchConfiguration("link_profile")
    record = LaunchConfiguration("record")

    # Arguments
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="kinematic",
        description="Vehicle dynamics mode: kinematic (M1) or dynamic (M2)",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="demo",
        description="Environment scenario: demo or eval",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Launch Gazebo Harmonic GUI if true (false for headless)",
    )
    policy_arg = DeclareLaunchArgument(
        "policy",
        default_value="semantic",
        description="Telemetry scheduling policy (semantic or fifo_observations)",
    )
    link_profile_arg = DeclareLaunchArgument(
        "link_profile",
        default_value="m64",
        description="Acoustic link profile: m64 or generic_1k",
    )
    record_arg = DeclareLaunchArgument(
        "record",
        default_value="false",
        description="Record full MCAP bag to results/ if true",
    )

    # 1. Gazebo simulation stack (Gazebo world, vehicle spawner, bridge, robot_state_publisher)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "mode": mode,
            "scenario": scenario,
            "gui": gui,
        }.items(),
    )

    # 2. Vehicle stack (nav sensor models, waypoint_follower, perception chain, scheduler)
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "vehicle.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "policy": policy,
            "link_profile": link_profile,
            "scenario": scenario,
        }.items(),
    )

    # 3. Acoustic link emulator
    link_emulator_node = Node(
        package="ps11_telemetry",
        executable="link_emulator",
        name="link_emulator",
        parameters=[{
            "use_sim_time": True,
            "profile": link_profile,
            "mode": "pull",
        }],
        output="screen",
    )

    # 4. Surface stack (surface_decoder)
    surface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "surface.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # 5. Metrics evaluation node
    metrics_node = Node(
        package="ps11_bringup",
        executable="metrics",
        name="metrics",
        parameters=[{
            "use_sim_time": True,
            "scenario": scenario,
            "link_profile": link_profile,
        }],
        output="screen",
    )

    # 6. Foxglove WebSocket bridge for operator view
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{
            "use_sim_time": True,
            "port": 8765,
            "send_buffer_limit": 100000000,
        }],
        output="screen",
    )

    # 7. Optional MCAP bag recording
    timestamp_str = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    bag_out_dir = os.path.abspath(os.path.join("results", f"run_{timestamp_str}_bag"))
    bag_record_proc = ExecuteProcess(
        condition=IfCondition(record),
        cmd=["ros2", "bag", "record", "-s", "mcap", "-a", "-o", bag_out_dir],
        output="screen",
    )

    return LaunchDescription([
        mode_arg,
        scenario_arg,
        gui_arg,
        policy_arg,
        link_profile_arg,
        record_arg,
        sim_launch,
        vehicle_launch,
        link_emulator_node,
        surface_launch,
        metrics_node,
        foxglove_bridge_node,
        bag_record_proc,
    ])

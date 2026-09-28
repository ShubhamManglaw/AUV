"""Launch file for PS11 navigation nodes (§9, T1.3, T1.5)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory("ps11_bringup")
    nav_config_path = os.path.join(pkg_bringup, "config", "nav.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )

    odom_noise_node = Node(
        package="ps11_nav",
        executable="odom_noise",
        name="odom_noise",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    depth_sim_node = Node(
        package="ps11_nav",
        executable="depth_sim",
        name="depth_sim",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    range_adapter_node = Node(
        package="ps11_nav",
        executable="range_adapter",
        name="range_adapter",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    waypoint_follower_node = Node(
        package="ps11_nav",
        executable="waypoint_follower",
        name="waypoint_follower",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            odom_noise_node,
            depth_sim_node,
            range_adapter_node,
            waypoint_follower_node,
        ]
    )

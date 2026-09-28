"""Vehicle bringup launch file (§14.4).

Starts navigation sensor models, waypoint follower, perception chain, and scheduler.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_nav = get_package_share_directory("ps11_nav")

    use_sim_time = LaunchConfiguration("use_sim_time")
    policy = LaunchConfiguration("policy")
    link_profile = LaunchConfiguration("link_profile")

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )
    policy_arg = DeclareLaunchArgument(
        "policy",
        default_value="semantic",
        description="Scheduling policy (semantic or fifo_observations)",
    )
    link_profile_arg = DeclareLaunchArgument(
        "link_profile",
        default_value="m64",
        description="Acoustic link profile (m64 or generic_1k)",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="demo",
        description="Operating scenario",
    )

    # 1. Navigation stack (odom_noise, depth_sim, range_adapter, waypoint_follower)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, "launch", "nav.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 2. Perception chain
    detector_node = Node(
        package="ps11_perception",
        executable="detector",
        name="detector",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    tracker_node = Node(
        package="ps11_perception",
        executable="tracker",
        name="tracker",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    geolocator_node = Node(
        package="ps11_perception",
        executable="geolocator",
        name="geolocator",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    contact_db_node = Node(
        package="ps11_perception",
        executable="contact_db",
        name="contact_db",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 3. Telemetry scheduler
    scheduler_node = Node(
        package="ps11_telemetry",
        executable="scheduler",
        name="scheduler",
        parameters=[{
            "use_sim_time": use_sim_time,
            "policy": policy,
            "profile": link_profile,
        }],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        policy_arg,
        link_profile_arg,
        scenario_arg,
        nav_launch,
        detector_node,
        tracker_node,
        geolocator_node,
        contact_db_node,
        scheduler_node,
    ])

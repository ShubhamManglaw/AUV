import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "Urdf Assembly"
    urdf_file = os.path.join(
        get_package_share_directory(package_name), "urdf", "Urdf Assembly.urdf"
    )
    robot_description = Command(["xacro ", urdf_file])

    world = LaunchConfiguration("world")

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo (gz sim) world .sdf file to load, by name or full path",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments=[("gz_args", ["-r -v 3 ", world])],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "Urdf Assembly", "-topic", "robot_description"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_world_arg,
            gz_sim,
            bridge,
            robot_state_publisher_node,
            spawn_entity,
            load_joint_state_broadcaster,
            load_joint_trajectory_controller,
        ]
    )

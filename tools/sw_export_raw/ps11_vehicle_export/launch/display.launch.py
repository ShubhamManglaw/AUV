import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "Urdf Assembly"
    urdf_file = os.path.join(
        get_package_share_directory(package_name), "urdf", "Urdf Assembly.urdf"
    )
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "config", "urdf.rviz"
    )

    use_gui = LaunchConfiguration("gui")

    declare_gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch joint_state_publisher_gui to move the joints by hand",
    )

    robot_description = Command(["xacro ", urdf_file])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            declare_gui_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )

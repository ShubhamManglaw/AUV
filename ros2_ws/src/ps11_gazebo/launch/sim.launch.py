"""Simulation launch file for PS11 AUV in Gazebo Harmonic (§8.1, §8.4, §14.4, T1.2).

Launches Gazebo world, robot_state_publisher, vehicle spawner at (0, 0, -12.5),
and ros_gz_bridge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = get_package_share_directory("ps11_gazebo")
    pkg_description = get_package_share_directory("ps11_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Ensure Gazebo finds meshes from package share directories
    gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(pkg_description),
    )
    qt_plugin_path = SetEnvironmentVariable(
        name="QT_QPA_PLATFORM_PLUGIN_PATH",
        value="/usr/lib/x86_64-linux-gnu/qt5/plugins",
    )

    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="kinematic",
        description="Simulation mode: 'kinematic' (M1) or 'dynamic' (M2)",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Whether to start Gazebo GUI",
    )
    x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Spawn X coordinate"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="0.0", description="Spawn Y coordinate"
    )
    z_arg = DeclareLaunchArgument(
        "z", default_value="-12.5", description="Spawn Z coordinate"
    )
    roll_arg = DeclareLaunchArgument("R", default_value="0.0", description="Spawn Roll")
    pitch_arg = DeclareLaunchArgument(
        "P", default_value="0.0", description="Spawn Pitch"
    )
    yaw_arg = DeclareLaunchArgument("Y", default_value="0.0", description="Spawn Yaw")

    world_path = os.path.join(pkg_gazebo, "worlds", "ocean_demo_kinematic.sdf")
    bridge_config_path = os.path.join(pkg_gazebo, "config", "bridge.yaml")
    xacro_path = os.path.join(pkg_description, "urdf", "ps11.urdf.xacro")

    # Robot description from xacro
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_path,
                " mode:=",
                LaunchConfiguration("mode"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Gazebo Sim Launch
    gz_args = PythonExpression(
        [
            "'-r ' + ('-s --headless-rendering ' if '",
            LaunchConfiguration("gui"),
            "' == 'false' else '') + '",
            world_path,
            "'",
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # Spawn vehicle into Gazebo
    spawn_vehicle = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ps11",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        output="screen",
    )

    # ROS <-> Gazebo Bridge
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_config_path,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Fast image transport bridge for camera RGB and depth (§8.4, T1.2)
    image_bridge_rgb = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_rgb",
        arguments=["/vehicle/camera/image"],
        remappings=[
            ("/vehicle/camera/image", "/vehicle/camera/image_raw"),
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    image_bridge_depth = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_depth",
        arguments=["/vehicle/camera/depth_image"],
        remappings=[
            ("/vehicle/camera/depth_image", "/vehicle/camera/depth"),
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_resource_path,
            qt_plugin_path,
            mode_arg,
            gui_arg,
            x_arg,
            y_arg,
            z_arg,
            roll_arg,
            pitch_arg,
            yaw_arg,
            gz_sim,
            robot_state_publisher_node,
            spawn_vehicle,
            bridge_node,
            image_bridge_rgb,
            image_bridge_depth,
        ]
    )

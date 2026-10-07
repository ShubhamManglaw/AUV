"""Surface bringup launch file (§14.4).

Starts the surface decoder node (honesty rule H1 compliant).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )

    surface_decoder_node = Node(
        package="ps11_telemetry",
        executable="surface_decoder",
        name="surface_decoder",
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": "map",
        }],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        surface_decoder_node,
    ])

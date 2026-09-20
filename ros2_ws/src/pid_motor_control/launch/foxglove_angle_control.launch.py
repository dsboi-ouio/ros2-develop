from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_launch = PathJoinSubstitution(
        [FindPackageShare("pid_motor_control"), "launch", "angle_control.launch.py"]
    )
    foxglove_launch = PathJoinSubstitution(
        [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
    )
    foxglove_port = LaunchConfiguration("foxglove_port")
    foxglove_address = LaunchConfiguration("foxglove_address")

    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch)),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(foxglove_launch),
                launch_arguments={
                    "port": foxglove_port,
                    "address": foxglove_address,
                }.items(),
            ),
        ]
    )

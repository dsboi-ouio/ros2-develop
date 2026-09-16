from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('signal_processing'), 'config', 'signal_processing.yaml']
    )

    return LaunchDescription(
        [
            Node(
                package='signal_processing',
                executable='signal_generator',
                name='signal_generator',
                output='screen',
                parameters=[config_file],
            ),
            Node(
                package='signal_processing',
                executable='signal_filter',
                name='signal_filter',
                output='screen',
                parameters=[config_file],
            ),
        ]
    )

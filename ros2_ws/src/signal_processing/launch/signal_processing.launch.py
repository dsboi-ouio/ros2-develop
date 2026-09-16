from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('signal_processing'), 'config', 'signal_processing.yaml']
    )
    start_foxglove = LaunchConfiguration('start_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_foxglove',
                default_value='true',
                description='Start foxglove_bridge together with the demo',
            ),
            DeclareLaunchArgument(
                'foxglove_port',
                default_value='8765',
                description='WebSocket port used by foxglove_bridge',
            ),
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
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                condition=IfCondition(start_foxglove),
                parameters=[
                    {
                        'address': '0.0.0.0',
                        'port': ParameterValue(foxglove_port, value_type=int),
                    }
                ],
            ),
        ]
    )

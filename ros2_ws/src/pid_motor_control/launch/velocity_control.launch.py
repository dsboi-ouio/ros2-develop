from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_config = PathJoinSubstitution(
        [FindPackageShare("pid_motor_control"), "config", "velocity_control.yaml"]
    )
    motor_config = PathJoinSubstitution(
        [FindPackageShare("motor_simulator"), "config", "motor_simulator.yaml"]
    )
    start_reference = LaunchConfiguration("start_reference")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_reference", default_value="true"),
            Node(
                package="motor_simulator",
                executable="motor_simulator_node",
                name="motor_simulator",
                parameters=[motor_config, {"publish_frequency_hz": 1000.0}],
                output="screen",
            ),
            Node(
                package="pid_motor_control",
                executable="pid_controller_node",
                name="pid_controller",
                parameters=[controller_config],
                output="screen",
            ),
            Node(
                package="pid_motor_control",
                executable="reference_generator_node",
                name="reference_generator",
                parameters=[controller_config],
                condition=IfCondition(start_reference),
                output="screen",
            ),
        ]
    )

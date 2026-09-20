from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_config = PathJoinSubstitution(
        [FindPackageShare("pid_motor_control"), "config", "angle_control.yaml"]
    )
    motor_config = PathJoinSubstitution(
        [FindPackageShare("motor_simulator"), "config", "motor_simulator.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="motor_simulator",
                executable="motor_simulator_node",
                name="motor_simulator",
                parameters=[motor_config, {"publish_frequency_hz": 1000.0}],
                output="screen",
            ),
            Node(
                package="pid_motor_control",
                executable="angle_pid_controller_node",
                name="angle_pid_controller",
                parameters=[controller_config],
                output="screen",
            ),
        ]
    )

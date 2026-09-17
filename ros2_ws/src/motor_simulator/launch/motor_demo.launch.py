from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("motor_simulator"), "config", "motor_simulator.yaml"]
    )
    output_csv = LaunchConfiguration("output_csv")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_csv",
                default_value="motor_response.csv",
                description="CSV file written by the logger node",
            ),
            Node(
                package="motor_simulator",
                executable="motor_simulator_node",
                name="motor_simulator",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="motor_simulator",
                executable="torque_source_node",
                name="torque_source",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="motor_simulator",
                executable="motor_csv_logger",
                name="motor_csv_logger",
                parameters=[{"output_path": output_csv}],
                output="screen",
            ),
        ]
    )

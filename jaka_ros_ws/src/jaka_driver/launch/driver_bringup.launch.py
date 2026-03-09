from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    ip = LaunchConfiguration("ip")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("jaka_driver"), "config", "driver_params.yaml"]
            ),
        ),
        DeclareLaunchArgument("ip", default_value="192.168.1.100"),
        Node(
            package="jaka_driver",
            executable="jaka_driver",
            name="jaka_driver",
            output="screen",
            parameters=[params_file, {"ip": ip}],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("hexnut_6d_planner"), "config", "planner_params.yaml"]
            ),
        ),
        Node(
            package="hexnut_6d_planner",
            executable="pose_fusion_node",
            name="pose_fusion_node",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="hexnut_6d_planner",
            executable="grasp_planner_node",
            name="grasp_planner_node",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="hexnut_6d_planner",
            executable="adaptive_compliance_node",
            name="adaptive_compliance_node",
            output="screen",
            parameters=[params_file],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_driver = LaunchConfiguration("use_driver")
    use_move_group = LaunchConfiguration("use_move_group")
    use_rviz = LaunchConfiguration("use_rviz")
    use_bin_picking = LaunchConfiguration("use_bin_picking")

    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="true"),
        DeclareLaunchArgument("use_move_group", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_bin_picking", default_value="false"),
        DeclareLaunchArgument("driver_package", default_value="jaka_driver"),
        DeclareLaunchArgument("driver_executable", default_value="jaka_driver"),
        DeclareLaunchArgument("driver_namespace", default_value=""),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.100"),

        Node(
            package=LaunchConfiguration("driver_package"),
            executable=LaunchConfiguration("driver_executable"),
            namespace=LaunchConfiguration("driver_namespace"),
            name="jaka_driver",
            output="screen",
            parameters=[
                {
                    "ip": LaunchConfiguration("robot_ip"),
                }
            ],
            condition=IfCondition(use_driver),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("jaka_zu5_moveit_config"), "launch", "rsp.launch.py"]
                )
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("jaka_zu5_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ),
            condition=IfCondition(use_move_group),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("jaka_zu5_moveit_config"), "launch", "moveit_rviz.launch.py"]
                )
            ),
            condition=IfCondition(use_rviz),
        ),

        Node(
            package="bin_packing_pkg",
            executable="bin_picking_node",
            name="bin_picking_node",
            output="screen",
            condition=IfCondition(use_bin_picking),
        ),
    ])

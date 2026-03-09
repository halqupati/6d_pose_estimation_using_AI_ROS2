from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_robot", default_value="true"),
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("launch_planner", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_move_group", default_value="true"),
        DeclareLaunchArgument("use_driver", default_value="true"),
        DeclareLaunchArgument("use_bin_picking", default_value="false"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.100"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("jaka_hardware_bringup"), "launch", "robot_hw.launch.py"]
                )
            ),
            launch_arguments={
                "use_driver": LaunchConfiguration("use_driver"),
                "use_move_group": LaunchConfiguration("use_move_group"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_bin_picking": LaunchConfiguration("use_bin_picking"),
                "robot_ip": LaunchConfiguration("robot_ip"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_robot")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("jaka_hardware_bringup"),
                        "launch",
                        "camera_d435_eye_in_hand.launch.py",
                    ]
                )
            ),
            condition=IfCondition(LaunchConfiguration("launch_camera")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("hexnut_6d_planner"),
                        "launch",
                        "planner_pipeline.launch.py",
                    ]
                )
            ),
            condition=IfCondition(LaunchConfiguration("launch_planner")),
        ),
    ])

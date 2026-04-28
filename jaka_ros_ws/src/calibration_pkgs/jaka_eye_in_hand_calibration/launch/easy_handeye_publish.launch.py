from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_name", default_value="jaka_s12_eye_in_hand_d435"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("easy_handeye2"), "launch", "publish.launch.py"])
            ),
            launch_arguments={
                "name": LaunchConfiguration("calibration_name"),
            }.items(),
        ),
    ])

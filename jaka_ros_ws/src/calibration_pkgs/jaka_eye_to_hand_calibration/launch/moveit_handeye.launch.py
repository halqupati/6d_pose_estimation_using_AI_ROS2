from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_robot", default_value="true"),
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("use_driver", default_value="true"),
        DeclareLaunchArgument("use_move_group", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("joint_states_topic", default_value="/jaka_driver/joint_position"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("serial_no", default_value="''"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),

        LogInfo(
            msg=(
                "MoveIt HandEye panel settings for this launch: "
                "Sensor configuration=Eye-to-hand, Sensor frame=camera_color_optical_frame, "
                "Object frame=handeye_target, End-effector frame=Link_06, "
                "Robot base frame=base_link."
            )
        ),
        LogInfo(
            msg=(
                "Mount the target rigidly to the tool, use /camera/camera/color/image_raw and "
                "/camera/camera/color/camera_info, and save the resulting base_link -> camera_link transform."
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("jaka_hardware_bringup"), "launch", "robot_hw.launch.py"]
                )
            ),
            launch_arguments={
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_driver": LaunchConfiguration("use_driver"),
                "use_move_group": LaunchConfiguration("use_move_group"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_bin_picking": "false",
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_robot")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("jaka_hardware_bringup"), "launch", "camera_d435_eye_in_hand.launch.py"]
                )
            ),
            launch_arguments={
                "camera_name": LaunchConfiguration("camera_name"),
                "camera_namespace": LaunchConfiguration("camera_namespace"),
                "serial_no": LaunchConfiguration("serial_no"),
                "parent_frame": "base_link",
                "camera_frame": LaunchConfiguration("camera_frame"),
                "publish_mount_tf": "false",
                "enable_filter": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_camera")),
        ),
    ])

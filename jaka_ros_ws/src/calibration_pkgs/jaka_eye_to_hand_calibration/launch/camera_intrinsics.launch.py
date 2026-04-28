from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_cameracalibrator(context, *args, **kwargs):
    pattern = LaunchConfiguration("pattern").perform(context)
    max_speed = LaunchConfiguration("max_chessboard_speed").perform(context)

    command = [
        "ros2",
        "run",
        "camera_calibration",
        "cameracalibrator",
        "--size",
        LaunchConfiguration("checkerboard_size").perform(context),
        "--square",
        LaunchConfiguration("square_size_m").perform(context),
        "--queue-size",
        LaunchConfiguration("queue_size").perform(context),
        f"image:={LaunchConfiguration('image_topic').perform(context)}",
        f"camera:={LaunchConfiguration('camera_namespace_for_info').perform(context)}",
    ]

    if pattern:
        command.extend(["--pattern", pattern])
    if LaunchConfiguration("no_service_check").perform(context).lower() == "true":
        command.append("--no-service-check")
    if max_speed:
        command.extend(["--max-chessboard-speed", max_speed])

    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("serial_no", default_value="''"),
        DeclareLaunchArgument("parent_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("checkerboard_size", default_value="8x6"),
        DeclareLaunchArgument("square_size_m", default_value="0.024"),
        DeclareLaunchArgument("pattern", default_value="chessboard"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("camera_namespace_for_info", default_value="/camera/camera/color"),
        DeclareLaunchArgument("queue_size", default_value="1"),
        DeclareLaunchArgument("max_chessboard_speed", default_value="0.5"),
        DeclareLaunchArgument("no_service_check", default_value="false"),

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
                "parent_frame": LaunchConfiguration("parent_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "publish_mount_tf": "false",
                "enable_filter": "false",
            }.items(),
        ),

        OpaqueFunction(function=_launch_cameracalibrator),
    ])

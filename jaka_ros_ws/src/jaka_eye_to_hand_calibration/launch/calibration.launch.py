from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
        ),
        launch_arguments=launch_arguments.items(),
    )


def _launch_setup(context, *args, **kwargs):
    runtime = LaunchConfiguration("runtime").perform(context)
    method = LaunchConfiguration("method").perform(context)

    if runtime not in {"real", "sim"}:
        return [LogInfo(msg="Invalid runtime. Use runtime:=real or runtime:=sim.")]

    if method not in {"intrinsics", "moveit", "easy_handeye"}:
        return [LogInfo(msg="Invalid method. Use intrinsics, moveit, or easy_handeye.")]

    common_args = {
        "robot_ip": LaunchConfiguration("robot_ip"),
        "calibration_name": LaunchConfiguration("calibration_name"),
        "tracking_base_frame": LaunchConfiguration("tracking_base_frame"),
        "tracking_marker_frame": LaunchConfiguration("tracking_marker_frame"),
        "freehand_robot_movement": LaunchConfiguration("freehand_robot_movement"),
        "checkerboard_size": LaunchConfiguration("checkerboard_size"),
        "square_size_m": LaunchConfiguration("square_size_m"),
        "pattern": LaunchConfiguration("pattern"),
        "image_topic": LaunchConfiguration("image_topic"),
        "camera_namespace_for_info": LaunchConfiguration("camera_namespace_for_info"),
        "queue_size": LaunchConfiguration("queue_size"),
        "max_chessboard_speed": LaunchConfiguration("max_chessboard_speed"),
        "no_service_check": LaunchConfiguration("no_service_check"),
    }

    if runtime == "real":
        if method == "intrinsics":
            return [
                _include("jaka_eye_to_hand_calibration", "camera_intrinsics.launch.py", common_args),
            ]
        if method == "moveit":
            return [
                _include("jaka_eye_to_hand_calibration", "moveit_handeye.launch.py", common_args),
            ]
        return [
            _include("jaka_eye_to_hand_calibration", "easy_handeye_calibrate.launch.py", common_args),
        ]

    if method == "intrinsics":
        return [
            LogInfo(
                msg=(
                    "Eye-to-hand simulation intrinsics will use the fixed simulated world camera "
                    "(camera_link) on /rgbd_camera/image and /rgbd_camera/camera_info. "
                    "By default this wrapper matches the built-in Gazebo checkerboard_plane model."
                )
            ),
            _include(
                "jaka_eye_in_hand_calibration",
                "calibration.launch.py",
                {
                    "runtime": "sim",
                    "method": "intrinsics",
                    "checkerboard_size": LaunchConfiguration("checkerboard_size"),
                    "square_size_m": LaunchConfiguration("square_size_m"),
                    "pattern": LaunchConfiguration("pattern"),
                    "use_gazebo_checkerboard_defaults": LaunchConfiguration(
                        "use_gazebo_checkerboard_defaults"
                    ),
                    "sim_checkerboard_size": LaunchConfiguration("sim_checkerboard_size"),
                    "sim_square_size_m": LaunchConfiguration("sim_square_size_m"),
                    "sim_pattern": LaunchConfiguration("sim_pattern"),
                    "sim_tracking_base_frame": "camera_link",
                    "sim_image_topic": "/rgbd_camera/image",
                    "sim_camera_info_topic": "/rgbd_camera/camera_info",
                    "sim_camera_namespace_for_info": "/rgbd_camera",
                    "sim_world": LaunchConfiguration("sim_world"),
                    "queue_size": LaunchConfiguration("queue_size"),
                    "max_chessboard_speed": LaunchConfiguration("max_chessboard_speed"),
                    "no_service_check": LaunchConfiguration("no_service_check"),
                },
            ),
        ]

    if method == "moveit":
        return [
            LogInfo(
                msg=(
                    "Eye-to-hand simulation MoveIt calibration will use the fixed simulated world camera "
                    "(camera_link) on /rgbd_camera/image and /rgbd_camera/camera_info, with a built-in "
                    "simulation ChArUco target attached to Link_06."
                )
            ),
            LogInfo(
                msg=(
                    "In the HandEyeCalibration panel use: Sensor configuration=Eye-to-hand, "
                    "Sensor frame=camera_color_optical_frame, Object frame=handeye_target, "
                    "End-effector frame=Link_06, Robot base frame=base_link."
                )
            ),
            _include(
                "jaka_eye_in_hand_calibration",
                "calibration.launch.py",
                {
                    "runtime": "sim",
                    "method": "moveit",
                    "use_handeye_sim_target": "true",
                    "sim_tracking_base_frame": "camera_link",
                    "sim_image_topic": "/rgbd_camera/image",
                    "sim_camera_info_topic": "/rgbd_camera/camera_info",
                    "sim_camera_namespace_for_info": "/rgbd_camera",
                    "sim_world": LaunchConfiguration("sim_world"),
                },
            ),
        ]

    return [
        LogInfo(
            msg=(
                "Eye-to-hand simulation extrinsic calibration is not fully supported in this repo yet. "
                "The fixed world camera exists as camera_link on /rgbd_camera, but the full eye-to-hand "
                "MoveIt/easy_handeye simulation workflow is not wired in this launch yet. "
                "For eye-to-hand extrinsics, use runtime:=real or extend this launch around the fixed camera."
            )
        ),
        LogInfo(
            msg=(
                "When you wire full eye-to-hand sim support, the context should be: "
                "Sensor configuration=Eye-to-hand, Sensor frame=camera_color_optical_frame, "
                "Object frame=handeye_target, End-effector frame=Link_06, Robot base frame=base_link."
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime", default_value="real"),
        DeclareLaunchArgument("method", default_value="moveit"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("calibration_name", default_value="jaka_s12_eye_to_hand_d435"),
        DeclareLaunchArgument("tracking_base_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("tracking_marker_frame", default_value="handeye_target"),
        DeclareLaunchArgument("freehand_robot_movement", default_value="true"),
        DeclareLaunchArgument("checkerboard_size", default_value="8x8"),
        DeclareLaunchArgument("square_size_m", default_value="0.024"),
        DeclareLaunchArgument("pattern", default_value="chessboard"),
        DeclareLaunchArgument("use_gazebo_checkerboard_defaults", default_value="true"),
        DeclareLaunchArgument("sim_checkerboard_size", default_value="7x7"),
        DeclareLaunchArgument("sim_square_size_m", default_value="0.25"),
        DeclareLaunchArgument("sim_pattern", default_value="chessboard"),
        DeclareLaunchArgument("sim_world", default_value="world.sdf"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("camera_namespace_for_info", default_value="/camera/camera/color"),
        DeclareLaunchArgument("queue_size", default_value="1"),
        DeclareLaunchArgument("max_chessboard_speed", default_value="0.5"),
        DeclareLaunchArgument("no_service_check", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])

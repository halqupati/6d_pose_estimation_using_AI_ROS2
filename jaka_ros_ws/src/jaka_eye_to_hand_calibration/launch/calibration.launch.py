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
                    "Eye-to-hand simulation in this repo does not yet provide a fixed world camera model. "
                    "Launching the existing simulated D435 intrinsic calibration path so you can still test "
                    "camera_calibration on the simulated sensor."
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
                    "queue_size": LaunchConfiguration("queue_size"),
                    "max_chessboard_speed": LaunchConfiguration("max_chessboard_speed"),
                    "no_service_check": LaunchConfiguration("no_service_check"),
                },
            ),
        ]

    return [
        LogInfo(
            msg=(
                "Eye-to-hand simulation extrinsic calibration is not fully supported in this repo yet. "
                "The current Gazebo camera is wrist-mounted for eye-in-hand simulation, not fixed in the workcell. "
                "For eye-to-hand extrinsics, use runtime:=real or add a fixed world camera model first."
            )
        ),
        LogInfo(
            msg=(
                "Once a fixed simulated cell camera exists, the context should be: "
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
        DeclareLaunchArgument("checkerboard_size", default_value="8x6"),
        DeclareLaunchArgument("square_size_m", default_value="0.024"),
        DeclareLaunchArgument("pattern", default_value="chessboard"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("camera_namespace_for_info", default_value="/camera/camera/color"),
        DeclareLaunchArgument("queue_size", default_value="1"),
        DeclareLaunchArgument("max_chessboard_speed", default_value="0.5"),
        DeclareLaunchArgument("no_service_check", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])

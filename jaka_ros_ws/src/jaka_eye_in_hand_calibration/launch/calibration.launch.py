import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _build_gazebo_moveit_config():
    moveit_config_pkg = get_package_share_directory("jaka_s12_moveit_config")
    initial_positions_file = os.path.join(moveit_config_pkg, "config", "initial_positions.yaml")
    return (
        MoveItConfigsBuilder(robot_name="jaka_s12", package_name="jaka_s12_moveit_config")
        .robot_description(
            file_path="config/jaka_s12.urdf.xacro",
            mappings={
                "use_gazebo": "true",
                "use_rviz_sim": "false",
                "initial_positions_file": initial_positions_file,
            },
        )
        .robot_description_semantic(file_path="config/jaka_s12.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )


def _include(package_name, launch_file, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
        ),
        launch_arguments=launch_arguments.items(),
    )


def _build_sim_actions(context):
    method = LaunchConfiguration("method").perform(context)
    world_name = LaunchConfiguration("sim_world").perform(context)
    sim_image_topic = LaunchConfiguration("sim_image_topic").perform(context)
    sim_camera_info_topic = LaunchConfiguration("sim_camera_info_topic").perform(context)
    sim_camera_namespace = LaunchConfiguration("sim_camera_namespace_for_info").perform(context)
    sim_tracking_base = LaunchConfiguration("sim_tracking_base_frame").perform(context)
    tracking_marker_frame = LaunchConfiguration("tracking_marker_frame").perform(context)
    calibration_name = LaunchConfiguration("calibration_name").perform(context)
    freehand_robot_movement = LaunchConfiguration("freehand_robot_movement").perform(context)

    moveit_config = _build_gazebo_moveit_config()
    robot_description = moveit_config.robot_description["robot_description"]
    moveit_config_pkg = get_package_share_directory("jaka_s12_moveit_config")
    models_path = os.path.join(get_package_share_directory("jaka_description"), "worlds")
    custom_gripper_path = get_package_share_directory("custom_gripper_pkg")
    world_path = os.path.join(models_path, world_name)
    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")

    actions = [
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[
                models_path,
                ":" + os.path.join(moveit_config_pkg, "worlds"),
                ":" + str(Path(get_package_share_directory("jaka_description")).parent.resolve()),
                ":" + str(Path(custom_gripper_path).parent.resolve()),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base_link",
            output="screen",
            arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            parameters=[{"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": f"-r -v 4 {world_path}"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-string",
                robot_description,
                "-name",
                "jaka_s12",
                "-allow_renaming",
                "true",
                "-x",
                "0.0",
                "-y",
                "0.0",
                "-z",
                "1.016",
                "-R",
                "0.0",
                "-P",
                "0.0",
                "-Y",
                "0.0",
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "/rgbd_mounted_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/rgbd_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                "/rgbd_mounted_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/rgbd_mounted_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["jaka_s12_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["custom_gripper_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]

    if method in {"moveit", "easy_handeye"}:
        actions.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
            )
        )

    if method == "moveit":
        actions.extend(
            [
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config_file],
                    parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
                ),
                LogInfo(
                    msg=(
                        "Simulation MoveIt hand-eye calibration ready. "
                        "Use image topic "
                        + sim_image_topic
                        + ", camera info topic "
                        + sim_camera_info_topic
                        + ", end-effector frame Link_06, robot base frame base_link. "
                        "The simulated camera frame default is "
                        + sim_tracking_base
                        + "; verify with: ros2 topic echo "
                        + sim_camera_info_topic
                        + " --once"
                    )
                ),
            ]
        )
    elif method == "intrinsics":
        actions.extend(
            [
                LogInfo(
                    msg=(
                        "Simulation intrinsic calibration will use "
                        + sim_image_topic
                        + " and "
                        + sim_camera_info_topic
                    )
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2",
                                "run",
                                "camera_calibration",
                                "cameracalibrator",
                                "--size",
                                LaunchConfiguration("checkerboard_size").perform(context),
                                "--square",
                                LaunchConfiguration("square_size_m").perform(context),
                                "--pattern",
                                LaunchConfiguration("pattern").perform(context),
                                "--queue-size",
                                LaunchConfiguration("queue_size").perform(context),
                                "--max-chessboard-speed",
                                LaunchConfiguration("max_chessboard_speed").perform(context),
                                "--no-service-check",
                                f"image:={sim_image_topic}",
                                f"camera:={sim_camera_namespace}",
                            ],
                            output="screen",
                        )
                    ],
                ),
            ]
        )
    else:
        actions.extend(
            [
                LogInfo(
                    msg=(
                        "Simulation easy_handeye2 ready. You still need a detector that publishes "
                        + sim_tracking_base
                        + " -> "
                        + tracking_marker_frame
                        + " in TF from the simulated camera stream."
                    )
                ),
                _include(
                    "easy_handeye2",
                    "calibrate.launch.py",
                    {
                        "calibration_type": "eye_in_hand",
                        "name": calibration_name,
                        "robot_base_frame": "base_link",
                        "robot_effector_frame": "Link_06",
                        "tracking_base_frame": sim_tracking_base,
                        "tracking_marker_frame": tracking_marker_frame,
                        "freehand_robot_movement": freehand_robot_movement,
                    },
                ),
            ]
        )

    return actions


def _launch_setup(context, *args, **kwargs):
    runtime = LaunchConfiguration("runtime").perform(context)
    method = LaunchConfiguration("method").perform(context)

    if runtime not in {"real", "sim"}:
        return [LogInfo(msg="Invalid runtime. Use runtime:=real or runtime:=sim.")]

    if method not in {"intrinsics", "moveit", "easy_handeye"}:
        return [LogInfo(msg="Invalid method. Use intrinsics, moveit, or easy_handeye.")]

    if runtime == "real":
        launch_args = {
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

        if method == "intrinsics":
            return [
                _include("jaka_eye_in_hand_calibration", "camera_intrinsics.launch.py", launch_args),
            ]
        if method == "moveit":
            return [
                _include("jaka_eye_in_hand_calibration", "moveit_handeye.launch.py", launch_args),
            ]
        return [
            _include("jaka_eye_in_hand_calibration", "easy_handeye_calibrate.launch.py", launch_args),
        ]

    return _build_sim_actions(context)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime", default_value="real"),
        DeclareLaunchArgument("method", default_value="moveit"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("calibration_name", default_value="jaka_s12_eye_in_hand_d435"),
        DeclareLaunchArgument("tracking_base_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("sim_tracking_base_frame", default_value="mounted_camera_link"),
        DeclareLaunchArgument("tracking_marker_frame", default_value="handeye_target"),
        DeclareLaunchArgument("freehand_robot_movement", default_value="true"),
        DeclareLaunchArgument("checkerboard_size", default_value="8x6"),
        DeclareLaunchArgument("square_size_m", default_value="0.024"),
        DeclareLaunchArgument("pattern", default_value="chessboard"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("camera_namespace_for_info", default_value="/camera/camera/color"),
        DeclareLaunchArgument("sim_image_topic", default_value="/rgbd_mounted_camera/image"),
        DeclareLaunchArgument("sim_camera_info_topic", default_value="/rgbd_mounted_camera/camera_info"),
        DeclareLaunchArgument("sim_camera_namespace_for_info", default_value="/rgbd_mounted_camera"),
        DeclareLaunchArgument("queue_size", default_value="1"),
        DeclareLaunchArgument("max_chessboard_speed", default_value="0.5"),
        DeclareLaunchArgument("no_service_check", default_value="false"),
        DeclareLaunchArgument("sim_world", default_value="world.sdf"),
        OpaqueFunction(function=_launch_setup),
    ])

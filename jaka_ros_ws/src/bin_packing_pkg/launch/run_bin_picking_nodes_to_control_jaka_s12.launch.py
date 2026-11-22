import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
    

from launch.actions import OpaqueFunction

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


# Conservative project defaults. Used for BOTH modes unless overridden.
# robot_mode:=real (default) is what jaka_hardware_bringup/robot_hw.launch.py uses.
CONSERVATIVE_MOTION_DEFAULTS = {
    "arm_velocity_scaling": "0.30",
    "arm_acceleration_scaling": "0.30",
    "grasp_descent_step": "0.002",
    "gripper_motion_duration": "1.20",
}

# Mirror of real_robot_limits in src/bin_picking_node_v5_to_control_jaka_s12.cpp.
# The C++ node is authoritative (it also rejects live `ros2 param set`); this copy
# only aborts the launch early with a clear message. Keep both in sync.
REAL_ROBOT_LIMITS = {
    "arm_velocity_scaling": ("max", 0.30),
    "arm_acceleration_scaling": ("max", 0.30),
    "grasp_descent_step": ("max", 0.002),
    "gripper_motion_duration": ("min", 1.20),
}


# Tightening launch arguments and their defaults (node parameters of the same name).
TIGHTEN_ARGS = {
    "tighten_snug_torque": "1.0",       # N*m on the wrist F/T sensor (jaws slip at 1.3-2.8)
    "tighten_stroke_deg": "180.0",      # turn per grip: 60, 120 or 180
    "tighten_max_turns": "4.0",         # safety stop
    "tighten_velocity_scaling": "0.5",  # arm speed while turning the nut
}


def _validate_robot_mode(context, *args, **kwargs):
    robot_mode = LaunchConfiguration("robot_mode").perform(context).strip().lower()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower() in ("true", "1", "yes")

    if robot_mode not in ("real", "sim"):
        raise RuntimeError(f"robot_mode must be 'real' or 'sim', got '{robot_mode}'.")
    if robot_mode == "real" and use_sim_time:
        raise RuntimeError(
            "robot_mode:=real with use_sim_time:=true is inconsistent. "
            "Use robot_mode:=sim use_sim_time:=true for Gazebo."
        )
    if robot_mode == "sim" and not use_sim_time:
        raise RuntimeError("robot_mode:=sim requires use_sim_time:=true.")

    resolved = {name: float(LaunchConfiguration(name).perform(context)) for name in REAL_ROBOT_LIMITS}
    if robot_mode == "real":
        for name, (kind, limit) in REAL_ROBOT_LIMITS.items():
            value = resolved[name]
            if (kind == "max" and value > limit + 1e-9) or (kind == "min" and value < limit - 1e-9):
                bound = "max" if kind == "max" else "min"
                raise RuntimeError(
                    f"Refusing to launch on the physical JAKA S12: {name}={value} violates real-robot "
                    f"{bound} {limit}. Fast presets are simulation-only (robot_mode:=sim)."
                )

    print(
        f"[run_bin_picking_nodes] robot_mode={robot_mode} use_sim_time={use_sim_time} "
        + " ".join(f"{k}={v}" for k, v in resolved.items())
    )
    return []


def generate_launch_description():
    arm_robot_sim_path = get_package_share_directory('jaka_s12_moveit_config')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("jaka_description"), "config", "display2.rviz"]
    )

    arm_velocity_scaling = LaunchConfiguration("arm_velocity_scaling")
    arm_acceleration_scaling = LaunchConfiguration("arm_acceleration_scaling")
    grasp_descent_step = LaunchConfiguration("grasp_descent_step")
    gripper_motion_duration = LaunchConfiguration("gripper_motion_duration")
    pick_safe_gap_z = LaunchConfiguration("pick_safe_gap_z")
    detected_nut_height = LaunchConfiguration("detected_nut_height")
    min_grasp_tcp_above_table = LaunchConfiguration("min_grasp_tcp_above_table")
    enable_collision_monitor = LaunchConfiguration("enable_collision_monitor")
    collision_check_period = LaunchConfiguration("collision_check_period")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_mode = LaunchConfiguration("robot_mode")
    task_mode = LaunchConfiguration("task_mode")
    # Tightening tuning (task_mode:=tighten). float() typing so "120" is not passed as an int.
    tighten_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in TIGHTEN_ARGS
    }

    # -------------------------
    # MoveIt configuration
    # -------------------------
    moveit_config = (
        MoveItConfigsBuilder(robot_name="jaka_s12", package_name="jaka_s12_moveit_config")
        .robot_description(file_path="config/jaka_s12.urdf.xacro")
        .robot_description_semantic(file_path="config/jaka_s12.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path= "config/moveit_controllers.yaml")
        .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    

    move_group_node = Node(
        name="bin_picking_node_v5_to_control_jaka_s12",
        package="bin_packing_pkg",
        executable="bin_picking_node_v5_to_control_jaka_s12",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim_time,
                "robot_mode": robot_mode,
                "task_mode": task_mode,
                "bolt_pose_source": LaunchConfiguration("bolt_pose_source"),
                **tighten_params,
                "flange_link": "grasp_tcp",
                "tool_center_offset": 0.019,
                "grasp_final_offset": 0.0,
                "grasp_descent_step": grasp_descent_step,
                "pick_approach_offset": 0.020,
                "pick_safe_gap_z": pick_safe_gap_z,
                "detected_nut_height": detected_nut_height,
                "min_grasp_tcp_above_table": min_grasp_tcp_above_table,
                "max_pick_descent": 0.25,
                "pick_xy_tolerance": 0.008,
                "pick_z_tolerance": 0.010,
                "pregrasp_height": 0.20,
                "place_hover_height": 0.40,
                "lift_height": 0.10,
                "pick_clearance_z": 0.006,
                "table_top_z": -0.06,
                "arm_velocity_scaling": arm_velocity_scaling,
                "arm_acceleration_scaling": arm_acceleration_scaling,
                "arm_planning_time": 5.0,
                "gripper_open_cmd": 0.0,
                "gripper_close_cmd": -0.025,
                "empty_close_threshold": -0.023,
                "gripper_motion_duration": gripper_motion_duration,
                "gripper_settle_time": 0.2,
            },
        ],
    )

    collision_monitor_node = Node(
        name="collision_monitor_node",
        package="bin_packing_pkg",
        executable="collision_monitor_node",
        output="screen",
        condition=IfCondition(enable_collision_monitor),
        parameters=[
            moveit_config.robot_description,
            # The PlanningSceneMonitor needs the SRDF too; without it the node logged
            # "Could not find parameter robot_description_semantic" and never monitored.
            moveit_config.robot_description_semantic,
            {
                "use_sim_time": use_sim_time,
                "planning_scene_topic": "/monitored_planning_scene",
                "check_period_sec": collision_check_period,
            },
        ],
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "robot_mode",
                default_value="real",
                choices=["real", "sim"],
                description="'real' = physical JAKA S12 (conservative limits enforced). "
                "'sim' = Gazebo only (requires use_sim_time:=true and a /clock publisher).",
            ),
            DeclareLaunchArgument(
                "arm_velocity_scaling", default_value=CONSERVATIVE_MOTION_DEFAULTS["arm_velocity_scaling"]
            ),
            DeclareLaunchArgument(
                "arm_acceleration_scaling", default_value=CONSERVATIVE_MOTION_DEFAULTS["arm_acceleration_scaling"]
            ),
            DeclareLaunchArgument(
                "grasp_descent_step", default_value=CONSERVATIVE_MOTION_DEFAULTS["grasp_descent_step"]
            ),
            DeclareLaunchArgument(
                "gripper_motion_duration", default_value=CONSERVATIVE_MOTION_DEFAULTS["gripper_motion_duration"]
            ),
            DeclareLaunchArgument("pick_safe_gap_z", default_value="0.003"),
            DeclareLaunchArgument("detected_nut_height", default_value="0.008"),
            DeclareLaunchArgument("min_grasp_tcp_above_table", default_value="0.020"),
            DeclareLaunchArgument("enable_collision_monitor", default_value="true"),
            DeclareLaunchArgument("collision_check_period", default_value="0.20"),
            DeclareLaunchArgument(
                "task_mode",
                default_value="pick_place",
                choices=["pick_place", "tighten", "pick_tighten"],
                description="pick_place = bin picking; tighten = tighten a nut already on the bolt; "
                "pick_tighten = pick a nut, put it on the bolt and tighten it (both sim only).",
            ),
            *[DeclareLaunchArgument(name, default_value=default) for name, default in TIGHTEN_ARGS.items()],
            DeclareLaunchArgument(
                "bolt_pose_source",
                default_value="camera",
                choices=["camera", "parameters"],
                description="Where the tightening tasks get the bolt pose: bolt_detector (camera) or parameters.",
            ),
            OpaqueFunction(function=_validate_robot_mode),
            move_group_node,
            collision_monitor_node,
        ]
    )

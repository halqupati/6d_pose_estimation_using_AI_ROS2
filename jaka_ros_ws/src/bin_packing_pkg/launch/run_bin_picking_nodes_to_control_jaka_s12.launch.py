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
    

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

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
            DeclareLaunchArgument("arm_velocity_scaling", default_value="0.30"),
            DeclareLaunchArgument("arm_acceleration_scaling", default_value="0.30"),
            DeclareLaunchArgument("grasp_descent_step", default_value="0.002"),
            DeclareLaunchArgument("gripper_motion_duration", default_value="1.20"),
            DeclareLaunchArgument("pick_safe_gap_z", default_value="0.003"),
            DeclareLaunchArgument("detected_nut_height", default_value="0.008"),
            DeclareLaunchArgument("min_grasp_tcp_above_table", default_value="0.020"),
            DeclareLaunchArgument("enable_collision_monitor", default_value="true"),
            DeclareLaunchArgument("collision_check_period", default_value="0.20"),
            move_group_node,
            collision_monitor_node,
        ]
    )

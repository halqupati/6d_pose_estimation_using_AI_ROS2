import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
                "use_sim_time": True,
                "flange_link": "grasp_tcp",
                "tool_center_offset": 0.019,
                "grasp_final_offset": 0.0,
                "grasp_descent_step": 0.002,
                "pick_approach_offset": 0.020,
                "pick_safe_gap_z": 0.003,
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
                "gripper_motion_duration": 1.2,
                "gripper_settle_time": 0.2,
            },
        ],
    )
    


    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_velocity_scaling", default_value="0.30"),
            DeclareLaunchArgument("arm_acceleration_scaling", default_value="0.30"),
            move_group_node,
            
        ]
    )

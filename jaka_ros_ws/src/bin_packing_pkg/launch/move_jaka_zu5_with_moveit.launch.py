import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
    

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    arm_robot_sim_path = get_package_share_directory('jaka_zu5_moveit_config')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("jaka_description"), "config", "display2.rviz"]
    )

    # -------------------------
    # MoveIt configuration
    # -------------------------
    moveit_config = (
        MoveItConfigsBuilder(robot_name="jaka_zu5", package_name="jaka_zu5_moveit_config")
        .robot_description(file_path="config/jaka_zu5.urdf.xacro")
        .robot_description_semantic(file_path="config/jaka_zu5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path= "config/moveit_controllers.yaml")
        .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    

    move_group_node = Node(
        name="bin_picking_node",
        package="bin_packing_pkg",
        executable="bin_picking_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription(
        [
            move_group_node,
            rviz_node,
        ]
    )

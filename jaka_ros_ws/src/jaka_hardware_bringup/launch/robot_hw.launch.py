import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _build_s12_moveit_config():
    moveit_config_pkg = get_package_share_directory("jaka_s12_moveit_config")
    initial_positions_file = os.path.join(moveit_config_pkg, "config", "initial_positions.yaml")
    return (
        MoveItConfigsBuilder(robot_name="jaka_s12", package_name="jaka_s12_moveit_config")
        .robot_description(
            file_path="config/jaka_s12.urdf.xacro",
            mappings={
                "use_gazebo": "false",
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


def generate_launch_description():
    use_driver = LaunchConfiguration("use_driver")
    use_move_group = LaunchConfiguration("use_move_group")
    use_rviz = LaunchConfiguration("use_rviz")
    use_bin_picking = LaunchConfiguration("use_bin_picking")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("jaka_s12_moveit_config"), "config", "moveit.rviz"]
    )
    moveit_config = _build_s12_moveit_config()

    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="true"),
        DeclareLaunchArgument("use_move_group", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_bin_picking", default_value="false"),
        DeclareLaunchArgument("joint_states_topic", default_value="/jaka_driver/joint_position"),
        DeclareLaunchArgument("driver_package", default_value="jaka_driver"),
        DeclareLaunchArgument("driver_executable", default_value="jaka_driver"),
        DeclareLaunchArgument("driver_namespace", default_value=""),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.100"),

        Node(
            package=LaunchConfiguration("driver_package"),
            executable=LaunchConfiguration("driver_executable"),
            namespace=LaunchConfiguration("driver_namespace"),
            name="jaka_driver",
            output="screen",
            parameters=[
                {
                    "ip": LaunchConfiguration("robot_ip"),
                }
            ],
            condition=IfCondition(use_driver),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
            remappings=[("/joint_states", joint_states_topic)],
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
            remappings=[("/joint_states", joint_states_topic)],
            condition=IfCondition(use_move_group),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
            condition=IfCondition(use_rviz),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("bin_packing_pkg"),
                        "launch",
                        "run_bin_picking_nodes_to_control_jaka_s12.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "use_sim_time": "false",
                # Physical JAKA S12: conservative motion defaults, real-robot limits enforced
                # by both the launch file and the C++ node. Never pass robot_mode:=sim here.
                "robot_mode": "real",
            }.items(),
            condition=IfCondition(use_bin_picking),
        ),
    ])

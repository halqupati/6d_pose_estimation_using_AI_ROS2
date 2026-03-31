import os
import xacro
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch.actions import SetEnvironmentVariable

#Add
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # === Parameters/Arguments ===
    robot_description_path = get_package_share_directory('jaka_description')
    arm_robot_sim_path = get_package_share_directory('jaka_s12_moveit_config')
    moveit_config_pkg = get_package_share_directory('jaka_s12_moveit_config')
    pkg_ur_description = 'jaka_s12_moveit_config'
    urdf_path = 'config/jaka_s12.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_ur_description), urdf_path)
    robo_des_raw = xacro.process_file(xacro_file).toxml()
    ompl_yaml_path = os.path.join(get_package_share_directory("jaka_s12_moveit_config"), 'config', 'ompl_planning.yaml')

    octomap_updater_config = os.path.join(
        get_package_share_directory("jaka_s12_moveit_config"),
        "config",
        "sensors_3d.yaml",
    )

    
    models_path = os.path.join(get_package_share_directory('jaka_description'), 'worlds')

    gazebo_resource_path = SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=models_path
    )

    world_path = PathJoinSubstitution([
        FindPackageShare("jaka_description"),  # Replace with your package name
        "worlds",
        "world.sdf"
    ])

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("jaka_description"), "config", "display2.rviz"]
    )
    
    # === Nodes ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robo_des_raw},
            {'use_sim_time': True}
        ]
    )
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    print("Before moveit")
    # -------------------------
    # MoveIt configuration
    # -------------------------
    moveit_config = (
        MoveItConfigsBuilder(robot_name="jaka_s12", package_name="jaka_s12_moveit_config")
        .robot_description(file_path="config/jaka_s12.urdf.xacro")
        .robot_description_semantic(file_path="config/jaka_s12.srdf")
        .trajectory_execution(file_path= "config/moveit_controllers.yaml")
        .moveit_cpp(moveit_config_pkg + "/config/controller_setting.yaml")
        .planning_pipelines( pipelines=["ompl"])
        .sensors_3d(moveit_config_pkg + "/config/sensors_3d.yaml")
        .to_moveit_configs()
    )
    

    # NEW: Create the actual MoveGroup Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "ompl.request_adapters": (
                    "default_planner_request_adapters/FixWorkspaceBounds "
                    "default_planner_request_adapters/FixStartStateBounds "
                    "default_planner_request_adapters/FixStartStateCollision "
                    "default_planner_request_adapters/FixStartStatePathConstraints "
                    "default_planner_request_adapters/AddTimeOptimalParameterization"
                ),
                "ompl.start_state_max_bounds_error": 0.1,
            
                'use_sim_time': True},
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

    jaka_zu5_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaka_s12_controller", "-c", "/controller_manager"],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robo_des_raw,
            "-name", "jaka_s12",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "1.018",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0"
        ]
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",

            "/rgbd_mounted_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_mounted_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/rgbd_mounted_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    depth_image_converter = Node(
        package="depth_image_proc",
        executable="convert_metric_node",
        name="depth_image_converter",
        output="screen",
        remappings=[
            ("image_raw", "/rgbd_camera/depth_image"),
            ("camera_info", "/rgbd_camera/camera_info"),
            ("image", "/rgbd_camera/depth_image/image")
        ]
    )

    # Point Cloud Filter (New Custom Node)
    pcl_filter_node = Node(
        package='jaka_s12_moveit_config', # Replace with your package name
        executable='statistical_filter_node',
        name='statistical_outlier_removal',
        output='screen',
        remappings=[
            ('input', '/rgbd_camera/points'),
            ('output', '/rgbd_camera/points_filtered_external')
        ],
        parameters=[
            {'mean_k': 50},
            {'stddev_mul_thresh': 1.0},
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        gazebo_resource_path,
        robot_state_publisher_node,
        static_tf,
        run_move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        jaka_zu5_controller_spawner,
        robotiq_gripper_controller_spawner,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_sim_bridge,
        depth_image_converter,
        pcl_filter_node,
    ])

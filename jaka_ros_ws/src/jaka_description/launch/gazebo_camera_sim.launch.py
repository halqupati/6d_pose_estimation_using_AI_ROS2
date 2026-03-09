
import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable

#Add
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # === Parameters/Arguments ===
    robot_description_path = get_package_share_directory('jaka_description')
    moveit_config_pkg = get_package_share_directory('jaka_zu5_moveit_config')
    pkg_ur_description = 'jaka_zu5_moveit_config'
    urdf_path = 'config/jaka_zu5.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_ur_description), urdf_path)
    robo_des_raw = xacro.process_file(xacro_file).toxml()

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
        arguments=["--frame-id", "world", "--child-frame-id", "Link_0"],
        parameters=[{"use_sim_time": True}],
    )




    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {'use_sim_time': True}
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    jaka_zu5_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaka_zu5_controller", "-c", "/controller_manager"],
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
            "-name", "jaka_zu5",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "1.1",
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

    return LaunchDescription([
        gazebo_resource_path,
        robot_state_publisher_node,
        static_tf,
        rviz_node,
        joint_state_broadcaster_spawner,
        jaka_zu5_controller_spawner,
        robotiq_gripper_controller_spawner,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_sim_bridge,
        depth_image_converter,
    ])


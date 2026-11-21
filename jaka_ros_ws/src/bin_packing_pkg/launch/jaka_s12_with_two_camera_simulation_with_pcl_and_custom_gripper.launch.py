import os
import xacro
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
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
    custom_gripper_path = get_package_share_directory('custom_gripper_pkg')
    octomap_updater_config = os.path.join(
        get_package_share_directory("jaka_s12_moveit_config"),
        "config",
        "sensors_3d.yaml",
    )

    models_path = os.path.join(get_package_share_directory('jaka_description'), 'worlds')

    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            models_path,                                                       # world SDF models (nut, checkerboard, etc.)
            ':' + os.path.join(moveit_config_pkg, 'worlds'),                   # jaka_s12_moveit_config/worlds models
            ':' + str(Path(robot_description_path).parent.resolve()),          # → model://jaka_description/
            ':' + str(Path(custom_gripper_path).parent.resolve())              # → model://custom_gripper_pkg/
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare("jaka_description"),  # Replace with your package name
        "worlds",
        "world.sdf"
    ])

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("jaka_description"), "config", "display2.rviz"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("jaka_s12_moveit_config"), "config", "ros2_controllers.yaml"]
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
    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file,
        ],
    )
    
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
                "ompl.start_state_max_bounds_error": 0.001,
            
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
        arguments=[
            "jaka_s12_controller",
            "-c", "/controller_manager",
            "--param-file", controllers_file,
        ],
    )

    custom_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "custom_gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file,
        ],
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
        # --headless-rendering makes the *server* render the two RGB-D cameras through
        # EGL on the NVIDIA GPU instead of the VNC display's software OpenGL (llvmpipe).
        # The Gazebo GUI still opens normally. Disable with headless_rendering:=false.
        launch_arguments={"gz_args": [
            "-r -v 4 ",
            PythonExpression(["'--headless-rendering ' if '", LaunchConfiguration("headless_rendering"),
                              "'.lower() == 'true' else ''"]),
            world_path,
        ]}.items(),
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

            # Tightening task (simulation only): thread angle of the nut on the bolt fixture.
            # Ground truth for tests and the launcher; the robot itself detects a seated nut
            # from its wrist torque, like a real robot would.
            "/world/empty_world/model/bolt_fixture/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            # Wrist force-torque sensor (flange <-> gripper body). torque.z = tightening torque.
            "/wrist_ft@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
        ],
        remappings=[
            ("/world/empty_world/model/bolt_fixture/joint_state", "/bolt_fixture/joint_states"),
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
        DeclareLaunchArgument("headless_rendering", default_value="true"),
        gazebo_resource_path,
        robot_state_publisher_node,
        run_move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        jaka_zu5_controller_spawner,
        custom_gripper_controller,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_sim_bridge,
        depth_image_converter,
    ])

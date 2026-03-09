import os
import sys
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_rviz_sim = LaunchConfiguration('use_rviz_sim', default='true')

    # -------------------------
    # Paths
    # -------------------------
    try:
        robot_description_path = get_package_share_directory('jaka_description')
        arm_robot_sim_path = get_package_share_directory('jaka_zu5_moveit_config')
        ros_gz_sim_path = get_package_share_directory('ros_gz_sim')
    except Exception as e:
        print("ERROR: Could not find one of the required packages.")
        print(e)
        sys.exit(1)

    # Set Gazebo launch file
    gazebo_launch_file = os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
    if not os.path.exists(gazebo_launch_file):
        print(f"ERROR: Gazebo launch file does not exist: {gazebo_launch_file}")
        sys.exit(1)
    else:
        print(f"Gazebo launch file found: {gazebo_launch_file}")


    # -------------------------
    # Gazebo resource path
    # -------------------------
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(arm_robot_sim_path, 'worlds'), ':' + str(Path(robot_description_path).parent.resolve())]
    )

    # -------------------------
    # World file argument
    # -------------------------
    default_world = os.path.join(
        arm_robot_sim_path,
        'worlds',
        'arm_on_the_table.sdf'  # full path to SDF
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file (full path to .sdf)'
    )
    print(f"World file found: {default_world}")




    # -------------------------
    # gz_args for Gazebo
    # -------------------------
    gz_args_sub = [
        LaunchConfiguration('world'),
        TextSubstitution(text=' -v 4 -r --physics-engine gz-physics-bullet-featherstone-plugin')
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'gz_args': gz_args_sub}.items()
    )


    # -------------------------
    # Process xacro
    # -------------------------
    xacro_file = os.path.join(arm_robot_sim_path, 'config', 'jaka_zu5.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    print(f"xacro_file found: {xacro_file}")


    # -------------------------
    # Spawn robot in Gazebo
    # -------------------------
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-x', '0.05', '-y', '0.0', '-z', '1.02',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-name', 'arm_robot',
            '-allow_renaming', 'false'
        ],
    )



    # -------------------------
    # MoveIt configuration
    # -------------------------
    moveit_config = (
        MoveItConfigsBuilder(robot_name="jaka_zu5")
        .robot_description(file_path="config/jaka_zu5.urdf.xacro")
        .robot_description_semantic(file_path="config/jaka_zu5.srdf")
        .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
        .to_moveit_configs()
    )


    # -------------------------
    # RViz
    # -------------------------
    rviz_config_file = os.path.join(get_package_share_directory("jaka_zu5_moveit_config"), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic, {"use_sim_time": True}],
    )

    # -------------------------
    # Static TF
    # -------------------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "Link_0"],
        parameters=[{"use_sim_time": True}],
    )

    # -------------------------
    # Robot state publisher
    # -------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description, {"use_sim_time": True},],
    )

    # -------------------------
    # ROS-Gazebo bridge
    # -------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[    
                    'camera/color/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    'camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    
                    'rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    'rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    'rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    'rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   
             ],
        output='screen'
    )
    # -------------------------
    # Load controllers
    # -------------------------
    load_controllers = []
    for controller in ["jaka_zu5_controller", "joint_state_broadcaster"]:
        load_controllers.append(
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner", controller],
                output="log"
            )
        )

    # -------------------------
    # Launch Description
    # -------------------------
    return LaunchDescription([
        gazebo_resource_path,
        world_arg,
        gazebo,
        gz_spawn_entity,
        robot_state_publisher,
        rviz_node,
        static_tf,
        bridge,
    ] + load_controllers)




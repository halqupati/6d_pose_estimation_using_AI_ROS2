import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('custom_gripper_pkg')
    urdf_file  = os.path.join(pkg_share, 'urdf', 'gripper.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'gripper.rviz')
    controllers = os.path.join(pkg_share, 'config', 'gripper_controllers.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Enable ros2_control (false = use joint_state_publisher_gui)'
    )
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Publishes TF from joint states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )


    # ros2_control manager — loads and manages controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers,
        ],
        output='screen',
        condition=IfCondition(use_ros2_control),
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ros2_control),
    )

    # Spawn gripper action controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ros2_control),
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        use_ros2_control_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        rviz2,
    ])

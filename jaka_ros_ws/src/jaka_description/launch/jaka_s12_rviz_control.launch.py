import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments if needed (e.g., file paths)
    return LaunchDescription([
        # Declare the robot_description parameter using a URDF file
        DeclareLaunchArgument(
            'robot_description',
            default_value=PathJoinSubstitution([FindPackageShare('jaka_description'), 'urdf', 'jaka_s12.urdf']),
            description='Path to the URDF file for the robot'
        ),

        # Start the robot_state_publisher node 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            # remappings=[('/joint_states', 'robot_jog_command')], # For the missing jaka_jog_panel
        ),
        
        # Start the joint_state_publisher_gui node for manual control
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        # Start rviz with the provided configuration file
        Node(
            package='rviz2',  # ROS 2 uses rviz2 instead of rviz
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('jaka_description'), 'config', 'jaka_s12_urdf.rviz'])],
        ),
    ])

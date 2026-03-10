import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Prepare the robot description from the URDF file
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('jaka_description'), 'urdf', 'jaka_zu5.urdf'])
    ])

    return LaunchDescription([
        # Start the robot_state_publisher node 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
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
            arguments=['-d', PathJoinSubstitution([FindPackageShare('jaka_description'), 'config', 'jaka_zu5_urdf.rviz'])],
        ),
    ])

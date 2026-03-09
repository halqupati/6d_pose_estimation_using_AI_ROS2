from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_pointcloud_topic = LaunchConfiguration("input_pointcloud_topic")
    filtered_pointcloud_topic = LaunchConfiguration("filtered_pointcloud_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("serial_no", default_value="''"),
            DeclareLaunchArgument("parent_frame", default_value="Link_6"),
            DeclareLaunchArgument("camera_x", default_value="0.060"),
            DeclareLaunchArgument("camera_y", default_value="0.000"),
            DeclareLaunchArgument("camera_z", default_value="0.120"),
            DeclareLaunchArgument("camera_roll", default_value="0.000"),
            DeclareLaunchArgument("camera_pitch", default_value="1.5708"),
            DeclareLaunchArgument("camera_yaw", default_value="0.000"),
            DeclareLaunchArgument(
                "input_pointcloud_topic",
                default_value="/camera/camera/depth/color/points",
            ),
            DeclareLaunchArgument(
                "filtered_pointcloud_topic",
                default_value="/rgbd_camera/points_filtered_external",
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                    )
                ),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "camera_namespace": LaunchConfiguration("camera_namespace"),
                    "serial_no": LaunchConfiguration("serial_no"),
                    "base_frame_id": "camera_link",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_sync": "true",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "true",
                    "publish_tf": "true",
                    "output": "screen",
                }.items(),
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="d435_eye_in_hand_tf",
                arguments=[
                    "--x",
                    LaunchConfiguration("camera_x"),
                    "--y",
                    LaunchConfiguration("camera_y"),
                    "--z",
                    LaunchConfiguration("camera_z"),
                    "--roll",
                    LaunchConfiguration("camera_roll"),
                    "--pitch",
                    LaunchConfiguration("camera_pitch"),
                    "--yaw",
                    LaunchConfiguration("camera_yaw"),
                    "--frame-id",
                    LaunchConfiguration("parent_frame"),
                    "--child-frame-id",
                    "camera_link",
                ],
                output="screen",
            ),

            ComposableNodeContainer(
                name="d435_pcl_filter_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="pcl_ros",
                        plugin="pcl_ros::StatisticalOutlierRemoval",
                        name="statistical_outlier_removal",
                        remappings=[
                            ("input", input_pointcloud_topic),
                            ("output", filtered_pointcloud_topic),
                        ],
                        parameters=[
                            {"mean_k": 50},
                            {"stddev_mul_thresh": 1.0},
                        ],
                    )
                ],
                output="screen",
            ),
        ]
    )

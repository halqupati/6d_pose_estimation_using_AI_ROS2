#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GraspPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_planner_node")

        self.declare_parameter("input_pose_topic", "/hexnut/pose_6d")
        self.declare_parameter("output_target_topic", "/graspnet/target_pose")
        self.declare_parameter("approach_offset_z", 0.03)

        input_pose_topic = (
            self.get_parameter("input_pose_topic").get_parameter_value().string_value
        )
        output_target_topic = (
            self.get_parameter("output_target_topic").get_parameter_value().string_value
        )
        self.approach_offset_z = (
            self.get_parameter("approach_offset_z").get_parameter_value().double_value
        )

        self.create_subscription(PoseStamped, input_pose_topic, self.on_pose_6d, 10)
        self.target_pub = self.create_publisher(PoseStamped, output_target_topic, 10)

        self.get_logger().info(
            f"GraspPlannerNode started: in={input_pose_topic}, out={output_target_topic}, dz={self.approach_offset_z}"
        )

    def on_pose_6d(self, msg: PoseStamped) -> None:
        target = PoseStamped()
        target.header = msg.header
        target.pose = msg.pose
        target.pose.position.z += self.approach_offset_z
        self.target_pub.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

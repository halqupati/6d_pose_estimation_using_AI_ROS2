#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class PoseFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_fusion_node")

        self.declare_parameter("yolo_pose_topic", "/yolov12_seg/hexnut_pose")
        self.declare_parameter("aae_pose_topic", "/aae/hexnut_pose")
        self.declare_parameter("output_pose_topic", "/hexnut/pose_6d")
        self.declare_parameter("fusion_mode", "prefer_aae")

        yolo_topic = self.get_parameter("yolo_pose_topic").get_parameter_value().string_value
        aae_topic = self.get_parameter("aae_pose_topic").get_parameter_value().string_value
        output_topic = (
            self.get_parameter("output_pose_topic").get_parameter_value().string_value
        )
        self.fusion_mode = self.get_parameter("fusion_mode").get_parameter_value().string_value

        self.latest_yolo: Optional[PoseStamped] = None
        self.latest_aae: Optional[PoseStamped] = None

        self.create_subscription(PoseStamped, yolo_topic, self.on_yolo_pose, 10)
        self.create_subscription(PoseStamped, aae_topic, self.on_aae_pose, 10)
        self.pose_pub = self.create_publisher(PoseStamped, output_topic, 10)

        self.get_logger().info(
            f"PoseFusionNode started: yolo={yolo_topic}, aae={aae_topic}, out={output_topic}, mode={self.fusion_mode}"
        )

    def on_yolo_pose(self, msg: PoseStamped) -> None:
        self.latest_yolo = msg
        self.publish_fused_pose()

    def on_aae_pose(self, msg: PoseStamped) -> None:
        self.latest_aae = msg
        self.publish_fused_pose()

    def publish_fused_pose(self) -> None:
        fused = PoseStamped()

        if self.fusion_mode == "prefer_yolo":
            source = self.latest_yolo if self.latest_yolo is not None else self.latest_aae
        else:
            source = self.latest_aae if self.latest_aae is not None else self.latest_yolo

        if source is None:
            return

        fused.header = source.header
        fused.pose = source.pose
        self.pose_pub.publish(fused)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

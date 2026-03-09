#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.node import Node


class AdaptiveComplianceNode(Node):
    def __init__(self) -> None:
        super().__init__("adaptive_compliance_node")

        self.declare_parameter("target_pose_topic", "/graspnet/target_pose")
        self.declare_parameter("wrench_topic", "/force_torque/wrench")
        self.declare_parameter("twist_cmd_topic", "/jaka_driver/compliance_twist_cmd")
        self.declare_parameter("admittance_gain", 0.002)
        self.declare_parameter("max_linear_speed", 0.02)

        target_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        wrench_topic = self.get_parameter("wrench_topic").get_parameter_value().string_value
        twist_cmd_topic = (
            self.get_parameter("twist_cmd_topic").get_parameter_value().string_value
        )
        self.admittance_gain = (
            self.get_parameter("admittance_gain").get_parameter_value().double_value
        )
        self.max_linear_speed = (
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )

        self.latest_target_pose: Optional[PoseStamped] = None

        self.create_subscription(PoseStamped, target_topic, self.on_target_pose, 10)
        self.create_subscription(WrenchStamped, wrench_topic, self.on_wrench, 10)
        self.twist_pub = self.create_publisher(TwistStamped, twist_cmd_topic, 10)

        self.get_logger().info(
            f"AdaptiveComplianceNode started: target={target_topic}, wrench={wrench_topic}, cmd={twist_cmd_topic}"
        )

    def on_target_pose(self, msg: PoseStamped) -> None:
        self.latest_target_pose = msg

    def on_wrench(self, msg: WrenchStamped) -> None:
        if self.latest_target_pose is None:
            return

        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        twist = TwistStamped()
        twist.header = msg.header

        vx = self._clamp(-self.admittance_gain * fx)
        vy = self._clamp(-self.admittance_gain * fy)
        vz = self._clamp(-self.admittance_gain * fz)

        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        self.twist_pub.publish(twist)

    def _clamp(self, value: float) -> float:
        return max(-self.max_linear_speed, min(self.max_linear_speed, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdaptiveComplianceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

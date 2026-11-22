#!/usr/bin/env python3

import copy
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FollowJointTrajectoryProxy(Node):
    def __init__(self):
        super().__init__("follow_joint_trajectory_proxy")

        self.declare_parameter(
            "proxy_action_name", "/jaka_s12_safe_controller/follow_joint_trajectory"
        )
        self.declare_parameter(
            "target_action_name", "/jaka_s12_controller/follow_joint_trajectory"
        )
        self.declare_parameter("min_point_separation_sec", 0.001)
        self.declare_parameter("wait_for_server_timeout_sec", 10.0)

        self._proxy_action_name = (
            self.get_parameter("proxy_action_name").get_parameter_value().string_value
        )
        self._target_action_name = (
            self.get_parameter("target_action_name").get_parameter_value().string_value
        )
        self._min_point_separation_sec = (
            self.get_parameter("min_point_separation_sec").get_parameter_value().double_value
        )
        self._wait_for_server_timeout_sec = (
            self.get_parameter("wait_for_server_timeout_sec").get_parameter_value().double_value
        )
        self._min_point_separation_ns = max(
            int(self._min_point_separation_sec * 1_000_000_000.0), 1
        )

        self._callback_group = ReentrantCallbackGroup()
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            self._target_action_name,
            callback_group=self._callback_group,
        )
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            self._proxy_action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._client_goal_handles = {}
        self._lock = threading.Lock()

        self.get_logger().info(
            "Proxying %s -> %s with minimum point separation %.6f s"
            % (
                self._proxy_action_name,
                self._target_action_name,
                self._min_point_separation_sec,
            )
        )

    def destroy_node(self):
        self._server.destroy()
        super().destroy_node()

    def _goal_callback(self, goal_request):
        if not goal_request.trajectory.points:
            self.get_logger().warning("Rejecting empty trajectory goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        with self._lock:
            client_goal_handle = self._client_goal_handles.get(id(goal_handle))

        if client_goal_handle is not None:
            client_goal_handle.cancel_goal_async()

        return CancelResponse.ACCEPT

    @staticmethod
    def _duration_to_ns(duration_msg):
        return int(duration_msg.sec) * 1_000_000_000 + int(duration_msg.nanosec)

    @staticmethod
    def _ns_to_duration(duration_msg, total_ns):
        clamped_ns = max(int(total_ns), 0)
        duration_msg.sec = clamped_ns // 1_000_000_000
        duration_msg.nanosec = clamped_ns % 1_000_000_000

    def _sanitize_trajectory(self, goal_msg):
        sanitized_goal = copy.deepcopy(goal_msg)
        points = sanitized_goal.trajectory.points
        if len(points) < 2:
            return sanitized_goal, 0

        fixes = 0
        previous_ns = self._duration_to_ns(points[0].time_from_start)
        for point in points[1:]:
            current_ns = self._duration_to_ns(point.time_from_start)
            if current_ns <= previous_ns:
                current_ns = previous_ns + self._min_point_separation_ns
                self._ns_to_duration(point.time_from_start, current_ns)
                fixes += 1
            previous_ns = current_ns

        return sanitized_goal, fixes

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()

    def _execute_callback(self, goal_handle):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return FollowJointTrajectory.Result()

        if not self._client.wait_for_server(timeout_sec=self._wait_for_server_timeout_sec):
            self.get_logger().error(
                "Target action server %s is not available." % self._target_action_name
            )
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_string = (
                "Target action server %s not available." % self._target_action_name
            )
            return result

        sanitized_goal, fixes = self._sanitize_trajectory(goal_handle.request)
        if fixes > 0:
            self.get_logger().warning(
                "Adjusted %d trajectory timestamps for %s."
                % (fixes, self._proxy_action_name)
            )

        send_goal_future = self._client.send_goal_async(
            sanitized_goal,
            feedback_callback=lambda feedback: goal_handle.publish_feedback(feedback.feedback),
        )
        if not self._wait_for_future(send_goal_future, self._wait_for_server_timeout_sec):
            self.get_logger().error("Timed out while forwarding trajectory goal.")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_string = "Timed out while forwarding trajectory goal."
            return result

        client_goal_handle = send_goal_future.result()
        if client_goal_handle is None or not client_goal_handle.accepted:
            self.get_logger().error("Target controller rejected forwarded goal.")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_string = "Target controller rejected forwarded goal."
            return result

        with self._lock:
            self._client_goal_handles[id(goal_handle)] = client_goal_handle

        result_future = client_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                client_goal_handle.cancel_goal_async()
            time.sleep(0.01)

        with self._lock:
            self._client_goal_handles.pop(id(goal_handle), None)

        if not result_future.done():
            self.get_logger().error("Timed out while waiting for target controller result.")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_string = "Timed out while waiting for target controller result."
            return result

        wrapped_result = result_future.result()
        result = wrapped_result.result

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = FollowJointTrajectoryProxy()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

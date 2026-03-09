import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

class JakaMover(Node):
    def __init__(self):
        super().__init__('jaka_mover_node')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('JAKA Mover Node Started')

    def move_to_pose1(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'jaka_zu5'
        
        # We must define the joints for pose_1 manually here
        # These values match your SRDF: [0, 1.57, -1.57, 1.57, 1.57, 0]
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_values = [0.0, 1.5707, -1.5707, 1.5707, 1.5707, 0.0]

        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_msg.request.goal_constraints.append(Constraints(joint_constraints=[jc]))

        self.get_logger().info('Sending Planning Request to Pose_1...')
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    mover = JakaMover()
    mover.move_to_pose1()
    rclpy.spin(mover)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

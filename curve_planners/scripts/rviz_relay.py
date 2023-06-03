#!/usr/bin/env python

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from curve_planners_msgs.action import PlanCurve
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class GoalSubscriberNode(Node):

    def __init__(self):
        super().__init__('rviz_relay')
        self.action_client = ActionClient(self, PlanCurve, 'plan_curve')
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            1
        )
        self.start_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.start_callback,
            1
        )
        self.goal_msg = PlanCurve.Goal()
        self.goal_msg.planner = 'dubins'
        self.goal_set = False

    def goal_callback(self, pose_stamped_msg):
        self.goal_msg.goal.pose = pose_stamped_msg.pose
        self.goal_msg.goal.header = pose_stamped_msg.header
        self.goal_set = True
        self.send_goal(self.goal_msg)
    
    def start_callback(self, pose_stamped_msg):
        self.goal_msg.start.pose = pose_stamped_msg.pose.pose
        self.goal_msg.start.header = pose_stamped_msg.header
        if self.goal_set:
            self.send_goal(self.goal_msg)

    def send_goal(self, goal_msg):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available!')
            return
        self.action_client.send_goal_async(goal_msg)


def main():
    rclpy.init()
    node = GoalSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


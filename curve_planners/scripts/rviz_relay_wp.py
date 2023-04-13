#!/usr/bin/env python

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from curve_planners_msgs.action import PlanCurve
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray


class GoalSubscriberNode(Node):

    def __init__(self):
        super().__init__('rviz_relay_wp')
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
        self.start_set = False

        self.waypoint_publisher = self.create_publisher(PoseArray, 'waypoints', 1)

    def goal_callback(self, pose_stamped_msg):
        self.goal_msg.goal.pose = pose_stamped_msg.pose
        self.goal_msg.goal.header = pose_stamped_msg.header
        self.send_goal(self.goal_msg)  

        wp_msg = PoseArray()
        wp_msg.header = pose_stamped_msg.header
        wp_msg.poses.append(self.goal_msg.start.pose)
        for wp in self.goal_msg.waypoints:
            wp_msg.poses.append(wp.pose)
        self.waypoint_publisher.publish(wp_msg)

        self.start_set = False
        self.goal_msg.waypoints.clear()
    
    def start_callback(self, pose_stamped_msg):
        if not self.start_set:
            self.goal_msg.start.pose = pose_stamped_msg.pose.pose
            self.goal_msg.start.header = pose_stamped_msg.header
            self.start_set = True
        else:
            wp = PoseStamped()
            wp.pose = pose_stamped_msg.pose.pose
            wp.header = pose_stamped_msg.header
            self.goal_msg.waypoints.append(wp)

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


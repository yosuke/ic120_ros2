#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from ic120_msgs.srv import DumpNav
import math

world2map_trans=[0,0,0]
world2map_rot=[0,0,0,0]

class Ic120Navigation(Node):

    def __init__(self):
        super().__init__('ic120_navigation')
        self.get_logger().info("Start nav")
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_srv_client = self.create_client(DumpNav, 'nav_srv')
        # world2map_trans,world2map_rot = self.tf_buffer.lookup_transform(target_frame="world", source_frame="map", time=Time(seconds=0))
        self.wait_for_service_ready()

    def wait_for_service_ready(self):
        while not self.nav_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('nav_srv not available, waiting again...')
        self.get_logger().info("Service is now available.")

    def goal_pose(self, pose: PoseStamped):
        print(pose)
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id="map"
        goal_pose.pose.pose.position.x = pose.pose.position.x-world2map_trans[0]
        goal_pose.pose.pose.position.y = pose.pose.position.y-world2map_trans[1]
        goal_pose.pose.pose.position.z = pose.pose.position.z-world2map_trans[2]
        goal_pose.pose.pose.orientation = pose.pose.orientation
        return goal_pose

    def send_goal(self, goal):
        self.client.wait_for_server()
        return self.client.send_goal_async(goal)

    def main_logic(self):
        request = DumpNav.Request()
        future = self.nav_srv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().is_ok:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = future.result().target_pose.position.x
            pose.pose.position.y = future.result().target_pose.position.y
            pose.pose.orientation = future.result().target_pose.orientation

            goal = self.goal_pose(pose)
            send_goal_future = self.send_goal(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            if send_goal_future.result():
                self.get_logger().info("Goal sent successfully.")
            else:
                self.get_logger().info("Failed to send goal.")
        else:
            self.get_logger().info("Service call failed.")

def main(args=None):
    rclpy.init(args=args)
    ic120_navigation = Ic120Navigation()
    ic120_navigation.main_logic()
    ic120_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tf_transformations
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, PoseArray, Pose
from ic120_msgs.srv import DumpNav
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile


# waypointはworld座標系で記載する
# X,Y,Theta, orientation_flag, dumpup_flag
# DX実験フィールドでは、南北がY、東西がX
waypoints = [
    # [21421.70,14020,-math.pi/2,True,False],#1
    # [21421.70,14030,-math.pi/2,True,False],#2
    # [21421.70,14040,-math.pi/2,True,False],#3
    # [21421.70,14030,-math.pi/2,True,False],#4
    # [21428.70,14030,0,True,True],#5
    # #2に戻る

    # [21421.70,14020,-math.pi/2,True,False],#1
    # [21421.70,14030,-math.pi/2,True,False],#2
    # [21428.70,14025,0,True,True],#3
    # [21414.70,14025,0,True,False],#3

    # [21426,14020,-math.pi/2,True,True],#1
    # [21426,14030,-math.pi/2,True,False],#2
    # [21421.70,14038,-math.pi/2,True,True],#3
    # [21421.70,14040,-math.pi/2,True,False],#4
    # [21421.70,14035,-math.pi/2,True,False],#5 #3reverse
    # [21426,14030,-math.pi/2,True,False],#6 #2reverse
    # [21426,14020,-math.pi/2,True,False],#7 #1reverse

    ### 本番用
    # [21426,14020,-math.pi/2,True,False],#1
    # [21426,14030,-math.pi/2,True,False],#2
    # [21421.70,14038,-math.pi/2,True,False],#3
    # [21421.70,14040,-math.pi/2,True,False],#4 #loading point
    # [21421.70,14035,-math.pi/2,True,False],#5 #3reverse
    # [21426,14030,-math.pi/2,True,False],#6 #2reverse
    # [21430,14020,0,True,False],#7
    # [21426,14020,0,True,False],#8
    # [21421.70,14020,0,True,True],#9 #release point
    # [21430,14020,0,True,False],#10 #7revserse #2に戻る

    ### 経路テスト
    # [21431.7,14020,-61.7*math.pi/180,True,False],#1
    # [21426,14030,-61.7*math.pi/180,True,False],#2
    # [21421.7,14038,-61.7*math.pi/180,True,False],#3
    # [21426,14030,-61.7*math.pi/180,True,False],#4 #2
    # [21431.7,14020,0,True,False],#5 #1
    # [21426,14021,0,True,False],#6

    ### 本番用２
    [21434.66,14017.39,-63.5*math.pi/180,True,False],#1
    [21427.81,14030,-63.5*math.pi/180,True,False],#2
    [21421.21,14043.35,-63.5*math.pi/180,True,False],#3
    [21427.81,14030,-63.5*math.pi/180,True,False],#4=#2
    [21434.66,14017.39,-63.5*math.pi/180,True,False],#5=1
    [21422,14023,0,True,False],#6     [21422,14022,0,True,True],#6bk
]

waypoint_num=0
world2map_trans=[0,0,0]
world2map_rot=[0,0,0,0]


class FakeConstManager(Node):

    def __init__(self):
        super().__init__('fake_const_manager')
        self.get_logger().info("Start navigation service")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoint_posearray_pub = self.create_publisher(PoseArray, "/ic120/waypoints", QoSProfile(depth=10))

        self.srv = self.create_service(DumpNav, "nav_srv", self.server_callback)

        # Wait for the transformation to be available
        self.get_logger().info("Waiting for 'world' to 'map' transform")
        self.tf_buffer.can_transform('world', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=4.0))

    def euler_to_quaternion(self, euler):
        q = tf_transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def waypoint_posearray_publisher(self):
        waypoints_array = PoseArray()
        waypoints_array.header.frame_id = "world"
        waypoints_array.header.stamp = self.get_clock().now().to_msg()

        for waypoint in waypoints:
            waypoint_arrow = Pose()
            waypoint_arrow.position.x = waypoint[0]
            waypoint_arrow.position.y = waypoint[1]
            waypoint_arrow.position.z = 0  # Assuming flat ground, adjust if necessary
            waypoint_arrow.orientation = self.euler_to_quaternion(Vector3(0, 0, waypoint[2]))
            waypoints_array.poses.append(waypoint_arrow)

        self.waypoint_posearray_pub.publish(waypoints_array)

    def server_callback(self, request, response):
        self.get_logger().info("Received service call for navigation")
        global waypoint_num

        waypoint = waypoints[waypoint_num]
        waypoint_num += 1
        if waypoint_num >= len(waypoints):
            waypoint_num = 0

        self.get_logger().info(f"Waypoint: {waypoint_num}, Position: {waypoint[0]}, {waypoint[1]}, {waypoint[2]}")

        is_ok_msg = Bool()
        is_ok_msg.data = True 

        response.is_ok = is_ok_msg
        response.target_pose.header.frame_id = "world"
        response.target_pose.pose.position.x = waypoint[0]
        response.target_pose.pose.position.y = waypoint[1]
        response.target_pose.pose.position.z = 0.0  # Assuming flat ground, adjust if necessary
        response.target_pose.pose.orientation = self.euler_to_quaternion(Vector3(0, 0, waypoint[2]))
        response.orientation_flag = waypoint[3]
        response.dump_flag = waypoint[4]

        return response

def main(args=None):
    rclpy.init(args=args)
    fake_const_manager = FakeConstManager()
    rclpy.spin(fake_const_manager)
    fake_const_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

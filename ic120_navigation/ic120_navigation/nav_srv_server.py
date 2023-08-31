#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import tf2_ros
import math
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ic120_msgs.srv import DumpNav
from geometry_msgs.msg import PoseArray,Pose
from rclpy.time import Time
from rclpy.time import Duration

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
        now = self.get_clock().now()
        self.waypoint_posearray_pub = self.create_publisher(PoseArray, "/ic120/waypoints", 10)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer, self)
        #listener.waitForTransform("world", "map", rclpy.Time(0), rclpy.Duration(4.0))
        transfrom_flg=self.tfBuffer.can_transform(target_frame="world",  source_frame="map", time=Time(seconds=0),timeout=Duration(seconds=4.0))
        if(transfrom_flg == True):
            world2map_trans,world2map_rot = self.tfBuffer.lookup_transform(target_frame="world", source_frame="map", time=Time(0))
            print("World2Map Trans:", world2map_trans[0], world2map_trans[1],  world2map_trans[2])
            self.waypoint_PoseArray_publisher()
            self.s = self.create_service(DumpNav, "/ic120/nav_srv", self.server)
            print("Ready to navigation service client")

    def euler_to_quaternion(self,euler):
        quaternion = Quaternion()
        quaternion.set_euler(euler.x, euler.y, euler.z)
        return quaternion
    
    def waypoint_PoseArray_publisher(self):
        waypoints_array = PoseArray()
        waypoints_array.header.frame_id="world"
        waypoints_array.header.stamp = self.get_clock().now()
        for waypoint in waypoints:
            waypoint_arrow = Pose()
            waypoint_arrow.position.x=waypoint[0]
            waypoint_arrow.position.y=waypoint[1]
            waypoint_arrow.position.z=world2map_trans[2]
            waypoint_arrow.orientation=self.euler_to_quaternion(Vector3(0,0,waypoint[2]))    
            waypoints_array.poses.append(waypoint_arrow)
        # print(waypoints_array)
        rclpy.sleep(1)
        self.waypoint_posearray_pub.publish(waypoints_array)

    def server(self,req):
        print("Get service call for navigation")
        print("World2Map Trans:", world2map_trans[0], world2map_trans[1])
        global waypoint_num

        waypoint = waypoints[waypoint_num]
        waypoint_num+=1
        if(waypoint_num>=len(waypoints)):
            waypoint_num=0

        print("waypoint:", waypoint_num)
        print("waypoint Pos:", waypoint[0],waypoint[1],waypoint[2])

        response = DumpNav.Response()
        response.is_ok.data = True
        response.target_pose.header.frame_id="world"
        response.target_pose.pose.position.x=waypoint[0] #-world2map_trans[0]
        response.target_pose.pose.position.y=waypoint[1] #-world2map_trans[1]
        response.target_pose.pose.position.z=world2map_trans[2]
        response.target_pose.pose.orientation=self.euler_to_quaternion(Vector3(0,0,waypoint[2]))    
        response.orientation_flag.data=waypoint[3]
        response.dump_flag.data=waypoint[4]
        return response


def main(args=None):
    rclpy.init(args=args)
    fake_const_manager = FakeConstManager()
    rclpy.spin(fake_const_manager)
    fake_const_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
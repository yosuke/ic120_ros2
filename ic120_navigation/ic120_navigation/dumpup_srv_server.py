#!/usr/bin/env python
# -*- coding: utf-8 -*-

from nav_msgs import msg
import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import Odometry
import math
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ic120_msgs.srv import DumpNav
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


dumping_time = 20*10

vessel_angle=0 

class DumpUpSrv(Node):
    
    def __init__(self):
        super().__init__('dumpup_srv')
        self.dumpup_pub = self.create_publisher(Bool, '/ic120/cmd_dump', 1)
        self.dump_direction_pub = self.create_publisher(Bool, '/ic120/cmd_dump_direction', 1)
        self.left_track_pid_pause_pub = self.create_publisher(Bool, '/ic120/left_track/pid_enable', 5)
        self.right_track_pid_pause_pub = self.create_publisher(Bool, '/ic120/right_track/pid_enable', 5)
        self.create_subscription(JointState, "/ic120/joint_states", self.js_callback, 10)
        self.s = self.create_service(DumpNav, "dumpup_srv", self.server)
        self.rate_5=self.create_rate(0.5)
        self.rate_2=self.create_rate(0.2)
        print("Ready to Dump up service client.")

    def js_callback(self, data):
        if data.name[0] == "vessel_pin_joint":
            global vessel_angle
            vessel_angle = data.position[0]
            # print("vessel_angle:", vessel_angle*180/3.1415,"deg")

    def server(self, req):    
        print("Get service call for dumpup")
        pid_enable = Bool()
        pid_enable.data = False

        ### dumpup
        is_dump_direction = Bool()
        is_dump_direction.data=True
        is_dump = Bool()
        is_dump.data=True

        while vessel_angle >= -65.0/180*math.pi:
            self.dump_direction_pub.publish(is_dump)
            self.dumpup_pub.publish(is_dump_direction)
            self.left_track_pid_pause_pub.publish(pid_enable)
            self.right_track_pid_pause_pub.publish(pid_enable)
            self.rate_5.sleep()

        self.rate_2.sleep()
        ### dump down
        is_dump.data=False

        while vessel_angle <= 0.0:
            self.dump_direction_pub.publish(is_dump)
            self.dumpup_pub.publish(is_dump_direction)
            self.left_track_pid_pause_pub.publish(pid_enable)
            self.right_track_pid_pause_pub.publish(pid_enable)
            self.rate_5.sleep()
        self.rate_2.sleep()
        response = DumpNav.Response()
        response.is_ok.data = True
        print("Done")
        print("Waiting for next service call")
        pid_enable.data = True
        self.left_track_pid_pause_pub.publish(pid_enable)
        self.right_track_pid_pause_pub.publish(pid_enable)
        return response

def main(args=None):
    rclpy.init(args=args)
    dumpup_srv=DumpUpSrv()
    rclpy.spin(dumpup_srv)
    dumpup_srv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

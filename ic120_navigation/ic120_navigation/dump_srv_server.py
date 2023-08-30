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
from ic120_navigation.srv import DumpNav
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

dumpup_pub = rclpy.Publisher('/ic120/cmd_dump', Bool, queue_size=1)
dump_direction_pub = rclpy.Publisher('/ic120/cmd_dump_direction', Bool, queue_size=1)
left_track_pid_pause_pub = rclpy.Publisher('/ic120/left_track/pid_enable', Bool, queue_size=5)
right_track_pid_pause_pub = rclpy.Publisher('/ic120/right_track/pid_enable', Bool, queue_size=5)

dumping_time = 20*10

vessel_angle=0 

class dumpup_srv(Node):
    
    def __init__(self):
        super.__init__('dumpup_srv')
        rclpy.Subscriber("/ic120/joint_states", JointState, self.js_callback,queue_size=10)
        s = rclpy.Service("dumpup_srv", DumpNav, self.server)
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
            dump_direction_pub.publish(is_dump)
            dumpup_pub.publish(is_dump_direction)
            left_track_pid_pause_pub.publish(pid_enable)
            right_track_pid_pause_pub.publish(pid_enable)
            rclpy.sleep(0.5)

        rclpy.sleep(2.0)
        ### dump down
        is_dump.data=False

        while vessel_angle <= 0.0:
            dump_direction_pub.publish(is_dump)
            dumpup_pub.publish(is_dump_direction)
            left_track_pid_pause_pub.publish(pid_enable)
            right_track_pid_pause_pub.publish(pid_enable)
            rclpy.sleep(0.5)
        rclpy.sleep(2.0)
        response = DumpNav.Response()
        response.is_ok.data = True
        print("Done")
        print("Waiting for next service call")
        pid_enable.data = True
        left_track_pid_pause_pub.publish(pid_enable)
        right_track_pid_pause_pub.publish(pid_enable)
        return response

def main(args=None):
    rclpy.init(args=args)
    dumpup_srv=dumpup_srv()
    rclpy.spin(dumpup_srv)
    dumpup_srv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from nav_msgs import msg
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time import Duration
import tf2_ros
from nav_msgs.msg import Odometry
import math
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion, Vector3, Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from ic120_msgs.srv import DumpNav

world2map_trans=[0,0,0]
world2map_rot=[0,0,0,0]

class Ic120Navigation(Node):

    def __init__(self):
        super().__init__('ic120_navigation')
        print("Start nav")
        self.client = ActionClient(self, NavigateToPose, 'NavigateToPose') 

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        transfrom_flg=self.tfBuffer.can_transform(target_frame="world",  source_frame="map", time=Time(seconds=0),timeout=Duration(seconds=4.0))
        
        if(transfrom_flg == True):
            self.get_logger().info("###############################################")

            world2map_trans,world2map_rot = self.tfBuffer.lookup_transform(target_frame="world", source_frame="map", time=Time(seconds=0))

            print("wait actionclient")

            self.client.wait_for_server()

            print("wait service")

            rclpy.wait_for_service('nav_srv')

            print("Start loop")

            while True:
                nav_srv_proxy  = rclpy.ServiceProxy('nav_srv', DumpNav)
                request = DumpNav.request()
                response = nav_srv_proxy (request)
                if(response.is_ok.data == False):
                    rclpy.sleep(1)
                    continue
                goal = self.goal_pose(response.target_pose)
                self.client.send_goal(goal)
                while True:
                    now = Time.now()
                    self.tfBuffer.can_transform(target_frame="map", source_frame="/ic120_tf/base_link", time=now, timeout=Duration(4.0))
                    position, quaternion = self.tfBuffer.lookup_transform("map", "/ic120_tf/base_link", now)

                    if(response.orientation_flag.data == True):
                        print("orientation_flat==True")
                        if(self.client.wait_for_result(rclpy.Duration(0.5)) == True):
                            if(response.dump_flag.data == True):
                                rclpy.sleep(1.0)
                                print("waiting for dumpup_manager")
                                rclpy.wait_for_service('dumpup_srv') 
                                dumpup_srv_proxy  = rclpy.ServiceProxy('dumpup_srv', DumpNav)
                                request = DumpNav.request()
                                response = dumpup_srv_proxy (request)
                                if(response.is_ok.data == True):
                                    print("finished")
                                    break
                            print("next waypoint")
                            break
                        print("moving!")
                    else:
                        print("orientation_flag==False")
                        # ウェイポイントのゴールの周囲１ｍ以内にロボットが来たら、次のウェイポイントを発行する
                        if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                            print("next waitpoint")
                            break
                        else:
                            print("moving!!!")
                            rclpy.sleep(0.5)
                # rospy.spin()

    def goal_pose(self,pose):
        print(pose)
        goal_pose = NavigateToPose.Goal()
        goal_pose.target_pose.header.frame_id="map"
        goal_pose.target_pose.pose.position.x = pose.pose.position.x-world2map_trans[0]
        goal_pose.target_pose.pose.position.y = pose.pose.position.y-world2map_trans[1]
        goal_pose.target_pose.pose.position.z = pose.pose.position.z-world2map_trans[2]
        goal_pose.target_pose.pose.orientation = pose.pose.orientation
        return goal_pose


def main(args=None):
    rclpy.init(args=args)
    ic120_navigation = Ic120Navigation()
    rclpy.spin(ic120_navigation)
    ic120_navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPoseConverter(Node):
    def __init__(self):
        super().__init__('convert_goal_pose')
        self.subscription = self.create_subscription(PoseStamped,'ic120/move_base_simple/goal',self.goal_pose_callback,10)
        self.publisher = self.create_publisher(PoseStamped,'ic120/goal_pose',10)

    def goal_pose_callback(self, msg):
        msg.header.frame_id = 'map'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
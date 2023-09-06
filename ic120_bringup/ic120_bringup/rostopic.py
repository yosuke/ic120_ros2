import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class RosTopicPublisher(Node):

    def __init__(self):
        super().__init__('cmd_spin_publisher')

        self.declare_parameter('topic_name', '/ic120/cmd_spin')
        self.declare_parameter('time', 1)
        self.declare_parameter('value', True)
        
        self.topic_name=self.get_parameter('topic_name').get_parameter_value().string_value
        self.time=self.get_parameter('time').get_parameter_value().integer_value
        self.value=self.get_parameter('value').get_parameter_value().bool_value

        self.publisher_ = self.create_publisher(Bool, self.topic_name, self.time)
        msg = Bool()
        msg.data = self.value
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    rostopic_publisher = RosTopicPublisher()
    rclpy.spin(rostopic_publisher)
    rostopic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
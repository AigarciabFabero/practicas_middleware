import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.publish_message)
        self.i = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hola ROS 2 from Python: {self.i}'
        self.i += 1
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
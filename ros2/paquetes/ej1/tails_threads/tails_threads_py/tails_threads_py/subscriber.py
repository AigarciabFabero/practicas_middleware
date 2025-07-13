import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.callback,
            100  # Ajusta la profundidad de la cola aquí
        )

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        time.sleep(1)  # Simula un procesamiento lento

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class MultiCallbackNode(Node):
    def __init__(self):
        super().__init__('multi_callback_node')
        
        # Publicador a `topic1`
        self.publisher1 = self.create_publisher(String, 'topic1', 10)
        self.timer1 = self.create_timer(1.0, self.publish_message1)

        # Publicador a `topic2`
        self.publisher2 = self.create_publisher(String, 'topic2', 10)
        self.timer2 = self.create_timer(2, self.publish_message2)

        # Suscriptor a `topic1`
        self.subscription1 = self.create_subscription(
            String,
            'topic1',
            self.listener_callback1,
            10
        )

        # Suscriptor a `topic2`
        self.subscription2 = self.create_subscription(
            String,
            'topic2',
            self.listener_callback2,
            10
        )

        self.get_logger().info('MultiCallbackNode is up and running.')

    def publish_message1(self):
        thread_id = threading.get_ident()
        msg = String()
        msg.data = 'Message from topic1'
        self.publisher1.publish(msg)
        self.get_logger().info(f'Published to topic1: "{msg.data}" on thread [{thread_id}]')

    def publish_message2(self):
        thread_id = threading.get_ident()
        msg = String()
        msg.data = 'Message from topic2'
        self.publisher2.publish(msg)
        self.get_logger().info(f'Published to topic2: "{msg.data}" on thread [{thread_id}]')

    def listener_callback1(self, msg):
        thread_id = threading.get_ident()
        self.get_logger().info(f'Received from topic1: "{msg.data}" on thread [{thread_id}]')

    def listener_callback2(self, msg):
        thread_id = threading.get_ident()
        self.get_logger().info(f'Received from topic2: "{msg.data}" on thread [{thread_id}]')

def main(args=None):
    rclpy.init(args=args)
    node = MultiCallbackNode()
    executor = MultiThreadedExecutor(num_threads=4)  # Número de hilos especificado
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
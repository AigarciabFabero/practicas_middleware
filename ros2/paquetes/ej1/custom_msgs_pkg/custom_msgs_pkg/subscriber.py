import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class CustomSubscriber(Node):

    def __init__(self):
        super().__init__('custom_subscriber')
        self.string_subscription = self.create_subscription(
            String,
            '/custom_string_topic',
            self.string_callback,
            10)
        self.int_subscription = self.create_subscription(
            Int32,
            '/custom_int_topic',
            self.int_callback,
            10)

    def string_callback(self, msg):
        self.get_logger().info(f"Recibido: texto='{msg.data}'")

    def int_callback(self, msg):
        self.get_logger().info(f"Recibido: número={msg.data}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = CustomSubscriber()
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
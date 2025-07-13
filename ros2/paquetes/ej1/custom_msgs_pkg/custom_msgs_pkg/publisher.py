import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class CustomPublisher(Node):

    def __init__(self):
        super().__init__('custom_publisher')
        self.string_publisher = self.create_publisher(String, '/custom_string_topic', 10)
        self.int_publisher = self.create_publisher(Int32, '/custom_int_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        string_msg = String()
        int_msg = Int32()

        string_msg.data = f"Mensaje número {self.i}"
        int_msg.data = self.i

        self.string_publisher.publish(string_msg)
        self.int_publisher.publish(int_msg)

        self.get_logger().info(f"Publicando: texto='{string_msg.data}', número={int_msg.data}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = CustomPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
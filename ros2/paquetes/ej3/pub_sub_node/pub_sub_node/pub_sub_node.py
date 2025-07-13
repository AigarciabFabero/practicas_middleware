import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pub_sub_node')

        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.publisher = self.create_publisher(String, 'output_topic', 10)

        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.sub_callback,
            10,
            callback_group=self.sub_cb_group
        )

        self.timer = self.create_timer(
            1.0,  
            self.timer_callback,
            callback_group=self.timer_cb_group
        )

    def sub_callback(self, msg):
        self.get_logger().info(f'Recibido: "{msg.data}"')
        rclpy.spin_once(self, timeout_sec=2)  
        
        respuesta = String()
        respuesta.data = f'Procesado: {msg.data}'
        self.publisher.publish(respuesta)
        self.get_logger().info('Mensaje procesado publicado')

    def timer_callback(self):
        msg = String()
        msg.data = 'Mensaje periódico'
        self.publisher.publish(msg)
        self.get_logger().info('Mensaje periódico publicado')

def main(args=None):
    rclpy.init(args=args)
    nodo = PubSubNode()

    # Usar MultiThreadedExecutor para manejar múltiples grupos de callbacks
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(nodo)

    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
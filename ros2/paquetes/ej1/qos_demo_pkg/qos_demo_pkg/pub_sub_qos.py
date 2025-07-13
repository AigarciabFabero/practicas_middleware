import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class PubSubNode(Node):
    def __init__(self):
        super().__init__('pub_sub_node')

        # Configuración de QoS para el publicador (Durable, Reliable)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.publisher_ = self.create_publisher(String, 'durable_topic', pub_qos)

        # Configuración de QoS para el suscriptor (Volátil, Best Effort)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.subscription = self.create_subscription(
            String,
            'volatile_topic',
            self.listener_callback,
            sub_qos
        )

        # Temporizador para publicar periódicamente
        self.timer = self.create_timer(2.0, self.publish_message)

        self.get_logger().info('PubSubNode está funcionando.')

    def publish_message(self):
        """Publica un mensaje en el tema durable_topic."""
        msg = String()
        msg.data = 'Mensaje desde durable_topic'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado en durable_topic: "{msg.data}"')

    def listener_callback(self, msg):
        """Procesa los mensajes recibidos en volatile_topic."""
        self.get_logger().info(f'Recibido en volatile_topic: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    # Crear nodo único con Publicador y Suscriptor
    node = PubSubNode()

    try:
        # Mantener el nodo en ejecución
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Limpiar recursos
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

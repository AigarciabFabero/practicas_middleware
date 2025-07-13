import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class DoubleTalker(Node):
    """Nodo que contiene dos publicadores y un suscriptor con diferentes grupos de callbacks."""

    def __init__(self):
        super().__init__('double_talker')

        # Contador para los publicadores
        self.i = 0
        self.pub1 = self.create_publisher(String, 'chatter1', 10)
        self.pub2 = self.create_publisher(String, 'chatter2', 10)

        # Grupos de callbacks
        self.mutually_exclusive_group = MutuallyExclusiveCallbackGroup()
        self.reentrant_group = ReentrantCallbackGroup()

        # Temporizadores para los publicadores (asociados al grupo mutuamente excluyente)
        self.timer1 = self.create_timer(1.0, self.timer_callback1, callback_group=self.mutually_exclusive_group)
        self.timer2 = self.create_timer(0.5, self.timer_callback2, callback_group=self.mutually_exclusive_group)

        # Suscriptor (asociado al grupo reentrante)
        self.subscriber = self.create_subscription(
            String,
            'chatter1',
            self.subscription_callback,
            10,
            callback_group=self.reentrant_group
        )

    def timer_callback1(self):
        """Callback del primer publicador."""
        msg = String()
        msg.data = f'Publicador 1: Hola Mundo {self.i}'
        self.i += 1
        self.get_logger().info(f'Publicador 1 enviando: "{msg.data}"')
        self.pub1.publish(msg)

    def timer_callback2(self):
        """Callback del segundo publicador."""
        msg = String()
        msg.data = f'Publicador 2: Hola Mundo {self.i}'
        self.i += 1
        self.get_logger().info(f'Publicador 2 enviando: "{msg.data}"')
        self.pub2.publish(msg)

    def subscription_callback(self, msg):
        """Callback del suscriptor."""
        self.get_logger().info(f'Suscriptor recibió: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    try:
        # Crear el nodo
        double_talker = DoubleTalker()

        # Usar un MultiThreadedExecutor para permitir concurrencia
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(double_talker)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            double_talker.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

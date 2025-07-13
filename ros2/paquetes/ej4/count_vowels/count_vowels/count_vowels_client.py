import rclpy
from rclpy.node import Node
from custom_interfaces.srv import StringToInt

class VowelCountClient(Node):
    def __init__(self):
        super().__init__('vowel_count_client')
        self.client = self.create_client(StringToInt, 'count_vowels')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio...')

        self.request = StringToInt.Request()

    def send_request(self, word):
        self.request.string = word
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    client = VowelCountClient()

    # Solicita la palabra al usuario
    word = input("Introduce una palabra: ")
    client.send_request(word)

    # Espera a que la respuesta esté lista
    rclpy.spin_until_future_complete(client, client.future)

    # Maneja la respuesta
    if client.future.result() is not None:
        response = client.future.result()
        print(f"La palabra tiene {response.integer} vocales.")
    else:
        client.get_logger().error('No se recibió respuesta del servicio.')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

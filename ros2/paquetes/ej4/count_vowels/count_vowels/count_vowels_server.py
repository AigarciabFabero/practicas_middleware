import rclpy
from rclpy.node import Node
from custom_interfaces.srv import StringToInt

class VowelCountService(Node):
    def __init__(self):
        super().__init__('vowel_count_service')
        self.srv = self.create_service(StringToInt, 'count_vowels', self.handle_service_request)

    def handle_service_request(self, request, response):
        word = request.string
        count = sum(1 for char in word if char.lower() in 'aeiou')
        response.integer = count
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VowelCountService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
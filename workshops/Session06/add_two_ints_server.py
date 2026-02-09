"""
ROS 2 service server using example_interfaces/srv/AddTwoInts.

Concepts:
- Service definition: request + response message pair
- Service server: waits for requests and returns a response
- Callback: executed once per service call
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create a service named 'add_two_ints'
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

        self.get_logger().info('AddTwoInts service ready')

    def add_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        # Access request fields
        response.sum = request.a + request.b

        self.get_logger().info(
            f"Request: a={request.a}, b={request.b} -> sum={response.sum}"
        )

        return response


def main():
    rclpy.init()
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

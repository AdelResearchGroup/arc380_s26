"""
ROS 2 service client using example_interfaces/srv/AddTwoInts.

Concepts:
- Client: sends a request to a service server
- Future: placeholder for a result that arrives asynchronously
- spin_until_future_complete(): waits while processing callbacks
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Block until the service becomes available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for add_two_ints service...')

    def send_request(self, a: int, b: int):
        request = AddTwoInts.Request()
        request.a = int(a)
        request.b = int(b)

        return self.cli.call_async(request)


def main():
    rclpy.init()
    node = AddTwoIntsClient()

    # Default values if none are provided
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 3

    future = node.send_request(a, b)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            f"Result: {a} + {b} = {future.result().sum}"
        )
    else:
        node.get_logger().error(f"Service call failed: {future.exception()}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

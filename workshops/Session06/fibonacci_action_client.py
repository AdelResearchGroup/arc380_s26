"""
Minimal ROS 2 action client example.

Concepts:
- ActionClient: sends goals, receives feedback/results asynchronously
- send_goal_async(): returns a Future for the goal handle
- get_result_async(): returns a Future for the final result
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order: int):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = int(order)

        # Wait for the action server to be available
        self._client.wait_for_server()

        self.get_logger().info(f"Sending goal: order={order}")
        return self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        # feedback_msg.feedback is the Feedback message
        seq = feedback_msg.feedback.sequence
        self.get_logger().info(f"Feedback: {seq}")


def main():
    rclpy.init()
    node = FibonacciActionClient()

    order = int(sys.argv[1]) if len(sys.argv) > 1 else 8

    goal_future = node.send_goal(order)
    rclpy.spin_until_future_complete(node, goal_future)

    goal_handle = goal_future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Goal accepted")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result
    node.get_logger().info(f"Result: {result.sequence}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

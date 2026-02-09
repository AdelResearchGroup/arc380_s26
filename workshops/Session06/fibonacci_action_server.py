"""
Minimal ROS 2 action server example.

Concepts:
- Action: long-running goal/feedback/result pattern
- ActionServer: accepts goals, can send periodic feedback, and returns a result
- GoalHandle: used to check cancellation, publish feedback, and mark success
"""

import time

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create an action server on name 'fibonacci'
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request: Fibonacci.Goal):
        # Decide whether to accept/reject a goal
        self.get_logger().info(f"Received goal request: order={goal_request.order}")
        if goal_request.order < 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # This runs in an executor thread when a goal is accepted.
        self.get_logger().info("Executing goal...")

        feedback_msg = Fibonacci.Feedback()
        sequence = [0, 1]

        # Handle small orders
        order = goal_handle.request.order
        if order == 0:
            sequence = [0]
        elif order == 1:
            sequence = [0, 1]

        # Publish initial feedback
        feedback_msg.sequence = sequence
        goal_handle.publish_feedback(feedback_msg)

        # Compute the sequence with periodic feedback
        for i in range(2, order + 1):
            # Check if the client asked to cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                result = Fibonacci.Result()
                result.sequence = sequence
                return result

            sequence.append(sequence[i - 1] + sequence[i - 2])
            feedback_msg.sequence = sequence
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate a long-running task
            time.sleep(0.5)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info("Goal succeeded")
        return result


def main():
    rclpy.init()
    node = FibonacciActionServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

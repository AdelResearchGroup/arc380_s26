"""
Minimal ROS 2 publisher example.

Concepts:
- Node: basic executable unit in ROS 2 (has a name, can create pub/sub/etc.)
- Publisher: sends messages on a topic
- Timer: calls a function periodically (common pattern for publishing)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Create a publisher on topic 'chatter' with message type String.
        # The last argument is the queue size (history depth) for outgoing messages.
        self.pub = self.create_publisher(String, 'chatter', 10)

        self.count = 0

        # Timer triggers publish_callback every 0.5 seconds.
        self.timer = self.create_timer(0.5, self.publish_callback)

    def publish_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! count={self.count}'
        self.pub.publish(msg)

        # get_logger() is ROS 2 logging (goes to console by default)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main():
    # Initialize rclpy (ROS 2 Python client library)
    rclpy.init()

    node = Talker()

    # spin() processes callbacks (timers, subscriptions, services, actions...)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Minimal ROS 2 subscriber example.

Concepts:
- Subscriber: receives messages from a topic
- Callback: function invoked when a message arrives
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')

        # Create a subscription to topic 'chatter' with a callback.
        # Queue size is the incoming message buffer depth.
        self.sub = self.create_subscription(String, 'chatter', self.msg_callback, 10)

    def msg_callback(self, msg: String):
        # This runs whenever a new message arrives on 'chatter'.
        self.get_logger().info(f'I heard: "{msg.data}"')


def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

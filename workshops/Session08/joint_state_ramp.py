"""
Minimal joint state ramp publisher.

Publishes sensor_msgs/JointState on /joint_states.
Moves ONE joint back-and-forth between fixed limits at a fixed speed.
All other joints are published as 0.0 each cycle.

"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRamp(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_ramp")

        self.all_joints = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.publish_rate_hz = 50.0    # Hz
        
        self.joint_name = "joint_1"
        self.min_value = -1.0          # rad
        self.max_value = 1.0           # rad
        self.speed = 0.5               # rad/s

        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        self.value = self.min_value
        self.direction = 1.0  # +1 increasing, -1 decreasing
        self.last_time = self.get_clock().now()

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.on_timer)

        if self.joint_name not in self.all_joints:
            raise RuntimeError(f"joint_name='{self.joint_name}' must be in all_joints={self.all_joints}")

        self.get_logger().info(
            f"Publishing /joint_states: moving '{self.joint_name}' in "
            f"[{self.min_value:.2f}, {self.max_value:.2f}] rad at {self.speed:.2f} rad/s"
        )

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Integrate joint value
        self.value += self.direction * self.speed * dt

        # Bounce at limits
        if self.value >= self.max_value:
            self.value = self.max_value
            self.direction = -1.0
        elif self.value <= self.min_value:
            self.value = self.min_value
            self.direction = 1.0

        # Publish all joints (others = 0.0)
        positions = [0.0] * len(self.all_joints)
        idx = self.all_joints.index(self.joint_name)
        positions[idx] = float(self.value)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self.all_joints
        msg.position = positions
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStateRamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

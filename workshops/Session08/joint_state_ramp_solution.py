"""
Minimal joint state ramp publisher (speed is a ROS parameter).

Publishes sensor_msgs/JointState on /joint_states.
Moves ONE joint back-and-forth between fixed limits.
All other joints are published as 0.0 each cycle.

Parameters (editable at runtime):
- speed (double, rad/s), must be > 0
- joint_name (string), must be in all_joints
- min_value (double, rad), must be < max_value
- max_value (double, rad), must be > min_value
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState


class JointStateRamp(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_ramp")

        self.all_joints = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.publish_rate_hz = 50.0  # Hz

        # --- ROS parameters ---
        self.declare_parameter("speed", 0.5)  # rad/s
        self.speed = float(self.get_parameter("speed").value)

        self.declare_parameter("joint_name", "joint_1")
        self.joint_name = self.get_parameter("joint_name").value

        self.declare_parameter("min_value", -1.0)  # rad
        self.min_value = float(self.get_parameter("min_value").value)

        self.declare_parameter("max_value", 1.0)  # rad
        self.max_value = float(self.get_parameter("max_value").value)

        # Callback for updating parameters at runtime
        self.add_on_set_parameters_callback(self._on_param_set)
        # -------------------------------

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

    # New callback to allow updating parameters at runtime
    def _on_param_set(self, params: list[Parameter]) -> SetParametersResult:
        """Allow updating parameters while the node is running."""
        for p in params:
            if p.name == "speed":
                new_speed = float(p.value)
                # Make sure the new speed is positive
                if new_speed <= 0.0:
                    return SetParametersResult(successful=False, reason="speed must be > 0")
                self.speed = new_speed
                self.get_logger().info(f"Updated speed = {self.speed:.3f} rad/s")

            if p.name == "joint_name":
                new_joint_name = p.value
                # Make sure the new joint name is valid
                if new_joint_name not in self.all_joints:
                    return SetParametersResult(successful=False, reason=f"joint_name must be in {self.all_joints}")
                self.joint_name = new_joint_name
                self.get_logger().info(f"Updated joint_name = '{self.joint_name}'")

            if p.name == "min_value":
                new_min_value = float(p.value)
                # Make sure the new min_value is less than max_value
                if new_min_value >= self.max_value:
                    return SetParametersResult(successful=False, reason="min_value must be < max_value")
                # We should also check that the value is within the robot joint limits, but for simplicity we skip that here
                # You will be implementing joint limit checks in Assignment 3
                self.min_value = new_min_value
                self.get_logger().info(f"Updated min_value = {self.min_value:.3f} rad")

            if p.name == "max_value":
                new_max_value = float(p.value)
                # Make sure the new max_value is greater than min_value
                if new_max_value <= self.min_value:
                    return SetParametersResult(successful=False, reason="max_value must be > min_value")
                # We should also check that the value is within the robot joint limits, but for simplicity we skip that here
                # You will be implementing joint limit checks in Assignment 3
                self.max_value = new_max_value
                self.get_logger().info(f"Updated max_value = {self.max_value:.3f} rad")

        return SetParametersResult(successful=True)

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

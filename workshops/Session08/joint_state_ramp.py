from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState


class JointStateRamp(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_ramp")

        # Parameters
        pass

        # Internal state
        self.joint_name: str = "joint_1"
        self.min_value: float = -1.0
        self.max_value: float = 1.0
        self.speed: float = 0.5
        self.publish_rate_hz: float = 50.0
        self.all_joints: list[str] = []

        self.value: float = 0.0
        self.direction: float = 1.0  # +1 increasing, -1 decreasing

        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        self.timer = None
        self.last_time = self.get_clock().now()

        # Apply initial params and create timer
        pass

        # Allow runtime updates via rqt / ros2 param set
        pass

        self.get_logger().info(
            f"Publishing /joint_states for '{self.joint_name}' (others 0.0) "
            f"range=[{self.min_value:.3f}, {self.max_value:.3f}] rad "
            f"speed={self.speed:.3f} rad/s rate={self.publish_rate_hz:.1f} Hz"
        )

    def _apply_params_from_server(self, reset_motion: bool) -> None:
        pass

    def _on_param_set(self, params: list[Parameter]) -> SetParametersResult:
        pass

    def _on_timer(self) -> None:
        pass


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

"""depth_sim ROS 2 node (§9.2, T1.3).

Sensor-model node (H2): depth = -ground_truth_z + N(0, noise_sigma_m),
published at rate_hz on /vehicle/depth (ps11_interfaces/Depth).
"""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from ps11_interfaces.msg import Depth
from rclpy.node import Node

from ps11_nav.error_model import depth_noise_sample


class DepthSimNode(Node):
    """Publish noisy depth derived from simulator ground truth."""

    def __init__(self) -> None:
        super().__init__("depth_sim")

        # Parameters (defaults mirror nav.yaml; the params file is authoritative)
        self.declare_parameter("noise_sigma_m", 0.02)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("seed", 42)

        self._sigma_m = (
            self.get_parameter("noise_sigma_m").get_parameter_value().double_value
        )
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be > 0")

        self._rng = np.random.default_rng(seed)
        self._latest_z_m: float | None = None

        # Honesty rule H2: depth_sim is a sensor-model node and may read /sim/*.
        self.create_subscription(Odometry, "/sim/gt/odom", self._on_gt_odom, 10)
        self._depth_pub = self.create_publisher(Depth, "/vehicle/depth", 10)

        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"Initialized depth_sim: noise_sigma_m={self._sigma_m}, "
            f"rate_hz={rate_hz}, seed={seed}"
        )

    def _on_gt_odom(self, msg: Odometry) -> None:
        self._latest_z_m = float(msg.pose.pose.position.z)

    def _on_timer(self) -> None:
        if self._latest_z_m is None:
            self.get_logger().warning(
                "No /sim/gt/odom received yet; nothing to publish.",
                throttle_duration_sec=5.0,
            )
            return
        depth_m = -self._latest_z_m + depth_noise_sample(self._rng, self._sigma_m)
        msg = Depth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.depth_m = float(depth_m)
        msg.variance = float(self._sigma_m**2)
        self._depth_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

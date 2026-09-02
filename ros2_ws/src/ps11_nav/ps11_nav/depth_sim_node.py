"""ROS 2 node for depth_sim (§9.2, T1.3)."""

from __future__ import annotations

from nav_msgs.msg import Odometry
import numpy as np
from ps11_common.params import load_yaml
from ps11_interfaces.msg import Depth
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DepthSimNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_sim")

        nav_cfg = load_yaml("nav.yaml").get("depth_sim", {})
        self.noise_sigma = float(nav_cfg.get("noise_sigma_m", 0.02))
        self.variance = float(self.noise_sigma**2)
        self.rng = np.random.default_rng()

        self.latest_gt_z: float | None = None
        self.latest_stamp = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_gt = self.create_subscription(
            Odometry,
            "/sim/gt/odom",
            self._on_gt_odom,
            qos,
        )
        self.pub_depth = self.create_publisher(
            Depth,
            "/vehicle/depth",
            qos,
        )

        # Publish at 10 Hz (§14.1)
        self.timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info("depth_sim node initialized (/vehicle/depth at 10 Hz)")

    def _on_gt_odom(self, msg: Odometry) -> None:
        self.latest_gt_z = msg.pose.pose.position.z
        self.latest_stamp = msg.header.stamp

    def _on_timer(self) -> None:
        if self.latest_gt_z is None:
            return

        noise = float(self.rng.normal(0.0, self.noise_sigma))
        depth_val = float(-self.latest_gt_z + noise)

        msg = Depth()
        if self.latest_stamp is not None:
            msg.header.stamp = self.latest_stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "depth_sensor_link"
        msg.depth_m = depth_val
        msg.variance = self.variance

        self.pub_depth.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

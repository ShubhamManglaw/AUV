"""ROS 2 node for range_adapter (§9.2, T1.3)."""

from __future__ import annotations

import math
import rclpy
from ps11_common.params import load_yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range


class RangeAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("range_adapter")

        nav_cfg = load_yaml("nav.yaml").get("range_adapter", {})
        self.min_range = float(nav_cfg.get("min_range_m", 0.2))
        self.max_range = float(nav_cfg.get("max_range_m", 50.0))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_scan = self.create_subscription(
            LaserScan,
            "/vehicle/altimeter/scan",
            self._on_scan,
            qos,
        )
        self.pub_altitude = self.create_publisher(
            Range,
            "/vehicle/altitude",
            qos,
        )

        self.get_logger().info(
            "range_adapter node initialized (/vehicle/altimeter/scan -> /vehicle/altitude)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        range_val = float("nan")
        for r in msg.ranges:
            if (
                not math.isnan(r)
                and not math.isinf(r)
                and self.min_range <= r <= self.max_range
            ):
                range_val = float(r)
                break

        range_msg = Range()
        range_msg.header = msg.header
        range_msg.header.frame_id = "altimeter_link"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.05
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range
        range_msg.range = range_val

        self.pub_altitude.publish(range_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RangeAdapterNode()
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

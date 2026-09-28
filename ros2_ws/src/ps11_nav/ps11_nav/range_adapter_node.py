"""range_adapter ROS 2 node (§9.2, T1.3).

Converts the Gazebo altimeter LaserScan (/vehicle/altimeter/scan) into a
sensor_msgs/Range on /vehicle/altitude. Emits the minimum finite reading
inside [min_range_m, max_range_m]; +Inf when no return falls inside the
limits (over range).
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class RangeAdapterNode(Node):
    """LaserScan -> Range adapter for the down-looking altimeter."""

    def __init__(self) -> None:
        super().__init__("range_adapter")

        # Parameters (defaults mirror nav.yaml; the params file is authoritative)
        self.declare_parameter("min_range_m", 0.2)
        self.declare_parameter("max_range_m", 50.0)

        self._min_range_m = (
            self.get_parameter("min_range_m").get_parameter_value().double_value
        )
        self._max_range_m = (
            self.get_parameter("max_range_m").get_parameter_value().double_value
        )

        self.create_subscription(
            LaserScan, "/vehicle/altimeter/scan", self._on_scan, 10
        )
        self._range_pub = self.create_publisher(Range, "/vehicle/altitude", 10)

        self.get_logger().info(
            f"Initialized range_adapter: min_range_m={self._min_range_m}, "
            f"max_range_m={self._max_range_m}"
        )

    def _on_scan(self, scan: LaserScan) -> None:
        valid = [
            r
            for r in scan.ranges
            if math.isfinite(r) and self._min_range_m <= r <= self._max_range_m
        ]

        msg = Range()
        msg.header = scan.header
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(scan.angle_max - scan.angle_min)
        msg.min_range = self._min_range_m
        msg.max_range = self._max_range_m
        if valid:
            msg.range = float(min(valid))
        else:
            msg.range = float("inf")  # over range: no return inside the limits
            self.get_logger().warning(
                "Altimeter scan had no finite return inside the range limits.",
                throttle_duration_sec=5.0,
            )
        self._range_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RangeAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""scheduler ROS 2 node (§11.7, T3.3).

Thin wrapper around the ROS-free SemanticPolicy. Subscribes to
/vehicle/contacts (ContactArray), /vehicle/nav/odom, /vehicle/mission/state
and the link's /link/tx_ready (pull backpressure); on each tx_ready the
policy decides one frame and the node publishes it on /link/tx
(ps11_interfaces/LinkFrame) — never faster than the link asks.

The codec (frozen) does the encoding; class priorities come from classes.yaml
via ps11_common.classes; all weights/thresholds come from scheduler.yaml; the
frame payload size comes from link_profiles.yaml (profile parameter, m64).
"""

from __future__ import annotations

import math
from dataclasses import replace

import rclpy
from nav_msgs.msg import Odometry
from ps11_common.classes import ClassDatabase
from ps11_common.params import load_yaml
from ps11_interfaces.msg import ContactArray, LinkFrame
from rclpy.node import Node
from std_msgs.msg import Empty, UInt8

from ps11_telemetry.scheduler_logic import (
    ContactInput,
    SemanticPolicy,
    VehicleState,
)


class SchedulerNode(Node):
    """Pull-driven semantic telemetry scheduler."""

    def __init__(self) -> None:
        super().__init__("scheduler")

        self.declare_parameter("profile", "m64")
        profile = self.get_parameter("profile").get_parameter_value().string_value

        cfg = load_yaml("scheduler.yaml")
        priorities = {c.id: float(c.priority) for c in ClassDatabase().all_classes()}
        profiles = load_yaml("link_profiles.yaml").get("profiles", {})
        if profile not in profiles:
            raise ValueError(f"link profile '{profile}' not in link_profiles.yaml")
        frame_payload_bytes = int(profiles[profile]["frame_payload_bytes"])

        self._policy = SemanticPolicy(cfg, priorities, frame_payload_bytes)
        self._vehicle = VehicleState()
        self._last_skipped: tuple[int, ...] = ()

        self.create_subscription(
            ContactArray, "/vehicle/contacts", self._on_contacts, 10
        )
        self.create_subscription(Odometry, "/vehicle/nav/odom", self._on_odom, 10)
        self.create_subscription(UInt8, "/vehicle/mission/state", self._on_state, 10)
        self.create_subscription(Empty, "/link/tx_ready", self._on_tx_ready, 10)
        self._tx_pub = self.create_publisher(LinkFrame, "/link/tx", 10)

        self.get_logger().info(
            f"Initialized scheduler: policy={cfg.get('policy')}, profile={profile} "
            f"({frame_payload_bytes} B frames), priorities={priorities}"
        )

    def _on_contacts(self, msg: ContactArray) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        contacts = [
            ContactInput(
                contact_id=int(c.contact_id),
                class_id=int(c.class_id),
                confidence=float(c.confidence),
                x_m=float(c.position.x),
                y_m=float(c.position.y),
                depth_m=float(c.depth_m),
                sigma_xy_m=float(c.sigma_xy_m),
                t_last_seen_s=float(c.last_seen.sec + c.last_seen.nanosec * 1e-9),
            )
            for c in msg.contacts
        ]
        self._last_skipped = self._policy.update_contacts(contacts, now)
        if self._last_skipped:
            self.get_logger().warning(
                f"skipped contacts violating codec id ranges: {list(self._last_skipped)}"
            )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._vehicle = replace(
            self._vehicle,
            x_m=float(p.x),
            y_m=float(p.y),
            z_m=float(p.z),
            yaw_rad=yaw,
        )

    def _on_state(self, msg: UInt8) -> None:
        self._vehicle = replace(self._vehicle, mission_state=int(msg.data))

    def _on_tx_ready(self, _msg: Empty) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        decision = self._policy.on_frame(now, self._vehicle)
        if decision.payload is None:
            return
        msg = LinkFrame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.seq = self._policy.seq
        msg.payload = list(decision.payload)
        self._tx_pub.publish(msg)
        self.get_logger().debug(f"tx frame seq={msg.seq} kind={decision.kind}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = SchedulerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

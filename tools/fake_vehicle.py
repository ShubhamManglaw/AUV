#!/usr/bin/env python3
"""Fake vehicle publisher for testing telemetry pipeline without Gazebo (§11.7, Q4).

Publishes:
  /vehicle/contacts (ps11_interfaces/ContactArray): 6 fixed contacts (2 debris, 1 starfish, 1 urchin, 2 scallops)
  /vehicle/nav/odom (nav_msgs/Odometry): slowly moving odometry along +X
  /vehicle/mission/state (std_msgs/UInt8): mission state 2 (SURVEY)
  /clock (rosgraph_msgs/Clock): simulated clock when run standalone
"""

from __future__ import annotations

import argparse
import time

import nav_msgs.msg
import rclpy
from geometry_msgs.msg import Point
from ps11_interfaces.msg import Contact, ContactArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header, UInt8


class FakeVehicleNode(Node):
    """Generates synthetic contacts, odometry, and mission state."""

    def __init__(self, publish_clock: bool = True, duration_s: float = 90.0) -> None:
        super().__init__("fake_vehicle")
        self.publish_clock = publish_clock
        self.duration_s = duration_s
        self.start_wall_time = time.monotonic()
        self.sim_time_s = 0.0

        # Best effort QoS for odom
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )

        # Publishers
        self.contacts_pub = self.create_publisher(ContactArray, "/vehicle/contacts", 10)
        self.odom_pub = self.create_publisher(
            nav_msgs.msg.Odometry, "/vehicle/nav/odom", best_effort_qos
        )
        self.state_pub = self.create_publisher(UInt8, "/vehicle/mission/state", 10)

        if self.publish_clock:
            self.clock_pub = self.create_publisher(Clock, "/clock", 10)
            # 50 Hz clock timer
            self.clock_timer = self.create_timer(0.02, self._tick_clock)

        # Timers for vehicle data
        self.contacts_timer = self.create_timer(0.5, self._publish_contacts)  # 2 Hz
        self.odom_timer = self.create_timer(0.1, self._publish_odom)  # 10 Hz
        self.state_timer = self.create_timer(1.0, self._publish_state)  # 1 Hz

        self.get_logger().info(
            f"fake_vehicle started: publish_clock={publish_clock}, duration={duration_s}s"
        )

    def _get_current_time(self) -> tuple[int, int]:
        if self.publish_clock:
            sec = int(self.sim_time_s)
            nanosec = int((self.sim_time_s - sec) * 1e9)
            return sec, nanosec
        else:
            now = self.get_clock().now()
            return now.seconds_nanoseconds()

    def _tick_clock(self) -> None:
        self.sim_time_s += 0.02
        clock_msg = Clock()
        sec = int(self.sim_time_s)
        nanosec = int((self.sim_time_s - sec) * 1e9)
        clock_msg.clock.sec = sec
        clock_msg.clock.nanosec = nanosec
        self.clock_pub.publish(clock_msg)

    def _publish_contacts(self) -> None:
        sec, nanosec = self._get_current_time()
        hdr = Header()
        hdr.stamp.sec = sec
        hdr.stamp.nanosec = nanosec
        hdr.frame_id = "map"

        # 6 fixed contacts (2 debris, 1 starfish, 1 sea_urchin, 2 scallops)
        raw_specs = [
            (1, 0, 0.85, 10.0, 5.0, -15.0, 0.4),  # debris
            (2, 0, 0.75, 15.0, 8.0, -15.0, 0.5),  # debris
            (3, 1, 0.70, 8.0, 12.0, -15.0, 0.5),  # starfish
            (4, 2, 0.65, 12.0, 16.0, -15.0, 0.6),  # sea_urchin
            (5, 3, 0.80, 18.0, 18.0, -15.0, 0.5),  # scallop
            (6, 3, 0.85, 22.0, 10.0, -15.0, 0.6),  # scallop
        ]

        contacts_msg = ContactArray()
        contacts_msg.header = hdr

        for cid, cls_id, conf, x, y, z, sigma in raw_specs:
            c = Contact()
            c.header = hdr
            c.contact_id = cid
            c.class_id = cls_id
            c.confidence = conf
            c.position = Point(x=x, y=y, z=z)
            c.sigma_xy_m = sigma
            c.depth_m = max(0.0, -z)
            contacts_msg.contacts.append(c)

        self.contacts_pub.publish(contacts_msg)

    def _publish_odom(self) -> None:
        sec, nanosec = self._get_current_time()
        t = sec + nanosec * 1e-9

        odom = nav_msgs.msg.Odometry()
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        # Slow forward movement along +X at 0.5 m/s
        odom.pose.pose.position.x = 0.5 * t
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = -12.5

        # Facing East (+X), yaw = 0 rad -> heading = 90 deg
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0

        self.odom_pub.publish(odom)

    def _publish_state(self) -> None:
        state_msg = UInt8()
        state_msg.data = 2  # SURVEY
        self.state_pub.publish(state_msg)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fake vehicle telemetry test publisher"
    )
    parser.add_argument("--no-clock", action="store_true", help="Do not publish /clock")
    parser.add_argument(
        "--duration", type=float, default=95.0, help="Run duration in seconds"
    )
    args = parser.parse_args()

    rclpy.init()
    node = FakeVehicleNode(publish_clock=not args.no_clock, duration_s=args.duration)

    start = time.time()
    try:
        while rclpy.ok() and (time.time() - start < args.duration):
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

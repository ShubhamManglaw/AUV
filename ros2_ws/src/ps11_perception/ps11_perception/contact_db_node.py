"""ROS 2 Contact Database Node (§10.8).

Subscribes:
  /vehicle/perception/observations (ps11_interfaces/ObservationArray)

Publishes:
  /vehicle/contacts (ps11_interfaces/ContactArray) at 2 Hz
  /vehicle/markers (visualization_msgs/MarkerArray) at 2 Hz
"""

from __future__ import annotations

from typing import Any

import rclpy
from geometry_msgs.msg import Point
from ps11_common.classes import ClassDatabase
from ps11_interfaces.msg import Contact, ContactArray, ObservationArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from ps11_perception.fusion import ContactDatabase, FusedContact


class ContactDbNode(Node):
    """Fuses observations into unique contacts and publishes contacts and markers at 2 Hz."""

    def __init__(self) -> None:
        super().__init__("contact_db")

        self.declare_parameter("gate_m", 2.0)
        self.declare_parameter("sigma_floor_m", 0.3)
        self.declare_parameter("publish_rate_hz", 2.0)

        gate_m = float(self.get_parameter("gate_m").value)
        sigma_floor_m = float(self.get_parameter("sigma_floor_m").value)
        pub_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.db = ContactDatabase(gate_m=gate_m, sigma_floor_m=sigma_floor_m)
        self.class_db = ClassDatabase()

        # Class colors (R, G, B, A)
        self.class_colors = {
            0: (1.0, 0.2, 0.2, 0.8),  # debris: red
            1: (1.0, 0.65, 0.0, 0.8),  # starfish: orange
            2: (0.7, 0.1, 0.9, 0.8),  # sea_urchin: purple
            3: (0.1, 0.8, 0.9, 0.8),  # scallop: cyan
        }

        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.contact_pub = self.create_publisher(
            ContactArray, "/vehicle/contacts", qos_rel
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/vehicle/markers", qos_rel
        )

        self.create_subscription(
            ObservationArray,
            "/vehicle/perception/observations",
            self._on_observations,
            qos_rel,
        )

        period = 1.0 / max(0.1, pub_rate_hz)
        self.timer = self.create_timer(period, self._on_publish_timer)

        self.get_logger().info(
            f"ContactDbNode ready (gate={gate_m} m, floor={sigma_floor_m} m, rate={pub_rate_hz} Hz)"
        )

    def _on_observations(self, msg: ObservationArray) -> None:
        for obs in msg.observations:
            self.db.update(
                track_id=int(obs.track_id),
                class_id=int(obs.class_id),
                confidence=float(obs.confidence),
                x=float(obs.position.x),
                y=float(obs.position.y),
                z=float(obs.position.z),
                sigma_xy_m=float(obs.sigma_xy_m),
                stamp=obs.header.stamp,
            )

    def _on_publish_timer(self) -> None:
        contacts = self.db.contacts
        now = self.get_clock().now().to_msg()

        # 1. Publish ContactArray
        contact_array = ContactArray()
        contact_array.header.stamp = now
        contact_array.header.frame_id = "map"

        for c in contacts:
            msg = Contact()
            msg.header.stamp = now
            msg.header.frame_id = "map"
            msg.contact_id = int(c.contact_id)
            msg.class_id = int(c.class_id)
            msg.confidence = float(c.confidence)
            msg.position = Point(x=float(c.x), y=float(c.y), z=float(c.z))
            msg.sigma_xy_m = float(c.sigma_xy_m)
            msg.depth_m = float(c.depth_m)
            if hasattr(c.first_seen_stamp, "sec"):
                msg.first_seen = c.first_seen_stamp
            else:
                msg.first_seen = now
            if hasattr(c.last_seen_stamp, "sec"):
                msg.last_seen = c.last_seen_stamp
            else:
                msg.last_seen = now
            msg.sightings = int(c.sightings)
            contact_array.contacts.append(msg)

        self.contact_pub.publish(contact_array)

        # 2. Publish MarkerArray
        marker_array = self._create_markers(contacts, now)
        self.marker_pub.publish(marker_array)

    def _create_markers(self, contacts: list[FusedContact], now: Any) -> MarkerArray:
        marker_array = MarkerArray()
        for c in contacts:
            c_name = self.class_db.id_to_name(c.class_id)
            color = self.class_colors.get(c.class_id, (1.0, 1.0, 1.0, 0.8))

            # Sphere marker for contact location
            sphere = Marker()
            sphere.header.stamp = now
            sphere.header.frame_id = "map"
            sphere.ns = "vehicle_contacts"
            sphere.id = c.contact_id * 3
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(c.x)
            sphere.pose.position.y = float(c.y)
            sphere.pose.position.z = float(c.z)
            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4
            sphere.color.r = color[0]
            sphere.color.g = color[1]
            sphere.color.b = color[2]
            sphere.color.a = color[3]
            marker_array.markers.append(sphere)

            # Uncertainty cylinder (flat circle on seabed)
            cyl = Marker()
            cyl.header.stamp = now
            cyl.header.frame_id = "map"
            cyl.ns = "vehicle_contacts"
            cyl.id = c.contact_id * 3 + 1
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = float(c.x)
            cyl.pose.position.y = float(c.y)
            cyl.pose.position.z = float(c.z)
            diam = float(2.0 * c.sigma_xy_m)
            cyl.scale.x = diam
            cyl.scale.y = diam
            cyl.scale.z = 0.05
            cyl.color.r = color[0]
            cyl.color.g = color[1]
            cyl.color.b = color[2]
            cyl.color.a = 0.25
            marker_array.markers.append(cyl)

            # Text label: <class> #<id> <conf>
            text = Marker()
            text.header.stamp = now
            text.header.frame_id = "map"
            text.ns = "vehicle_contacts"
            text.id = c.contact_id * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(c.x)
            text.pose.position.y = float(c.y)
            text.pose.position.z = float(c.z) + 0.6
            text.scale.z = 0.35
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{c_name} #{c.contact_id} ({c.confidence:.2f})"
            marker_array.markers.append(text)

        return marker_array


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ContactDbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

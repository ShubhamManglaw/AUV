"""Surface decoder node for PS11 AUV telemetry (§11.8).

Subscribes ONLY to /link/rx (Honesty rule H1).
Decodes acoustic frames, unwraps modulo-2048 time, maintains contact database
and vehicle track, and publishes:
- /surface/contacts (ContactArray)
- /surface/vehicle_track (Path)
- /surface/markers (MarkerArray)
"""

import math
from typing import Any

import nav_msgs.msg
import rclpy
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from ps11_common.classes import get_class_db
from ps11_interfaces.msg import Contact as ContactMsg
from ps11_interfaces.msg import ContactArray as ContactArrayMsg
from ps11_interfaces.msg import LinkFrame
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from ps11_telemetry.surface_state import SurfaceStateTracker, VehiclePoseRecord


def float_to_builtin_time(t_s: float) -> BuiltinTime:
    """Convert float seconds to builtin_interfaces/Time."""
    sec = int(t_s)
    nanosec = int(max(0.0, (t_s - sec)) * 1e9)
    return BuiltinTime(sec=sec, nanosec=nanosec)


def hex_to_color_rgba(hex_str: str, alpha: float = 1.0) -> ColorRGBA:
    """Parse hex color like '#E4572E' to std_msgs/ColorRGBA."""
    hex_clean = hex_str.lstrip("#")
    if len(hex_clean) == 6:
        r = int(hex_clean[0:2], 16) / 255.0
        g = int(hex_clean[2:4], 16) / 255.0
        b = int(hex_clean[4:6], 16) / 255.0
    else:
        r, g, b = 1.0, 1.0, 1.0
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(alpha))


def heading_to_quaternion(heading_deg: float) -> Quaternion:
    """Convert compass heading (0=north, clockwise) to ENU yaw quaternion."""
    yaw_deg = (90.0 - heading_deg) % 360.0
    yaw_rad = math.radians(yaw_deg)
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw_rad / 2.0),
        w=math.cos(yaw_rad / 2.0),
    )


class SurfaceDecoderNode(Node):
    """ROS 2 node decoding /link/rx frames for surface operators."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__("surface_decoder", **kwargs)

        self.declare_parameter("mission_start_s", 0.0)
        self.declare_parameter("frame_id", "map")

        mission_start_s = (
            self.get_parameter("mission_start_s").get_parameter_value().double_value
        )
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        self._tracker = SurfaceStateTracker(mission_start_s=mission_start_s)
        self._class_db = get_class_db()

        # Honesty rule H1: Subscribe ONLY to /link/rx
        self._sub_rx = self.create_subscription(
            LinkFrame,
            "/link/rx",
            self._rx_callback,
            10,
        )

        # Publishers
        self._pub_contacts = self.create_publisher(
            ContactArrayMsg, "/surface/contacts", 10
        )
        self._pub_track = self.create_publisher(
            nav_msgs.msg.Path, "/surface/vehicle_track", 10
        )
        self._pub_markers = self.create_publisher(MarkerArray, "/surface/markers", 10)

        # 1 Hz timer to update marker age labels
        self._timer = self.create_timer(1.0, self._timer_callback)

        self.get_logger().info(
            f"Initialized surface_decoder (H1 compliant): mission_start_s={mission_start_s}, frame_id='{self._frame_id}'"
        )

    def _now_s(self) -> float:
        """Return current ROS clock in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _rx_callback(self, msg: LinkFrame) -> None:
        payload = bytes(msg.payload)
        now_s = self._now_s()

        updated, msg_type = self._tracker.handle_payload(payload, now_s)
        if not updated:
            return

        self._publish_contacts()
        self._publish_markers()

        if msg_type == "heartbeat":
            self._publish_track()

    def _timer_callback(self) -> None:
        # Periodic update of marker age labels
        if self._tracker.contacts or self._tracker.latest_heartbeat:
            self._publish_markers()

    def _publish_contacts(self) -> None:
        now_stamp = self.get_clock().now().to_msg()
        msg = ContactArrayMsg()
        msg.header = Header(stamp=now_stamp, frame_id=self._frame_id)

        for c in self._tracker.contacts:
            c_msg = ContactMsg()
            c_msg.header = Header(stamp=now_stamp, frame_id=self._frame_id)
            c_msg.contact_id = int(c.contact_id)
            c_msg.class_id = int(c.class_id)
            c_msg.confidence = float(c.confidence)
            # In map frame: x east, y north, depth positive down (z = -depth)
            c_msg.position = Point(
                x=float(c.x_m),
                y=float(c.y_m),
                z=-float(c.depth_m),
            )
            c_msg.sigma_xy_m = float(c.sigma_xy_m)
            c_msg.depth_m = float(c.depth_m)
            c_msg.first_seen = float_to_builtin_time(c.first_seen_s)
            c_msg.last_seen = float_to_builtin_time(c.last_seen_s)
            c_msg.sightings = int(c.sightings)
            msg.contacts.append(c_msg)

        self._pub_contacts.publish(msg)

    def _publish_track(self) -> None:
        now_stamp = self.get_clock().now().to_msg()
        path = nav_msgs.msg.Path()
        path.header = Header(stamp=now_stamp, frame_id=self._frame_id)

        for pose_rec in self._tracker.track:
            ps = PoseStamped()
            ps.header = Header(
                stamp=float_to_builtin_time(pose_rec.t_s),
                frame_id=self._frame_id,
            )
            ps.pose.position = Point(
                x=float(pose_rec.x_m),
                y=float(pose_rec.y_m),
                z=-float(pose_rec.depth_m),
            )
            ps.pose.orientation = heading_to_quaternion(pose_rec.heading_deg)
            path.poses.append(ps)

        self._pub_track.publish(path)

    def _publish_markers(self) -> None:
        now_s = self._now_s()
        now_stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # 1. Contact markers (Sphere, Text Label, Uncertainty Cylinder)
        for c in self._tracker.contacts:
            try:
                class_info = self._class_db.get_by_id(c.class_id)
                class_name = class_info.name
                color = hex_to_color_rgba(class_info.color, alpha=1.0)
                translucent_color = hex_to_color_rgba(class_info.color, alpha=0.25)
            except KeyError:
                class_name = f"class_{c.class_id}"
                color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                translucent_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.25)

            pos = Point(x=float(c.x_m), y=float(c.y_m), z=-float(c.depth_m))
            age_s = self._tracker.get_contact_age(c.contact_id, now_s)

            # Sphere marker
            m_sphere = Marker()
            m_sphere.header = Header(stamp=now_stamp, frame_id=self._frame_id)
            m_sphere.ns = "surface_contacts"
            m_sphere.id = int(c.contact_id)
            m_sphere.type = Marker.SPHERE
            m_sphere.action = Marker.ADD
            m_sphere.pose.position = pos
            m_sphere.pose.orientation.w = 1.0
            m_sphere.scale.x = 0.8
            m_sphere.scale.y = 0.8
            m_sphere.scale.z = 0.8
            m_sphere.color = color
            ma.markers.append(m_sphere)

            # Text label marker: <class> #<id> <conf> and age since last report
            m_text = Marker()
            m_text.header = Header(stamp=now_stamp, frame_id=self._frame_id)
            m_text.ns = "surface_contact_labels"
            m_text.id = int(c.contact_id)
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position = Point(
                x=float(c.x_m),
                y=float(c.y_m),
                z=-float(c.depth_m) + 0.8,
            )
            m_text.pose.orientation.w = 1.0
            m_text.scale.z = 0.4  # Text height
            m_text.text = (
                f"{class_name} #{c.contact_id} {c.confidence:.2f} ({age_s:.0f}s ago)"
            )
            m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            ma.markers.append(m_text)

            # Flat cylinder with radius = decoded sigma
            m_cyl = Marker()
            m_cyl.header = Header(stamp=now_stamp, frame_id=self._frame_id)
            m_cyl.ns = "surface_uncertainty"
            m_cyl.id = int(c.contact_id)
            m_cyl.type = Marker.CYLINDER
            m_cyl.action = Marker.ADD
            m_cyl.pose.position = pos
            m_cyl.pose.orientation.w = 1.0
            m_cyl.scale.x = float(2.0 * c.sigma_xy_m)  # Diameter x
            m_cyl.scale.y = float(2.0 * c.sigma_xy_m)  # Diameter y
            m_cyl.scale.z = 0.05  # Flat disk
            m_cyl.color = translucent_color
            ma.markers.append(m_cyl)

        # 2. Vehicle pose and heading arrow from latest heartbeat
        latest_hb: VehiclePoseRecord | None = self._tracker.latest_heartbeat
        if latest_hb is not None:
            m_arrow = Marker()
            m_arrow.header = Header(stamp=now_stamp, frame_id=self._frame_id)
            m_arrow.ns = "surface_vehicle_heading"
            m_arrow.id = 0
            m_arrow.type = Marker.ARROW
            m_arrow.action = Marker.ADD
            m_arrow.pose.position = Point(
                x=float(latest_hb.x_m),
                y=float(latest_hb.y_m),
                z=-float(latest_hb.depth_m),
            )
            m_arrow.pose.orientation = heading_to_quaternion(latest_hb.heading_deg)
            m_arrow.scale.x = 2.0  # Arrow shaft length
            m_arrow.scale.y = 0.4  # Arrow width
            m_arrow.scale.z = 0.4  # Arrow height
            m_arrow.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)
            ma.markers.append(m_arrow)

        self._pub_markers.publish(ma)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SurfaceDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

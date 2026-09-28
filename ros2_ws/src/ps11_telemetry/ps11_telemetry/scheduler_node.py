"""ROS 2 Acoustic Telemetry Scheduler Node (§11.7).

Inputs:
  /vehicle/contacts (ps11_interfaces/ContactArray)
  /vehicle/nav/odom (nav_msgs/Odometry)
  /vehicle/mission/state (std_msgs/UInt8)
  /link/tx_ready (std_msgs/Empty)

Output:
  /link/tx (ps11_interfaces/LinkFrame)
"""

from __future__ import annotations

import math

import nav_msgs.msg
import rclpy
from ps11_common.classes import get_class_db
from ps11_common.params import load_yaml
from ps11_interfaces.msg import ContactArray, LinkFrame
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Header, UInt8

from ps11_telemetry.codec import pack_frame
from ps11_telemetry.policy import CandidateContact, SemanticPolicy, VehicleState


class SchedulerNode(Node):
    """Schedules telemetry transmissions over the constrained acoustic link (§11.7)."""

    def __init__(self) -> None:
        super().__init__("scheduler")

        # Declare parameters
        self.declare_parameter("policy", "semantic")
        self.declare_parameter("profile", "m64")
        self.declare_parameter("mission_start_s", 0.0)

        policy_name = self.get_parameter("policy").get_parameter_value().string_value
        profile_name = self.get_parameter("profile").get_parameter_value().string_value
        mission_start_s = (
            self.get_parameter("mission_start_s").get_parameter_value().double_value
        )

        # Load configs
        sched_cfg = load_yaml("scheduler.yaml")
        profiles_cfg = load_yaml("link_profiles.yaml").get("profiles", {})
        prof = profiles_cfg.get(profile_name, {})

        self.frame_payload_bytes = int(prof.get("frame_payload_bytes", 8))
        self.k_slots = max(1, self.frame_payload_bytes // 8)

        # Load class priorities
        class_db = get_class_db()
        class_priorities = {c.id: c.priority for c in class_db.all_classes()}

        # Build policy
        hb_cfg = sched_cfg.get("heartbeat", {})
        score_cfg = sched_cfg.get("scoring", {})
        bat_cfg = sched_cfg.get("battery", {})

        self.policy = SemanticPolicy(
            class_priorities=class_priorities,
            hb_max_period_s=float(hb_cfg.get("max_period_s", 15.0)),
            hb_min_period_s=float(hb_cfg.get("min_period_s", 5.0)),
            update_min_move_m=float(score_cfg.get("update_min_move_m", 1.0)),
            update_sigma_ratio=float(score_cfg.get("update_sigma_ratio", 0.5)),
            update_novelty=float(score_cfg.get("update_novelty", 0.3)),
            age_boost_tau_s=float(score_cfg.get("age_boost_tau_s", 30.0)),
            age_boost_max_s=float(score_cfg.get("age_boost_max_s", 60.0)),
            battery_drain_per_hour=float(bat_cfg.get("drain_per_hour", 0.1)),
            mission_start_s=mission_start_s,
        )

        self.vehicle_state = VehicleState(mission_start_s=mission_start_s)
        self.seq = 0

        # QoS for sensor data
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )

        # Subscriptions
        self.contacts_sub = self.create_subscription(
            ContactArray,
            "/vehicle/contacts",
            self._on_contacts,
            10,
        )
        self.odom_sub = self.create_subscription(
            nav_msgs.msg.Odometry,
            "/vehicle/nav/odom",
            self._on_odom,
            best_effort_qos,
        )
        self.mission_state_sub = self.create_subscription(
            UInt8,
            "/vehicle/mission/state",
            self._on_mission_state,
            10,
        )
        self.tx_ready_sub = self.create_subscription(
            Empty,
            "/link/tx_ready",
            self._on_tx_ready,
            10,
        )

        # Publisher
        self.tx_pub = self.create_publisher(LinkFrame, "/link/tx", 10)

        self.get_logger().info(
            f"Scheduler initialized: policy={policy_name}, profile={profile_name} "
            f"({self.frame_payload_bytes} B payload, {self.k_slots} slots), "
            f"mission_start_s={mission_start_s}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_contacts(self, msg: ContactArray) -> None:
        now_s = self._now_s()
        candidates: list[CandidateContact] = []
        for c in msg.contacts:
            depth_m = max(0.0, -c.position.z)
            # Contact timestamp
            c_time = c.header.stamp.sec + c.header.stamp.nanosec * 1e-9
            if c_time <= 0.0:
                c_time = now_s

            candidates.append(
                CandidateContact(
                    id=c.contact_id,
                    class_id=c.class_id,
                    confidence=c.confidence,
                    x_m=c.position.x,
                    y_m=c.position.y,
                    depth_m=depth_m,
                    sigma_m=c.sigma_xy_m,
                    last_seen_s=c_time,
                )
            )
        self.policy.update_contacts(candidates, now_s)

    def _on_odom(self, msg: nav_msgs.msg.Odometry) -> None:
        self.vehicle_state.x_m = msg.pose.pose.position.x
        self.vehicle_state.y_m = msg.pose.pose.position.y
        self.vehicle_state.depth_m = max(0.0, -msg.pose.pose.position.z)

        # Heading calculation from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.vehicle_state.heading_deg = (90.0 - math.degrees(yaw)) % 360.0

    def _on_mission_state(self, msg: UInt8) -> None:
        self.vehicle_state.state = int(msg.data)

    def _on_tx_ready(self, _msg: Empty) -> None:
        now_s = self._now_s()
        messages = self.policy.select_messages_for_frame(
            k_slots=self.k_slots,
            now_s=now_s,
            vehicle=self.vehicle_state,
        )

        if not messages:
            # Silence saves modem energy
            return

        payload = pack_frame(messages, self.frame_payload_bytes)

        frame = LinkFrame()
        frame.header = Header(stamp=self.get_clock().now().to_msg())
        frame.seq = self.seq
        frame.payload = list(payload)
        self.seq += 1

        self.tx_pub.publish(frame)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

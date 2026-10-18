"""ROS 2 Metrics Node (§12.1–12.2).

Authorized honesty rules exception (H2):
- Only odom_noise, depth_sim and metrics may read /sim/*
- Only metrics may read world_objects.yaml

Publishes:
  /eval/counters (ps11_interfaces/DemoCounters) at 1 Hz

At shutdown:
  Writes results/run_<timestamp>/summary.json
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import cv2
import nav_msgs.msg
import rclpy
from ps11_common.classes import get_class_db
from ps11_common.image_utils import image_to_numpy
from ps11_common.params import load_yaml
from ps11_interfaces.msg import ContactArray, DemoCounters, LinkStats
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from ps11_bringup.metrics import (
    ContactObservation,
    EvaluationCounters,
    GroundTruthObject,
    calculate_counters,
)


class MetricsNode(Node):
    """Computes live telemetry & perception counters and writes summary at shutdown."""

    def __init__(self) -> None:
        super().__init__("metrics")

        # Declare parameters
        self.declare_parameter("scenario", "demo")
        self.declare_parameter("link_profile", "m64")
        self.declare_parameter("output_dir", "")

        scenario = self.get_parameter("scenario").get_parameter_value().string_value
        self.link_profile = (
            self.get_parameter("link_profile").get_parameter_value().string_value
        )
        custom_out_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        # Bitrate lookup
        profiles_cfg = load_yaml("link_profiles.yaml").get("profiles", {})
        prof = profiles_cfg.get(self.link_profile, {})
        self.link_bitrate_bps = int(prof.get("bitrate_bps", 64))

        # Output directory setup
        timestamp_str = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        if custom_out_dir:
            self.run_dir = Path(custom_out_dir)
        else:
            self.run_dir = Path("results") / f"run_{timestamp_str}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        # Load ground truth objects
        self.scenario = scenario
        self.gt_objects: list[GroundTruthObject] = []
        self._load_ground_truth(scenario)

        # State counters
        self.semantic_bits_sent = 0
        self.jpeg_equiv_bits = 0
        self.image_frames_counted = 0
        self.contacts_onboard_count = 0
        self.surface_contacts: list[ContactObservation] = []
        self.first_view_times: dict[int, float] = {}
        self.contact_first_surface_arrival: dict[int, float] = {}

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )

        # Subscriptions
        self.create_subscription(
            Image,
            "/vehicle/camera/image_raw",
            self._on_image,
            sensor_qos,
        )
        self.create_subscription(
            LinkStats,
            "/link/stats",
            self._on_link_stats,
            10,
        )
        self.create_subscription(
            ContactArray,
            "/vehicle/contacts",
            self._on_vehicle_contacts,
            10,
        )
        self.create_subscription(
            ContactArray,
            "/surface/contacts",
            self._on_surface_contacts,
            10,
        )
        self.create_subscription(
            nav_msgs.msg.Odometry,
            "/sim/gt/odom",
            self._on_gt_odom,
            sensor_qos,
        )

        # Publisher for live counters at 1 Hz
        self.counters_pub = self.create_publisher(DemoCounters, "/eval/counters", 10)
        self.timer = self.create_timer(1.0, self._on_publish_counters)

        self.latest_counters = EvaluationCounters()

        self.get_logger().info(
            f"Metrics node initialized: scenario={scenario} ({len(self.gt_objects)} GT objects), "
            f"link_profile={self.link_profile} ({self.link_bitrate_bps} bps), output_dir={self.run_dir}"
        )

    def _load_ground_truth(self, scenario: str) -> None:
        class_db = get_class_db()
        # Look for scenario-specific objects file or default world_objects.yaml
        filename = (
            f"world_objects_{scenario}.yaml"
            if scenario not in ("demo", "")
            else "world_objects.yaml"
        )
        try:
            cfg = load_yaml(filename)
        except (FileNotFoundError, KeyError, ValueError):
            cfg = load_yaml("world_objects.yaml")

        raw_objects = cfg.get("objects", [])
        for obj in raw_objects:
            cls_name = obj.get("class", "debris")
            try:
                cls_info = class_db.get_by_name(cls_name)
                cls_id = cls_info.id
            except KeyError:
                cls_id = int(obj.get("class_id", 0))

            self.gt_objects.append(
                GroundTruthObject(
                    id=int(obj.get("id", len(self.gt_objects))),
                    name=str(obj.get("name", f"{cls_name}_{obj.get('id', 0)}")),
                    class_id=cls_id,
                    x=float(obj.get("x", 0.0)),
                    y=float(obj.get("y", 0.0)),
                    z=float(obj.get("z", -15.0)),
                    size_m=float(obj.get("size_m", 0.5)),
                )
            )

    def _on_image(self, msg: Image) -> None:
        try:
            arr = image_to_numpy(msg)
            # Encode at JPEG quality 75 per §12.2
            success, enc = cv2.imencode(
                ".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            )
            if success:
                self.jpeg_equiv_bits += len(enc) * 8
                self.image_frames_counted += 1
        except (cv2.error, ValueError) as e:
            self.get_logger().warn(f"Failed to JPEG encode camera frame: {e}")

    def _on_link_stats(self, msg: LinkStats) -> None:
        self.semantic_bits_sent = int(msg.payload_bits_sent)

    def _on_vehicle_contacts(self, msg: ContactArray) -> None:
        self.contacts_onboard_count = len(msg.contacts)

    def _on_surface_contacts(self, msg: ContactArray) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        for c in msg.contacts:
            if c.contact_id not in self.contact_first_surface_arrival:
                self.contact_first_surface_arrival[c.contact_id] = now_s

        obs_list: list[ContactObservation] = []
        for c in msg.contacts:
            first_s = c.first_seen.sec + c.first_seen.nanosec * 1e-9
            last_s = c.last_seen.sec + c.last_seen.nanosec * 1e-9
            arr_s = self.contact_first_surface_arrival.get(c.contact_id, now_s)
            obs_list.append(
                ContactObservation(
                    contact_id=c.contact_id,
                    class_id=c.class_id,
                    x=c.position.x,
                    y=c.position.y,
                    z=c.position.z,
                    confidence=c.confidence,
                    first_seen_s=first_s,
                    last_seen_s=last_s,
                    surface_arrival_s=arr_s,
                )
            )
        self.surface_contacts = obs_list

    def _on_gt_odom(self, msg: nav_msgs.msg.Odometry) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        vx = msg.pose.pose.position.x
        vy = msg.pose.pose.position.y
        # Record first view time if vehicle is within 5 m of object
        for g in self.gt_objects:
            if g.id not in self.first_view_times:
                dist = math.hypot(g.x - vx, g.y - vy)
                if dist <= 5.0:
                    self.first_view_times[g.id] = now_s

    def _on_publish_counters(self) -> None:
        self.latest_counters = calculate_counters(
            semantic_bits_sent=self.semantic_bits_sent,
            jpeg_equiv_bits=self.jpeg_equiv_bits,
            link_bitrate_bps=self.link_bitrate_bps,
            contacts_onboard_count=self.contacts_onboard_count,
            surface_contacts=self.surface_contacts,
            gt_objects=self.gt_objects,
            first_view_times=self.first_view_times,
            max_distance_m=3.0,
        )

        msg = DemoCounters()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.semantic_bits_sent = int(self.latest_counters.semantic_bits_sent)
        msg.jpeg_equiv_bits = int(self.latest_counters.jpeg_equiv_bits)
        msg.ratio_vs_jpeg = float(self.latest_counters.ratio_vs_jpeg)
        msg.jpeg_airtime_at_link_s = float(self.latest_counters.jpeg_airtime_at_link_s)
        msg.contacts_onboard = int(self.latest_counters.contacts_onboard)
        msg.contacts_at_surface = int(self.latest_counters.contacts_at_surface)
        msg.gt_objects_in_view = int(self.latest_counters.gt_objects_total)
        msg.gt_objects_reported = int(self.latest_counters.gt_objects_reported)
        msg.mean_position_error_m = float(self.latest_counters.mean_position_error_m)
        msg.mean_first_report_latency_s = float(
            self.latest_counters.mean_first_report_latency_s
        )

        self.counters_pub.publish(msg)

    def write_summary(self) -> None:
        """Write summary.json to output directory at shutdown."""
        summary = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "scenario": self.scenario,
            "scenario_note": "demo scenario, the scenario the system was tuned on",
            "semantic_bits_sent": self.latest_counters.semantic_bits_sent,
            "jpeg_equiv_bits": self.latest_counters.jpeg_equiv_bits,
            "ratio_vs_jpeg": round(self.latest_counters.ratio_vs_jpeg, 2),
            "jpeg_airtime_at_link_s": round(
                self.latest_counters.jpeg_airtime_at_link_s, 2
            ),
            "contacts_onboard": self.latest_counters.contacts_onboard,
            "contacts_at_surface": self.latest_counters.contacts_at_surface,
            "correct_contacts_at_surface": self.latest_counters.correct_contacts_at_surface,
            "false_contacts_at_surface": self.latest_counters.false_contacts_at_surface,
            "gt_objects_total": self.latest_counters.gt_objects_total,
            "gt_objects_reported": self.latest_counters.gt_objects_reported,
            "surface_recall": round(self.latest_counters.surface_recall, 4),
            "mean_position_error_m": round(
                self.latest_counters.mean_position_error_m, 3
            ),
            "mean_first_report_latency_s": round(
                self.latest_counters.mean_first_report_latency_s, 2
            ),
            "mean_first_report_latency_note": "onboard-to-operator delay (surface arrival time minus contact first_seen onboard)",
            "mean_time_since_mission_start_s": round(
                self.latest_counters.mean_time_since_mission_start_s, 2
            ),
            "mean_time_since_mission_start_note": "time since mission start when contact arrived at surface",
            "link_profile": self.link_profile,
            "link_bitrate_bps": self.link_bitrate_bps,
            "image_frames_counted": self.image_frames_counted,
        }

        summary_file = self.run_dir / "summary.json"
        try:
            with open(summary_file, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
            print(f"[metrics] Wrote summary to {summary_file}")

            # Also create/update symlink results/latest_summary.json
            latest_file = Path("results") / "latest_summary.json"
            with open(latest_file, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
        except OSError as e:
            print(f"[metrics] Failed to write summary: {e}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Link emulator ROS 2 node (§11.6).

Thin wrapper around LinkModel using simulation time.
"""

import rclpy
from builtin_interfaces.msg import Time
from ps11_common.params import load_yaml
from ps11_interfaces.msg import LinkFrame, LinkStats
from rclpy.node import Node
from std_msgs.msg import Empty

from ps11_telemetry.link_model import LinkModel


class LinkEmulatorNode(Node):
    """Acoustic link emulator node."""

    def __init__(self) -> None:
        super().__init__("link_emulator")

        # Declare parameters
        self.declare_parameter("profile", "m64")
        self.declare_parameter("mode", "pull")
        self.declare_parameter("seed", 42)

        profile_name = self.get_parameter("profile").get_parameter_value().string_value
        mode = self.get_parameter("mode").get_parameter_value().string_value
        seed = self.get_parameter("seed").get_parameter_value().integer_value

        # Load link profiles configuration
        config = load_yaml("link_profiles.yaml")
        profiles = config.get("profiles", {})
        if profile_name not in profiles:
            self.get_logger().warn(
                f"Profile '{profile_name}' not in link_profiles.yaml; falling back to default m64."
            )
            prof = profiles.get(
                "m64",
                {
                    "label": "m64",
                    "bitrate_bps": 64,
                    "frame_payload_bytes": 8,
                    "latency_s": 0.5,
                    "loss_prob": 0.05,
                    "half_duplex": True,
                },
            )
        else:
            prof = profiles[profile_name]

        self.model = LinkModel(
            bitrate_bps=int(prof.get("bitrate_bps", 64)),
            frame_payload_bytes=int(prof.get("frame_payload_bytes", 8)),
            latency_s=float(prof.get("latency_s", 0.5)),
            loss_prob=float(prof.get("loss_prob", 0.05)),
            half_duplex=bool(prof.get("half_duplex", True)),
            label=str(prof.get("label", profile_name)),
            mode=mode,
            seed=seed,
        )

        self.get_logger().info(
            f"Initialized link_emulator: profile={profile_name} (label='{self.model.label}'), "
            f"mode={mode}, bitrate={self.model.bitrate_bps} bps, payload={self.model.frame_payload_bytes} B, "
            f"airtime={self.model.airtime_s:.3f} s, latency={self.model.latency_s:.3f} s, "
            f"loss_prob={self.model.loss_prob:.3f}"
        )

        # Publishers
        self.rx_pub = self.create_publisher(LinkFrame, "/link/rx", 10)
        self.tx_ready_pub = self.create_publisher(Empty, "/link/tx_ready", 10)
        self.stats_pub = self.create_publisher(LinkStats, "/link/stats", 10)

        # Subscriptions
        self.tx_sub = self.create_subscription(LinkFrame, "/link/tx", self._on_tx, 10)

        # Timers: step loop at 50 Hz (0.02s) and stats publication at 1 Hz (1.0s)
        self.step_timer = self.create_timer(0.02, self._on_step)
        self.stats_timer = self.create_timer(1.0, self._on_publish_stats)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _to_stamp(now_s: float) -> Time:
        sec = int(now_s)
        nanosec = round((now_s - sec) * 1e9)
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Time(sec=sec, nanosec=nanosec)

    def _on_tx(self, msg: LinkFrame) -> None:
        now_s = self._now_s()
        self.model.send_frame(bytes(msg.payload), msg.seq, now_s)

    def _on_step(self) -> None:
        now_s = self._now_s()
        rx_frames, tx_ready = self.model.step(now_s)

        for seq, payload, arr_time_s in rx_frames:
            rx_msg = LinkFrame()
            rx_msg.header.stamp = self._to_stamp(arr_time_s)
            rx_msg.header.frame_id = "link"
            rx_msg.seq = seq
            rx_msg.payload = list(payload)
            self.rx_pub.publish(rx_msg)

        if tx_ready:
            self.tx_ready_pub.publish(Empty())

    def _on_publish_stats(self) -> None:
        now_s = self._now_s()
        stats = self.model.get_stats(now_s)

        stats_msg = LinkStats()
        stats_msg.header.stamp = self.get_clock().now().to_msg()
        stats_msg.header.frame_id = "link"
        stats_msg.profile_label = stats.profile_label
        stats_msg.payload_bits_sent = stats.payload_bits_sent
        stats_msg.frames_sent = stats.frames_sent
        stats_msg.frames_lost = stats.frames_lost
        stats_msg.frames_rejected = stats.frames_rejected
        stats_msg.queue_len = stats.queue_len
        stats_msg.utilisation = float(stats.utilisation)

        self.stats_pub.publish(stats_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LinkEmulatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Node integration test for GeolocatorNode with static TF and synthetic tracks (Q2 / T2.7)."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from ps11_interfaces.msg import ObservationArray, Track, TrackArray
from ps11_perception.geolocator_node import GeolocatorNode
from rclpy.node import Node
from sensor_msgs.msg import Range
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class GeolocatorTestHarness(Node):
    """Helper node to broadcast static TF, publish synthetic tracks/altitude, and collect observations."""

    def __init__(self) -> None:
        super().__init__("geo_test_harness")
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.track_pub = self.create_publisher(
            TrackArray, "/vehicle/perception/tracks", 10
        )
        self.alt_pub = self.create_publisher(Range, "/vehicle/altitude", 10)

        self.received_observations: list[ObservationArray] = []
        self.create_subscription(
            ObservationArray,
            "/vehicle/perception/observations",
            self._on_obs,
            10,
        )

    def _on_obs(self, msg: ObservationArray) -> None:
        self.received_observations.append(msg)

    def broadcast_static_tf(self) -> None:
        # Camera at (0, 0, -12.5), pitched 45 deg down along +X in map
        # Quaternion for 45 deg pitch down:
        # Optical Z points [1/sqrt(2), 0, -1/sqrt(2)]
        # We can construct the quaternion directly:
        # Angle-axis: rotation from standard frame to optical frame pitched down
        # Or rotation matrix:
        # R = [[0, s2, s2], [-1, 0, 0], [0, s2, -s2]]
        # Trace R: R00+R11+R22 = -s2 ≈ -0.7071
        # Let's use standard quaternion for pitch 45 deg down + optical frame conversion:
        # Rotation: yaw=0, pitch=45 deg down (pitch=+0.7854 rad around Y), then standard camera optical:
        # optical link relative to vehicle base: roll=-90 deg, yaw=-90 deg
        # Alternatively, direct quaternion corresponding to:
        # R = [[0, s2, s2], [-1, 0, 0], [0, s2, -s2]]:
        # qw = 0.5 * sqrt(1 + 0 - 0 - (-s2)) = 0.5 * sqrt(1 + s2) = 0.5 * sqrt(1.7071) ≈ 0.65328
        # qx = (R21 - R12) / (4*qw) = (s2 - 0) / (4*0.65328) = 0.7071 / 2.6131 ≈ 0.27060
        # qy = (R02 - R20) / (4*qw) = (s2 - 0) / (4*0.65328) ≈ 0.27060
        # qz = (R10 - R01) / (4*qw) = (-1 - s2) / (4*0.65328) = -1.7071 / 2.6131 ≈ -0.65328
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "camera_optical_frame"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = -12.5

        tf.transform.rotation.x = 0.65328148
        tf.transform.rotation.y = 0.65328148
        tf.transform.rotation.z = 0.27059805
        tf.transform.rotation.w = 0.27059805

        self.tf_broadcaster.sendTransform(tf)


def test_geolocator_node_with_static_tf_and_synthetic_tracks() -> None:
    """GeolocatorNode receives synthetic tracks, performs TF lookup, and outputs correct ObservationArray."""
    rclpy.init()
    try:
        harness = GeolocatorTestHarness()
        geo_node = GeolocatorNode()

        # Send static TF
        harness.broadcast_static_tf()

        # Publish altitude = 2.5 m (seabed at z = -15.0)
        alt_msg = Range()
        alt_msg.range = 2.5
        harness.alt_pub.publish(alt_msg)

        # Let nodes process TF and altitude
        start_time = time.time()
        while time.time() - start_time < 0.5:
            rclpy.spin_once(harness, timeout_sec=0.05)
            rclpy.spin_once(geo_node, timeout_sec=0.05)

        # Publish synthetic track at image centre (u=320, v=240)
        track_msg = TrackArray()
        track_msg.header.stamp = harness.get_clock().now().to_msg()
        track_msg.header.frame_id = "camera_optical_frame"

        trk = Track()
        trk.header = track_msg.header
        trk.track_id = 42
        trk.class_id = 1  # starfish
        trk.confidence = 0.88
        trk.u = 320.0
        trk.v = 240.0
        trk.width = 40.0
        trk.height = 40.0
        trk.hits = 7
        track_msg.tracks.append(trk)

        harness.track_pub.publish(track_msg)

        # Spin until observation received or timeout
        start_time = time.time()
        while (
            time.time() - start_time < 3.0 and len(harness.received_observations) == 0
        ):
            rclpy.spin_once(harness, timeout_sec=0.05)
            rclpy.spin_once(geo_node, timeout_sec=0.05)

        assert len(harness.received_observations) > 0, (
            "No ObservationArray received from GeolocatorNode"
        )
        obs_array = harness.received_observations[0]
        assert len(obs_array.observations) == 1, (
            f"Expected 1 observation, got {len(obs_array.observations)}"
        )

        obs = obs_array.observations[0]
        assert obs.track_id == 42
        assert obs.class_id == 1
        assert math.isclose(obs.confidence, 0.88, abs_tol=1e-3)

        # Expected world position: camera at (0, 0, -12.5), looking 45 deg down +X:
        # hit at x = 2.5 m, y = 0.0 m, z = -15.0 m
        assert math.isclose(obs.position.x, 2.5, abs_tol=0.05), (
            f"Expected x~2.5, got {obs.position.x}"
        )
        assert math.isclose(obs.position.y, 0.0, abs_tol=0.05), (
            f"Expected y~0.0, got {obs.position.y}"
        )
        assert math.isclose(obs.position.z, -15.0, abs_tol=0.05), (
            f"Expected z~-15.0, got {obs.position.z}"
        )
        assert obs.sigma_xy_m > 0.0

    finally:
        geo_node.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()

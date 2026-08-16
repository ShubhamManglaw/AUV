"""Integration and honesty rule (H1) tests for surface_decoder node."""

import math
from typing import Any

import pytest
import rclpy
from ps11_interfaces.msg import ContactArray, LinkFrame
from ps11_telemetry.surface_decoder_node import SurfaceDecoderNode
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_h1_subscription_isolation(ros_context: Any):
    """Honesty Rule H1: Node may ONLY subscribe to /link/rx and /clock."""
    node = SurfaceDecoderNode()
    try:
        sub_topics = [sub.topic_name for sub in node.subscriptions]

        # Allowed topics: /link/rx, /clock, and internal ROS infrastructure topics
        allowed = {
            "/link/rx",
            "/clock",
            "/parameter_events",
            "/rosout",
        }

        for topic in sub_topics:
            assert topic in allowed, (
                f"H1 VIOLATION: surface_decoder subscribed to forbidden topic: '{topic}'"
            )

        # Verify it DOES subscribe to /link/rx
        assert "/link/rx" in sub_topics, (
            "surface_decoder missing mandatory subscription to /link/rx"
        )
    finally:
        node.destroy_node()


def test_surface_decoder_reference_frames(ros_context: Any):
    """Feed reference frames on /link/rx and verify decoded /surface/contacts."""
    node = SurfaceDecoderNode(parameter_overrides=[])
    test_node = rclpy.create_node("test_surface_decoder_helper")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(test_node)

    received_contacts: list[ContactArray] = []
    _ = test_node.create_subscription(
        ContactArray,
        "/surface/contacts",
        lambda msg: received_contacts.append(msg),
        10,
    )

    import time

    def wait_for_contacts(target_count: int, timeout_sec: float = 2.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if len(received_contacts) >= target_count:
                return True
        return False

    try:
        # Reference vector 1: ref_contact (hex 8146019ffa1e4190)
        # contact_id: 5, class_id: 0, conf: 6/7, x: 12.5, y: -3.0, depth: 15.0, sigma: 1.0, t: 100
        frame1 = LinkFrame()
        frame1.header.stamp.sec = 100
        frame1.payload = list(bytes.fromhex("8146019ffa1e4190"))
        node._rx_callback(frame1)

        assert wait_for_contacts(1), (
            "Timed out waiting for /surface/contacts from frame1"
        )
        contacts1 = received_contacts[-1].contacts
        assert len(contacts1) == 1
        c1 = contacts1[0]
        assert c1.contact_id == 5
        assert c1.class_id == 0
        assert math.isclose(c1.confidence, 6 / 7, abs_tol=1e-3)
        assert math.isclose(c1.position.x, 12.5, abs_tol=1e-3)
        assert math.isclose(c1.position.y, -3.0, abs_tol=1e-3)
        assert math.isclose(c1.position.z, -15.0, abs_tol=1e-3)
        assert math.isclose(c1.depth_m, 15.0, abs_tol=1e-3)
        assert math.isclose(c1.sigma_xy_m, 1.0, abs_tol=1e-3)
        assert c1.sightings == 1

        # Reference vector 2: ref_heartbeat (hex 43200cffd0ca1883)
        # t_s: 100, x: 12.5, y: -3.0, depth: 12.5, heading: 90.0, battery: 0.8, state: 2, pending: 3
        frame2 = LinkFrame()
        frame2.header.stamp.sec = 101
        frame2.payload = list(bytes.fromhex("43200cffd0ca1883"))
        node._rx_callback(frame2)

        executor.spin_once(timeout_sec=0.1)

        # Heartbeat does not alter contact list
        assert len(received_contacts[-1].contacts) == 1
        assert len(node._tracker.track) == 1
        hb = node._tracker.latest_heartbeat
        assert hb is not None
        assert math.isclose(hb.x_m, 12.5, abs_tol=1e-3)
        assert math.isclose(hb.heading_deg, 90.0, abs_tol=1e-3)

        # Reference vector 3: contact_negative_xy (hex 8a8dfe1fd81423e8)
        # contact_id: 42, class_id: 1, conf: 5/7, x: -15.5, y: -20.0, depth: 10.0, sigma: 0.5, t: 250
        frame3 = LinkFrame()
        frame3.header.stamp.sec = 250
        frame3.payload = list(bytes.fromhex("8a8dfe1fd81423e8"))
        node._rx_callback(frame3)

        assert wait_for_contacts(2), (
            "Timed out waiting for /surface/contacts from frame3"
        )

        contacts3 = received_contacts[-1].contacts
        assert len(contacts3) == 2
        # Ordered by contact_id: [5, 42]
        c42 = next(c for c in contacts3 if c.contact_id == 42)
        assert c42.class_id == 1
        assert math.isclose(c42.confidence, 5 / 7, abs_tol=1e-3)
        assert math.isclose(c42.position.x, -15.5, abs_tol=1e-3)
        assert math.isclose(c42.position.y, -20.0, abs_tol=1e-3)
        assert math.isclose(c42.position.z, -10.0, abs_tol=1e-3)
        assert math.isclose(c42.depth_m, 10.0, abs_tol=1e-3)
        assert math.isclose(c42.sigma_xy_m, 0.5, abs_tol=1e-3)
        assert c42.sightings == 1

    finally:
        test_node.destroy_node()
        node.destroy_node()

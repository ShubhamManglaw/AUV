"""Unit tests for pure-Python SurfaceStateTracker."""

import math

from ps11_telemetry import codec
from ps11_telemetry.surface_state import SurfaceStateTracker


def test_surface_state_contact_handling():
    tracker = SurfaceStateTracker(mission_start_s=0.0)

    # Reference contact vector 1
    # contact_id: 5, class_id: 0, conf: ~0.857, x: 12.5, y: -3.0, depth: 15.0, sigma: 1.0, t: 100
    ref_payload = bytes.fromhex("8146019ffa1e4190")

    updated, msg_type = tracker.handle_payload(ref_payload, now_s=100.0)
    assert updated is True
    assert msg_type == "contact"

    contacts = tracker.contacts
    assert len(contacts) == 1
    c = contacts[0]
    assert c.contact_id == 5
    assert c.class_id == 0
    assert math.isclose(c.confidence, 6 / 7, abs_tol=1e-3)
    assert math.isclose(c.x_m, 12.5, abs_tol=1e-3)
    assert math.isclose(c.y_m, -3.0, abs_tol=1e-3)
    assert math.isclose(c.depth_m, 15.0, abs_tol=1e-3)
    assert math.isclose(c.sigma_xy_m, 1.0, abs_tol=1e-3)
    assert math.isclose(c.first_seen_s, 100.0, abs_tol=1e-3)
    assert math.isclose(c.last_seen_s, 100.0, abs_tol=1e-3)
    assert c.sightings == 1

    # Contact update for the same ID (latest report wins)
    updated_contact = codec.ContactReport(
        contact_id=5,
        class_id=0,
        confidence=0.95,
        x_m=13.0,
        y_m=-2.5,
        depth_m=15.5,
        sigma_m=0.5,
        t_s=110,
        is_update=True,
    )
    upd_payload = codec.pack_frame([updated_contact], 8)
    updated, msg_type = tracker.handle_payload(upd_payload, now_s=110.0)
    assert updated is True
    assert msg_type == "contact"

    assert len(tracker.contacts) == 1
    c2 = tracker.get_contact(5)
    assert c2 is not None
    assert math.isclose(c2.x_m, 13.0, abs_tol=1e-3)
    assert math.isclose(c2.y_m, -2.5, abs_tol=1e-3)
    assert math.isclose(c2.depth_m, 15.5, abs_tol=1e-3)
    assert math.isclose(c2.sigma_xy_m, 0.5, abs_tol=1e-3)
    assert math.isclose(c2.first_seen_s, 100.0, abs_tol=1e-3)
    assert math.isclose(c2.last_seen_s, 110.0, abs_tol=1e-3)
    assert c2.sightings == 2

    # Check contact age
    assert math.isclose(tracker.get_contact_age(5, now_s=125.0), 15.0, abs_tol=1e-3)


def test_surface_state_heartbeat_and_track():
    tracker = SurfaceStateTracker(mission_start_s=0.0)

    # Reference heartbeat vector 2
    # t_s: 100, x: 12.5, y: -3.0, depth: 12.5, heading: 90.0, battery: 0.8, state: 2, pending: 3
    hb_payload = bytes.fromhex("43200cffd0ca1883")
    updated, msg_type = tracker.handle_payload(hb_payload, now_s=100.0)
    assert updated is True
    assert msg_type == "heartbeat"

    assert len(tracker.track) == 1
    hb = tracker.latest_heartbeat
    assert hb is not None
    assert math.isclose(hb.t_s, 100.0, abs_tol=1e-3)
    assert math.isclose(hb.x_m, 12.5, abs_tol=1e-3)
    assert math.isclose(hb.y_m, -3.0, abs_tol=1e-3)
    assert math.isclose(hb.depth_m, 12.5, abs_tol=1e-3)
    assert math.isclose(hb.heading_deg, 90.0, abs_tol=1e-3)
    assert hb.state == 2
    assert hb.pending == 3


def test_surface_state_padding_and_extended():
    tracker = SurfaceStateTracker(mission_start_s=0.0)

    # Padding message
    pad_payload = bytes(8)
    updated, msg_type = tracker.handle_payload(pad_payload, now_s=10.0)
    assert updated is False
    assert msg_type == "padding"

    # Extended message (type 3)
    ext_payload = bytes([0xC0, 0, 0, 0, 0, 0, 0, 0])
    updated, msg_type = tracker.handle_payload(ext_payload, now_s=10.0)
    assert updated is False
    assert msg_type == "extended"

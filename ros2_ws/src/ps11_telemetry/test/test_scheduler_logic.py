"""Focused unit tests for the T3.3 semantic scheduler policy (plan §11.7).

The frozen codec is used as-is (encode/decode) — no golden-vector duplication.
Class priorities are loaded from classes.yaml via ps11_common.classes.
"""

from __future__ import annotations

import pytest
from ps11_common.classes import ClassDatabase
from ps11_common.params import load_yaml
from ps11_telemetry.codec import ContactReport, Heartbeat, decode, unpack_frame
from ps11_telemetry.scheduler_logic import (
    ContactInput,
    SemanticPolicy,
    VehicleState,
)

CFG = load_yaml("scheduler.yaml")


def priorities() -> dict[int, float]:
    db = ClassDatabase()
    return {c.id: c.priority for c in db.all_classes()}


def make_policy() -> SemanticPolicy:
    return SemanticPolicy(CFG, priorities(), frame_payload_bytes=8)


def contact(cid=7, cls=0, conf=0.8, x=10.0, y=20.0, depth=12.5, sigma=0.5, t_last=None):
    return ContactInput(cid, cls, conf, x, y, depth, sigma, t_last)


def test_no_contacts_no_contact_packet() -> None:
    policy = make_policy()
    d = policy.on_frame(0.0, VehicleState())
    assert d.kind == "heartbeat"  # initial heartbeat (none ever sent)
    d = policy.on_frame(1.0, VehicleState())
    assert d.payload is None and d.kind is None
    # with no contacts the hb_min floor keeps heartbeats flowing (5 s cadence),
    # but no frame ever contains a CONTACT:
    for t in (5.0, 10.0, 15.0):
        d = policy.on_frame(t, VehicleState())
        assert d.kind == "heartbeat"
        assert d.payload[0] >> 6 == 0b01  # type bits = 01 (HEARTBEAT)
        assert isinstance(decode(d.payload), Heartbeat)


def test_one_contact_yields_valid_8byte_packet() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())  # heartbeat first (never sent)
    policy.update_contacts([contact()], 0.5)
    d = policy.on_frame(0.5, VehicleState())
    assert d.kind == "contact"
    assert isinstance(d.payload, bytes)
    assert len(d.payload) == 8


def test_packet_decodes_with_frozen_codec() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())
    policy.update_contacts(
        [contact(cid=7, cls=0, conf=0.8, x=10.0, y=20.0, depth=12.5)], 0.5
    )
    d = policy.on_frame(0.5, VehicleState())
    msg = decode(d.payload)
    assert isinstance(msg, ContactReport)
    assert msg.contact_id == 7
    assert msg.class_id == 0
    assert msg.is_update is False
    assert msg.x_m == pytest.approx(10.0, abs=0.25)  # 0.5 m quantisation
    assert msg.y_m == pytest.approx(20.0, abs=0.25)
    assert msg.depth_m == pytest.approx(12.5, abs=0.25)
    assert msg.confidence == pytest.approx(round(0.8 * 7) / 7, abs=1e-6)


def test_debris_sent_before_scallop_at_same_confidence() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())  # initial heartbeat
    policy.update_contacts(
        [contact(cid=20, cls=3, conf=0.8), contact(cid=10, cls=0, conf=0.8)], 0.5
    )
    d1 = policy.on_frame(0.5, VehicleState())
    msg1 = decode(d1.payload)
    assert isinstance(msg1, ContactReport) and msg1.contact_id == 10  # debris first
    d2 = policy.on_frame(1.0, VehicleState())
    msg2 = decode(d2.payload)
    assert isinstance(msg2, ContactReport) and msg2.contact_id == 20  # scallop after


def test_unchanged_contact_not_spammed() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())
    policy.update_contacts([contact()], 0.5)
    d1 = policy.on_frame(0.5, VehicleState())
    assert d1.kind == "contact"
    # same contact, unchanged: suppressed; hb_min (5 s) not reached
    d2 = policy.on_frame(1.0, VehicleState())
    d3 = policy.on_frame(4.0, VehicleState())
    assert d2.payload is None and d3.payload is None
    # but the heartbeat floor still applies at >= 5 s since last heartbeat
    d4 = policy.on_frame(5.0, VehicleState())
    assert d4.kind == "heartbeat"


def test_small_position_change_does_not_trigger_update() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())
    policy.update_contacts([contact(x=10.0, y=20.0)], 0.5)
    policy.on_frame(0.5, VehicleState())  # sent
    policy.update_contacts([contact(x=10.4, y=20.0)], 2.0)  # moved 0.4 m < 1.0 m
    d = policy.on_frame(2.0, VehicleState())
    assert d.payload is None  # not eligible; hb_min (5 s) not reached either


def test_contact_t_uses_last_observation_time() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())
    # observed at 7.0 s, processed at 10.0 s (still within the hb_max window)
    policy.update_contacts([contact(t_last=7.0)], 10.0)
    d = policy.on_frame(10.0, VehicleState())
    msg = decode(d.payload)
    assert isinstance(msg, ContactReport)
    assert msg.t_s == 7  # §11.3: t = time of last observation, mod 2048


def test_multislot_frame_fills_heartbeat_then_contacts() -> None:
    policy = SemanticPolicy(CFG, priorities(), frame_payload_bytes=32)  # k = 4
    policy.update_contacts(
        [contact(cid=20, cls=3, conf=0.6), contact(cid=10, cls=0, conf=0.8)], 0.5
    )
    d = policy.on_frame(0.5, VehicleState())
    assert d.kind == "heartbeat"  # hb_max due (nothing sent yet) -> slot 1
    assert len(d.payload) == 32
    msgs = unpack_frame(d.payload)
    assert len(msgs) == 3  # heartbeat + both contacts, no duplicate spam
    assert isinstance(msgs[0], Heartbeat)
    assert [m.contact_id for m in msgs[1:]] == [10, 20]  # scoring order
    assert all(m.is_update is False for m in msgs[1:])
    # everything sent: the next tx_ready with nothing new sends nothing
    d2 = policy.on_frame(1.0, VehicleState())
    assert d2.payload is None


def test_meaningful_update_becomes_eligible_again() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())
    policy.update_contacts([contact(x=10.0, y=20.0)], 0.5)
    policy.on_frame(0.5, VehicleState())  # sent
    policy.update_contacts([contact(x=12.0, y=20.0)], 6.0)  # moved 2 m (>= 1.0)
    d = policy.on_frame(6.0, VehicleState())
    assert d.kind == "contact"
    msg = decode(d.payload)
    assert isinstance(msg, ContactReport)
    assert msg.contact_id == 7
    assert msg.is_update is True


def test_out_of_range_contact_handled_per_codec_rules() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())
    skipped = policy.update_contacts(
        [contact(cid=300), contact(cid=7, cls=9), contact(cid=8, cls=0, conf=0.9)], 0.5
    )
    assert skipped == (7, 300)  # invalid ids recorded, valid one kept
    d = policy.on_frame(0.5, VehicleState())
    assert d.kind == "contact" and d.skipped == ()  # per-frame skips are empty
    msg = decode(d.payload)
    assert isinstance(msg, ContactReport) and msg.contact_id == 8
    # encode() itself would raise for out-of-range ids — the policy never tries
    with pytest.raises(ValueError):
        __import__("ps11_telemetry.codec", fromlist=["encode"]).encode(
            ContactReport(300, 0, 0.5, 0, 0, 0, 0.5, 0, False)
        )


def test_tx_ready_backpressure_no_unbounded_queue() -> None:
    policy = make_policy()
    policy.on_frame(0.0, VehicleState())
    policy.update_contacts([contact()], 0.5)
    # a burst of tx_ready with nothing new: only the first yields the packet
    outputs = [policy.on_frame(t, VehicleState()) for t in (0.5, 0.6, 0.7, 0.8)]
    kinds = [d.kind for d in outputs]
    assert kinds[0] == "contact"
    assert all(k is None for k in kinds[1:])
    # and the policy holds no queue of pending frames
    assert policy.seq == 2  # heartbeat + contact only


def test_deterministic_selection_for_ties() -> None:
    results = []
    for _ in range(3):
        policy = make_policy()
        policy.on_frame(0.0, VehicleState())
        policy.update_contacts(
            [contact(cid=9, cls=0, conf=0.8), contact(cid=4, cls=0, conf=0.8)], 0.5
        )
        d = policy.on_frame(0.5, VehicleState())
        msg = decode(d.payload)
        results.append(msg.contact_id)
    assert results == [4, 4, 4]  # lower contact id wins every time


def test_heartbeat_period_and_fields() -> None:
    policy = make_policy()
    d = policy.on_frame(
        0.0, VehicleState(x_m=5.0, y_m=-7.5, z_m=-12.5, yaw_rad=0.0, mission_state=2)
    )
    assert d.kind == "heartbeat"  # never sent -> hb_max due immediately
    msg = decode(d.payload)
    assert isinstance(msg, Heartbeat)
    assert msg.x_m == pytest.approx(5.0, abs=0.25)
    assert msg.y_m == pytest.approx(-7.5, abs=0.25)
    assert msg.depth_m == pytest.approx(12.5, abs=0.25)
    assert msg.heading_deg == pytest.approx(90.0, abs=0.1)  # yaw 0 -> north
    assert msg.state == 2
    assert 0.0 < msg.battery_frac <= 1.0
    # next heartbeat must wait for hb_min/hb_max, not fire immediately
    d2 = policy.on_frame(1.0, VehicleState())
    assert d2.kind is None
    d3 = policy.on_frame(15.0, VehicleState())  # >= hb_max (15 s)
    assert d3.kind == "heartbeat"

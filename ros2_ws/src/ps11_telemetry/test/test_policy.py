"""Unit tests for semantic telemetry scheduling policy (§11.7).

Acceptance checks:
- debris before scallop at equal confidence
- heartbeat at least every 15 s
- no update for moves < 1 m
- update for >= 1 m or sigma halved
- age boost
"""

import pytest
from ps11_telemetry.codec import ContactReport, Heartbeat
from ps11_telemetry.policy import CandidateContact, SemanticPolicy, VehicleState


def test_debris_before_scallop_at_equal_confidence():
    """Debris (priority 1.0) must be scheduled before scallop (priority 0.2) at equal confidence."""
    policy = SemanticPolicy()
    veh = VehicleState()

    debris = CandidateContact(
        id=1,
        class_id=0,  # debris (priority 1.0)
        confidence=0.8,
        x_m=10.0,
        y_m=5.0,
        depth_m=12.0,
        sigma_m=0.5,
        last_seen_s=1.0,
    )
    scallop = CandidateContact(
        id=2,
        class_id=3,  # scallop (priority 0.2)
        confidence=0.8,
        x_m=12.0,
        y_m=6.0,
        depth_m=12.0,
        sigma_m=0.5,
        last_seen_s=1.0,
    )

    policy.update_contacts([scallop, debris], now_s=1.0)

    # Frame with 1 slot
    msgs = policy.select_messages_for_frame(k_slots=1, now_s=1.0, vehicle=veh)
    assert len(msgs) == 1
    assert isinstance(msgs[0], ContactReport)
    assert msgs[0].contact_id == 1  # Debris must be chosen first!


def test_heartbeat_at_least_every_15s():
    """Heartbeat must be scheduled if now - last_heartbeat >= 15 s, even with waiting candidates."""
    policy = SemanticPolicy(hb_max_period_s=15.0)
    veh = VehicleState(x_m=5.0, y_m=10.0, depth_m=8.0, heading_deg=45.0, state=2)

    contact = CandidateContact(
        id=1,
        class_id=0,
        confidence=0.9,
        x_m=10.0,
        y_m=5.0,
        depth_m=12.0,
        sigma_m=0.5,
        last_seen_s=0.0,
    )
    policy.update_contacts([contact], now_s=0.0)

    # First frame at t=0s transmits the candidate
    msgs_t0 = policy.select_messages_for_frame(k_slots=1, now_s=0.0, vehicle=veh)
    assert len(msgs_t0) == 1
    assert isinstance(msgs_t0[0], ContactReport)

    # Add another candidate at t=10s
    contact2 = CandidateContact(
        id=2,
        class_id=0,
        confidence=0.9,
        x_m=15.0,
        y_m=5.0,
        depth_m=12.0,
        sigma_m=0.5,
        last_seen_s=10.0,
    )
    policy.update_contacts([contact2], now_s=10.0)

    # At t=15.1s (>= 15s since last heartbeat at t=0s), heartbeat must take slot 0
    msgs_t15 = policy.select_messages_for_frame(k_slots=1, now_s=15.1, vehicle=veh)
    assert len(msgs_t15) == 1
    assert isinstance(msgs_t15[0], Heartbeat)
    assert msgs_t15[0].state == 2
    assert msgs_t15[0].x_m == pytest.approx(5.0)


def test_no_update_for_moves_less_than_1m():
    """Small position changes (< 1.0 m) without sigma drop do not trigger update reports."""
    policy = SemanticPolicy(update_min_move_m=1.0)
    veh = VehicleState()

    c = CandidateContact(
        id=5,
        class_id=1,
        confidence=0.7,
        x_m=10.0,
        y_m=10.0,
        depth_m=5.0,
        sigma_m=1.0,
        last_seen_s=1.0,
    )
    policy.update_contacts([c], now_s=1.0)
    # Send contact
    msgs = policy.select_messages_for_frame(k_slots=1, now_s=1.0, vehicle=veh)
    assert len(msgs) == 1
    assert msgs[0].contact_id == 5

    # Move by 0.6 m (hypotenuse 0.6 < 1.0 m), sigma unchanged
    c_moved_small = CandidateContact(
        id=5,
        class_id=1,
        confidence=0.7,
        x_m=10.4,
        y_m=10.4,
        depth_m=5.0,
        sigma_m=1.0,
        last_seen_s=2.0,
    )
    policy.update_contacts([c_moved_small], now_s=2.0)
    assert 5 not in policy.candidate_queue  # No update queued!


def test_update_for_ge_1m_or_sigma_halved():
    """Moves >= 1.0 m OR sigma halved trigger an update report with is_update=True."""
    policy = SemanticPolicy(update_min_move_m=1.0, update_sigma_ratio=0.5)
    veh = VehicleState()

    # Initial send
    c = CandidateContact(
        id=10,
        class_id=2,
        confidence=0.8,
        x_m=20.0,
        y_m=20.0,
        depth_m=10.0,
        sigma_m=1.0,
        last_seen_s=1.0,
    )
    policy.update_contacts([c], now_s=1.0)
    policy.select_messages_for_frame(k_slots=1, now_s=1.0, vehicle=veh)

    # 1. Test move >= 1.0 m (moved 1.2 m along X)
    c_moved = CandidateContact(
        id=10,
        class_id=2,
        confidence=0.8,
        x_m=21.2,
        y_m=20.0,
        depth_m=10.0,
        sigma_m=1.0,
        last_seen_s=5.0,
    )
    policy.update_contacts([c_moved], now_s=5.0)
    assert 10 in policy.candidate_queue
    msgs = policy.select_messages_for_frame(k_slots=1, now_s=5.0, vehicle=veh)
    assert len(msgs) == 1
    assert isinstance(msgs[0], ContactReport)
    assert msgs[0].contact_id == 10
    assert msgs[0].is_update is True
    assert msgs[0].x_m == pytest.approx(21.2, abs=0.5)

    # 2. Test sigma halved (moved 0.0 m, but sigma goes from 1.0 to 0.4 <= 0.5 * 1.0)
    c_sigma_halved = CandidateContact(
        id=10,
        class_id=2,
        confidence=0.8,
        x_m=21.2,
        y_m=20.0,
        depth_m=10.0,
        sigma_m=0.4,
        last_seen_s=10.0,
    )
    policy.update_contacts([c_sigma_halved], now_s=10.0)
    assert 10 in policy.candidate_queue
    msgs_sigma = policy.select_messages_for_frame(k_slots=1, now_s=10.0, vehicle=veh)
    assert len(msgs_sigma) == 1
    assert isinstance(msgs_sigma[0], ContactReport)
    assert msgs_sigma[0].is_update is True
    assert msgs_sigma[0].sigma_m <= 0.5


def test_age_boost():
    """Score increases as waiting time increases via age boost factor."""
    policy = SemanticPolicy(age_boost_tau_s=30.0, age_boost_max_s=60.0)

    # Scallop priority 0.2, starfish priority 0.3
    # Candidate A (scallop) created at t=0s
    cand_scallop = CandidateContact(
        id=1,
        class_id=3,  # priority 0.2
        confidence=0.8,
        x_m=5.0,
        y_m=5.0,
        depth_m=10.0,
        sigma_m=0.5,
        last_seen_s=0.0,
    )
    # At t=0s, score is 0.2 * 0.8 * 1.0 = 0.16
    score_fresh = policy.compute_candidate_score(
        cand_scallop, become_candidate_time_s=0.0, is_update=False, now_s=0.0
    )
    assert score_fresh == pytest.approx(0.16)

    # At t=30s, age=30s, age_boost = 1 + 30/30 = 2.0 -> score = 0.16 * 2.0 = 0.32
    score_aged = policy.compute_candidate_score(
        cand_scallop, become_candidate_time_s=0.0, is_update=False, now_s=30.0
    )
    assert score_aged == pytest.approx(0.32)
    assert score_aged > score_fresh

    # Verify that with age boost, aged scallop (0.32) beats fresh starfish (0.3 * 0.8 * 1.0 = 0.24)
    cand_starfish = CandidateContact(
        id=2,
        class_id=1,  # priority 0.3
        confidence=0.8,
        x_m=6.0,
        y_m=6.0,
        depth_m=10.0,
        sigma_m=0.5,
        last_seen_s=30.0,
    )
    score_starfish_fresh = policy.compute_candidate_score(
        cand_starfish, become_candidate_time_s=30.0, is_update=False, now_s=30.0
    )
    assert score_starfish_fresh == pytest.approx(0.24)
    assert score_aged > score_starfish_fresh

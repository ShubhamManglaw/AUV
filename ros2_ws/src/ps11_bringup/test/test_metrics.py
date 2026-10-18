"""Unit tests for metrics and matching logic (§12.1–12.2, Q5).

Acceptance checks:
- matching logic: same class within 3.0 m
- different class rejected even if < 3.0 m
- same class rejected if > 3.0 m
- calculation of counters (bits, ratio, airtime, recall, error, latency)
"""

import pytest
from ps11_bringup.metrics import (
    ContactObservation,
    GroundTruthObject,
    calculate_counters,
    match_contacts_to_gt,
)


def test_matching_same_class_within_3m():
    """Matching accepts same class within 3.0 m distance."""
    gt_objects = [
        GroundTruthObject(id=1, name="debris_1", class_id=0, x=10.0, y=10.0, z=-15.0),
        GroundTruthObject(id=2, name="starfish_1", class_id=1, x=20.0, y=20.0, z=-15.0),
    ]

    # Contact 1: debris 1.5 m away from GT 1
    # Contact 2: starfish 2.5 m away from GT 2
    contacts = [
        ContactObservation(
            contact_id=101,
            class_id=0,
            x=11.0,
            y=11.1,
            z=-15.0,
            confidence=0.85,
            first_seen_s=1.0,
            last_seen_s=3.0,
        ),
        ContactObservation(
            contact_id=102,
            class_id=1,
            x=21.5,
            y=21.5,
            z=-15.0,
            confidence=0.75,
            first_seen_s=2.0,
            last_seen_s=5.0,
        ),
    ]

    matches, unmatched_gt, unmatched_c = match_contacts_to_gt(
        gt_objects, contacts, max_distance_m=3.0
    )

    assert len(matches) == 2
    assert len(unmatched_gt) == 0
    assert len(unmatched_c) == 0
    assert matches[0].gt.id == 1
    assert matches[0].contact.contact_id == 101
    assert matches[1].gt.id == 2
    assert matches[1].contact.contact_id == 102


def test_matching_rejects_different_class_and_out_of_range():
    """Matching rejects contacts with different class or distance > 3.0 m."""
    gt_objects = [
        GroundTruthObject(id=1, name="debris_1", class_id=0, x=10.0, y=10.0, z=-15.0),
    ]

    # Contact A: class 1 (starfish) at distance 0.5 m (different class!)
    # Contact B: class 0 (debris) at distance 3.5 m (> 3.0 m!)
    contacts = [
        ContactObservation(
            contact_id=201,
            class_id=1,
            x=10.3,
            y=10.4,
            z=-15.0,
            confidence=0.9,
            first_seen_s=1.0,
            last_seen_s=2.0,
        ),
        ContactObservation(
            contact_id=202,
            class_id=0,
            x=13.5,
            y=10.0,
            z=-15.0,
            confidence=0.9,
            first_seen_s=1.0,
            last_seen_s=2.0,
        ),
    ]

    matches, unmatched_gt, unmatched_c = match_contacts_to_gt(
        gt_objects, contacts, max_distance_m=3.0
    )

    assert len(matches) == 0
    assert len(unmatched_gt) == 1
    assert len(unmatched_c) == 2


def test_evaluation_counters_calculation():
    """Counters accurately compute bits, compression ratio, airtime, recall, and errors."""
    gt_objects = [
        GroundTruthObject(id=1, name="debris_1", class_id=0, x=0.0, y=0.0, z=-15.0),
        GroundTruthObject(id=2, name="debris_2", class_id=0, x=50.0, y=50.0, z=-15.0),
        GroundTruthObject(id=3, name="starfish_1", class_id=1, x=20.0, y=20.0, z=-15.0),
        GroundTruthObject(id=4, name="scallop_1", class_id=3, x=30.0, y=30.0, z=-15.0),
    ]

    # Surface reports: debris 1 reported (error 1.0 m, latency 4.0 s), starfish 1 reported (error 2.0 m, latency 6.0 s)
    surface_contacts = [
        ContactObservation(
            contact_id=1,
            class_id=0,
            x=1.0,
            y=0.0,
            z=-15.0,
            confidence=0.8,
            first_seen_s=2.0,
            last_seen_s=6.0,
        ),
        ContactObservation(
            contact_id=2,
            class_id=1,
            x=20.0,
            y=22.0,
            z=-15.0,
            confidence=0.7,
            first_seen_s=5.0,
            last_seen_s=11.0,
        ),
    ]

    counters = calculate_counters(
        semantic_bits_sent=1280,
        jpeg_equiv_bits=1280000,
        link_bitrate_bps=64,
        contacts_onboard_count=3,
        surface_contacts=surface_contacts,
        gt_objects=gt_objects,
        first_view_times={1: 2.0, 3: 5.0},
        max_distance_m=3.0,
    )

    assert counters.semantic_bits_sent == 1280
    assert counters.jpeg_equiv_bits == 1280000
    assert counters.ratio_vs_jpeg == pytest.approx(1000.0)
    assert counters.jpeg_airtime_at_link_s == pytest.approx(20000.0)
    assert counters.contacts_onboard == 3
    assert counters.contacts_at_surface == 2
    assert counters.gt_objects_total == 4
    assert counters.gt_objects_reported == 2
    assert counters.surface_recall == pytest.approx(0.5)
    # Errors: 1.0 and 2.0 -> mean 1.5 m
    assert counters.mean_position_error_m == pytest.approx(1.5)
    # Latencies: 6-2 = 4s and 11-5 = 6s -> mean 5.0 s
    assert counters.mean_first_report_latency_s == pytest.approx(5.0)
    assert counters.correct_contacts_at_surface == 2
    assert counters.false_contacts_at_surface == 0


def test_false_contacts_at_surface():
    """Unmatched surface contacts are accurately counted as false contacts."""
    gt_objects = [
        GroundTruthObject(id=1, name="debris_1", class_id=0, x=0.0, y=0.0, z=-15.0),
    ]

    # Contact 1: matched (within 1m)
    # Contact 2: false contact (same class but 20m away)
    # Contact 3: false contact (wrong class, no GT match)
    surface_contacts = [
        ContactObservation(
            contact_id=1,
            class_id=0,
            x=0.5,
            y=0.0,
            z=-15.0,
            confidence=0.8,
            first_seen_s=2.0,
            last_seen_s=6.0,
            surface_arrival_s=6.5,
        ),
        ContactObservation(
            contact_id=2,
            class_id=0,
            x=20.0,
            y=20.0,
            z=-15.0,
            confidence=0.6,
            first_seen_s=10.0,
            last_seen_s=12.0,
            surface_arrival_s=13.0,
        ),
        ContactObservation(
            contact_id=3,
            class_id=2,
            x=0.0,
            y=0.0,
            z=-15.0,
            confidence=0.5,
            first_seen_s=15.0,
            last_seen_s=16.0,
            surface_arrival_s=17.0,
        ),
    ]

    counters = calculate_counters(
        semantic_bits_sent=640,
        jpeg_equiv_bits=100000,
        link_bitrate_bps=64,
        contacts_onboard_count=3,
        surface_contacts=surface_contacts,
        gt_objects=gt_objects,
        first_view_times={1: 1.0},
        max_distance_m=3.0,
    )

    assert counters.contacts_at_surface == 3
    assert counters.correct_contacts_at_surface == 1
    assert counters.false_contacts_at_surface == 2
    # Latency: arrival 6.5 - first_seen 2.0 = 4.5s
    assert counters.mean_first_report_latency_s == pytest.approx(4.5)
    # Old latency: arrival 6.5 - first_view 1.0 = 5.5s
    assert counters.mean_time_since_mission_start_s == pytest.approx(5.5)


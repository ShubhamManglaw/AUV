"""Unit tests for ContactDatabase and observation fusion (T2.8 / Q3).

Acceptance checks (§10.8):
1. Same track updates the same contact.
2. New track of the same class within the gate joins it.
3. Inverse-variance position fusion.
4. Fused sigma never drops below the 0.3 m floor.
"""

import math

from ps11_perception.fusion import ContactDatabase


def test_same_track_updates_same_contact() -> None:
    """Observations with the same track ID always update the same contact."""
    db = ContactDatabase(gate_m=2.0, sigma_floor_m=0.3)

    c1 = db.update(
        track_id=10,
        class_id=0,
        confidence=0.7,
        x=10.0,
        y=5.0,
        z=-15.0,
        sigma_xy_m=0.8,
        stamp=100.0,
    )
    assert c1.contact_id == 0
    assert c1.sightings == 1

    # Second observation with track_id=10 (even if moved)
    c2 = db.update(
        track_id=10,
        class_id=0,
        confidence=0.85,
        x=10.2,
        y=5.1,
        z=-15.0,
        sigma_xy_m=0.8,
        stamp=100.5,
    )
    assert c2.contact_id == 0
    assert c2.sightings == 2
    assert c2.confidence == 0.85
    assert len(db.contacts) == 1


def test_new_track_within_gate_joins_contact() -> None:
    """New track of the same class within max(gate, 3*sigma) joins the contact."""
    db = ContactDatabase(gate_m=2.0, sigma_floor_m=0.3)

    # First observation establishes Contact 0 at (10, 10)
    c1 = db.update(
        track_id=1,
        class_id=2,  # sea_urchin
        confidence=0.75,
        x=10.0,
        y=10.0,
        z=-15.0,
        sigma_xy_m=0.5,
        stamp=1.0,
    )
    assert c1.contact_id == 0

    # Second observation with NEW track_id=2, same class, 0.8 m away (0.8 m < 2.0 m gate)
    c2 = db.update(
        track_id=2,
        class_id=2,
        confidence=0.80,
        x=10.5,
        y=10.6,
        z=-15.0,
        sigma_xy_m=0.5,
        stamp=2.0,
    )
    assert c2.contact_id == 0, "New track within gate should join existing contact"
    assert c2.sightings == 2
    assert 2 in c2.associated_tracks

    # Third observation with NEW track_id=3, same class, but 5.0 m away (outside gate)
    c3 = db.update(
        track_id=3,
        class_id=2,
        confidence=0.70,
        x=15.0,
        y=10.0,
        z=-15.0,
        sigma_xy_m=0.5,
        stamp=3.0,
    )
    assert c3.contact_id == 1, "New track outside gate should create new contact"
    assert len(db.contacts) == 2

    # Fourth observation with NEW track_id=4, DIFFERENT class (class 1), at (10.1, 10.1)
    c4 = db.update(
        track_id=4,
        class_id=1,  # starfish
        confidence=0.90,
        x=10.1,
        y=10.1,
        z=-15.0,
        sigma_xy_m=0.5,
        stamp=4.0,
    )
    assert c4.contact_id == 2, "Different class within gate must not join"
    assert len(db.contacts) == 3


def test_inverse_variance_fusion() -> None:
    """Position updates use exact inverse-variance weighting."""
    db = ContactDatabase(gate_m=2.0, sigma_floor_m=0.1)

    # Observation 1: x=10.0, sigma=1.0 -> w1 = 1 / 1.0^2 = 1.0
    db.update(
        track_id=1,
        class_id=0,
        confidence=0.5,
        x=10.0,
        y=0.0,
        z=-15.0,
        sigma_xy_m=1.0,
        stamp=1.0,
    )

    # Observation 2: x=20.0, sigma=0.5 -> w2 = 1 / 0.5^2 = 4.0
    c = db.update(
        track_id=1,
        class_id=0,
        confidence=0.6,
        x=20.0,
        y=0.0,
        z=-15.0,
        sigma_xy_m=0.5,
        stamp=2.0,
    )

    # Fused x = (1.0 * 10.0 + 4.0 * 20.0) / (1.0 + 4.0) = 90.0 / 5.0 = 18.0
    assert math.isclose(c.x, 18.0, rel_tol=1e-5), f"Expected fused x=18.0, got {c.x}"


def test_sigma_never_below_floor() -> None:
    """Fused sigma is clamped to sigma_floor_m (0.3 m) regardless of observation count."""
    db = ContactDatabase(gate_m=2.0, sigma_floor_m=0.3)

    # Feed 100 observations with sigma=0.5 m
    for i in range(100):
        c = db.update(
            track_id=1,
            class_id=0,
            confidence=0.8,
            x=5.0,
            y=5.0,
            z=-15.0,
            sigma_xy_m=0.5,
            stamp=float(i),
        )

    # Raw 1/sqrt(100 * 4) = 1/20 = 0.05 m < 0.3 m
    assert c.sigma_xy_m == 0.3, (
        f"Expected sigma clamped to floor 0.3, got {c.sigma_xy_m}"
    )

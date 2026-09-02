"""Unit tests for lawnmower waypoint generator (§9.1, T1.5)."""

import math
from ps11_nav.lawnmower import generate_lawnmower_waypoints, point_to_segment_distance


def test_lawnmower_generation():
    waypoints = generate_lawnmower_waypoints(
        origin_x=5.0,
        origin_y=-10.0,
        area_x=40.0,
        area_y=20.0,
        leg_spacing=4.0,
        altitude=2.5,
        seabed_z=-15.0,
    )

    # 20m area_y with 4m spacing = 6 legs, 2 waypoints per leg = 12 waypoints
    assert len(waypoints) == 12

    # Check Z matches seabed_z + altitude = -12.5
    for wp in waypoints:
        assert math.isclose(wp.z, -12.5, abs_tol=1e-5)

    # Leg 0 (even): starts at (5, -10), ends at (45, -10)
    assert math.isclose(waypoints[0].x, 5.0)
    assert math.isclose(waypoints[0].y, -10.0)
    assert math.isclose(waypoints[1].x, 45.0)
    assert math.isclose(waypoints[1].y, -10.0)

    # Leg 1 (odd): starts at (45, -6), ends at (5, -6)
    assert math.isclose(waypoints[2].x, 45.0)
    assert math.isclose(waypoints[2].y, -6.0)
    assert math.isclose(waypoints[3].x, 5.0)
    assert math.isclose(waypoints[3].y, -6.0)


def test_point_to_segment_distance():
    # Segment from (0, 0) to (10, 0)
    # Point at (5, 2) -> distance should be 2.0
    d = point_to_segment_distance(5.0, 2.0, 0.0, 0.0, 10.0, 0.0)
    assert math.isclose(d, 2.0, abs_tol=1e-6)

    # Point beyond segment end at (12, 0) -> distance should be 2.0
    d = point_to_segment_distance(12.0, 0.0, 0.0, 0.0, 10.0, 0.0)
    assert math.isclose(d, 2.0, abs_tol=1e-6)

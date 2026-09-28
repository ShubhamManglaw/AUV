"""Lawnmower waypoint generation and geometric utilities (§9.1, T1.5)."""

from __future__ import annotations

import math
from typing import NamedTuple


class Waypoint(NamedTuple):
    x: float
    y: float
    z: float


def generate_lawnmower_waypoints(
    origin_x: float,
    origin_y: float,
    area_x: float,
    area_y: float,
    leg_spacing: float,
    altitude: float = 2.5,
    seabed_z: float = -15.0,
) -> list[Waypoint]:
    """Generate alternating lawnmower waypoints in the map frame.

    The survey area runs from origin_x to origin_x + area_x in X,
    and origin_y to origin_y + area_y in Y.
    Legs run parallel to the X axis, stepping across Y by leg_spacing.
    """
    target_z = seabed_z + altitude

    # Number of legs needed to cover area_y
    n_legs = max(2, int(math.ceil(area_y / leg_spacing)) + 1)
    waypoints: list[Waypoint] = []

    for i in range(n_legs):
        y = origin_y + i * leg_spacing
        if i % 2 == 0:
            # Even leg: fly in +X direction
            x_start = origin_x
            x_end = origin_x + area_x
        else:
            # Odd leg: fly in -X direction
            x_start = origin_x + area_x
            x_end = origin_x

        waypoints.append(Waypoint(x_start, y, target_z))
        waypoints.append(Waypoint(x_end, y, target_z))

    return waypoints


def point_to_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    """Calculate perpendicular distance from point (px, py) to line segment (ax, ay)-(bx, by)."""
    dx = bx - ax
    dy = by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(px - ax, py - ay)

    # Project point onto segment: t = dot(p - a, b - a) / |b - a|^2
    t = ((px - ax) * dx + (py - ay) * dy) / l2
    t = max(0.0, min(1.0, t))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)

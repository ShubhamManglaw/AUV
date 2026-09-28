"""Unit tests for geolocator pure logic (T2.7 / Q2).

Acceptance checks (§10.7):
1. Camera 2.5 m above a flat seabed pitched 45°: image centre hits 2.5 m ahead.
2. Upward / horizontal rays are discarded.
3. Rays beyond max_range_m are discarded.
4. First-order uncertainty formula verification.
"""

import math

import numpy as np
from ps11_perception.geo import project_pixel_to_seabed


def test_camera_pitched_45_hits_ahead() -> None:
    """Camera 2.5 m above flat seabed pitched 45° down: image centre hits 2.5 m ahead."""
    # Camera intrinsics (640x480, principal point at 320, 240)
    fx = 320.0
    fy = 320.0
    cx = 320.0
    cy = 240.0
    k_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    camera_pos = np.array([0.0, 0.0, 0.0])  # Camera at origin
    altitude = 2.5  # 2.5 m above seabed

    # Rotation matrix: camera pointing +X in map, pitched 45 deg down
    # Optical frame Z (forward) -> map [1/sqrt(2), 0, -1/sqrt(2)]
    # Optical frame X (right)   -> map [0, -1, 0]
    # Optical frame Y (down)    -> map [1/sqrt(2), 0, 1/sqrt(2)]
    s2 = 1.0 / math.sqrt(2.0)
    r_mat = np.array(
        [
            [0.0, s2, s2],
            [1.0, 0.0, 0.0],
            [0.0, s2, -s2],
        ]
    )

    # Image centre pixel: u=cx, v=cy
    result = project_pixel_to_seabed(
        u=320.0,
        v=240.0,
        k_matrix=k_matrix,
        camera_pos_map=camera_pos,
        camera_rot_map=r_mat,
        altitude_m=altitude,
        max_range_m=12.0,
        sigma_nav_m=0.2,
        sigma_px=8.0,
        sigma_alt_m=0.1,
    )

    assert result is not None, "Expected valid seabed intersection"
    # Expected intersection: X = 2.5 m ahead, Y = 0.0, Z = -2.5 m
    assert math.isclose(result.x, 2.5, abs_tol=1e-3), f"Expected x=2.5, got {result.x}"
    assert math.isclose(result.y, 0.0, abs_tol=1e-3), f"Expected y=0.0, got {result.y}"
    assert math.isclose(result.z, -2.5, abs_tol=1e-3), (
        f"Expected z=-2.5, got {result.z}"
    )
    assert math.isclose(result.depth_m, 2.5, abs_tol=1e-3)

    # Slant range s = 2.5 * sqrt(2) ≈ 3.5355 m
    expected_range = 2.5 * math.sqrt(2.0)
    assert math.isclose(result.range_m, expected_range, abs_tol=1e-3)


def test_upward_rays_discarded() -> None:
    """Rays pointing horizontally or upward (dz >= -1e-3) are discarded."""
    fx, fy, cx, cy = 320.0, 320.0, 320.0, 240.0
    k_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    camera_pos = np.array([0.0, 0.0, -10.0])

    # Camera looking straight horizontal (Z_opt -> +X_map, dz=0)
    r_horizontal = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
    )

    result = project_pixel_to_seabed(
        u=cx,
        v=cy,
        k_matrix=k_matrix,
        camera_pos_map=camera_pos,
        camera_rot_map=r_horizontal,
        altitude_m=2.5,
    )
    assert result is None, "Horizontal ray should be discarded"

    # Camera looking upward (dz > 0)
    s2 = 1.0 / math.sqrt(2.0)
    r_upward = np.array(
        [
            [0.0, -s2, s2],
            [-1.0, 0.0, 0.0],
            [0.0, -s2, s2],
        ]
    )
    result_up = project_pixel_to_seabed(
        u=cx,
        v=cy,
        k_matrix=k_matrix,
        camera_pos_map=camera_pos,
        camera_rot_map=r_upward,
        altitude_m=2.5,
    )
    assert result_up is None, "Upward ray should be discarded"


def test_beyond_max_range_discarded() -> None:
    """Intersections farther than max_range_m are discarded."""
    fx, fy, cx, cy = 320.0, 320.0, 320.0, 240.0
    k_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    camera_pos = np.array([0.0, 0.0, 0.0])

    # Very shallow pitch: dz = -0.1, altitude = 2.0 -> range = 2.0 / 0.1 = 20 m > 12 m
    r_shallow = np.array(
        [
            [0.0, 0.1, 0.994987],
            [-1.0, 0.0, 0.0],
            [0.0, 0.994987, -0.1],
        ]
    )

    result = project_pixel_to_seabed(
        u=cx,
        v=cy,
        k_matrix=k_matrix,
        camera_pos_map=camera_pos,
        camera_rot_map=r_shallow,
        altitude_m=2.0,
        max_range_m=12.0,
    )
    assert result is None, "Ray exceeding max_range_m should be discarded"


def test_sigma_formula() -> None:
    """Uncertainty matches the exact first-order formula."""
    fx, fy, cx, cy = 320.0, 320.0, 320.0, 240.0
    k_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    camera_pos = np.array([0.0, 0.0, 0.0])
    altitude = 2.0

    # Looking straight down: Z_opt -> -Z_map (d = [0, 0, -1])
    r_down = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
    )

    sigma_nav = 0.25
    sigma_px = 8.0
    sigma_alt = 0.15

    result = project_pixel_to_seabed(
        u=cx,
        v=cy,
        k_matrix=k_matrix,
        camera_pos_map=camera_pos,
        camera_rot_map=r_down,
        altitude_m=altitude,
        sigma_nav_m=sigma_nav,
        sigma_px=sigma_px,
        sigma_alt_m=sigma_alt,
    )
    assert result is not None

    # For straight down: s = 2.0, d_xy = 0, |dz| = 1.0
    # term_nav = 0.25^2 = 0.0625
    # term_px  = (2.0 * 8.0 / 320.0)^2 = (16 / 320)^2 = 0.05^2 = 0.0025
    # term_alt = (0.15 * 0 / 1.0)^2 = 0
    # sigma_xy = sqrt(0.0625 + 0.0025) = sqrt(0.065) ≈ 0.25495 m
    expected_sigma = math.sqrt(0.25**2 + 0.05**2 + 0.0)
    assert math.isclose(result.sigma_xy_m, expected_sigma, rel_tol=1e-5)

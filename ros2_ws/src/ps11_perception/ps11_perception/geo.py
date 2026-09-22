"""Geolocator geometry and projection module (ROS-free logic) (§10.7).

Projects pixel coordinates to world coordinates on the seabed:
1. Pixel ray in optical frame: r = K⁻¹ [u, v, 1]ᵀ, normalised.
2. Rotate and translate to map: origin t, direction d = R · r.
3. Local seabed plane: z_s = t_z - altitude.
4. Downward ray check: d_z < -1e-3.
5. Ray-plane intersection: s = (z_s - t_z) / d_z.
6. Range check: s <= max_range_m.
7. Uncertainty: σ_xy = sqrt(σ_nav² + (s·σ_px / f_x)² + (σ_alt · |d_xy| / |d_z|)²).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GeolocationResult:
    """Result of pixel ray intersection with the seabed."""

    x: float
    y: float
    z: float
    depth_m: float
    range_m: float
    sigma_xy_m: float


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert a quaternion (x, y, z, w) into a 3x3 orthonormal rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        return np.eye(3, dtype=float)
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def project_pixel_to_seabed(
    u: float,
    v: float,
    k_matrix: np.ndarray,
    camera_pos_map: np.ndarray,
    camera_rot_map: np.ndarray,
    altitude_m: float,
    max_range_m: float = 12.0,
    sigma_nav_m: float = 0.2,
    sigma_px: float = 8.0,
    sigma_alt_m: float = 0.1,
) -> GeolocationResult | None:
    """Project a pixel coordinate (u, v) to the seabed plane in map frame.

    Args:
        u: Horizontal pixel coordinate (px).
        v: Vertical pixel coordinate (px).
        k_matrix: 3x3 camera intrinsic matrix [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
        camera_pos_map: 3-element vector [tx, ty, tz] camera position in map.
        camera_rot_map: 3x3 rotation matrix from optical frame to map frame.
        altitude_m: Distance from camera/vehicle to seabed (m), must be > 0.
        max_range_m: Maximum allowable ray distance (m).
        sigma_nav_m: Navigation position standard deviation (m).
        sigma_px: Pixel uncertainty standard deviation (px).
        sigma_alt_m: Altimeter standard deviation (m).

    Returns:
        GeolocationResult if intersection is valid and downward, else None.
    """
    if altitude_m <= 0.0:
        return None

    fx = float(k_matrix[0, 0])
    fy = float(k_matrix[1, 1])
    cx = float(k_matrix[0, 2])
    cy = float(k_matrix[1, 2])

    if fx <= 0.0 or fy <= 0.0:
        return None

    # 1. Ray in optical frame: r = K⁻¹ [u, v, 1]ᵀ, normalised
    rx = (u - cx) / fx
    ry = (v - cy) / fy
    rz = 1.0
    norm_r = math.sqrt(rx * rx + ry * ry + rz * rz)
    r_opt = np.array([rx / norm_r, ry / norm_r, rz / norm_r], dtype=float)

    # 2. Rotate to map frame: direction d = R · r
    d = camera_rot_map @ r_opt
    dx, dy, dz = float(d[0]), float(d[1]), float(d[2])

    # 4. If d_z >= -1e-3, discard (ray doesn't point down)
    if dz >= -1e-3:
        return None

    # 3. Local seabed plane: z_s = t_z - altitude
    tx, ty, tz = (
        float(camera_pos_map[0]),
        float(camera_pos_map[1]),
        float(camera_pos_map[2]),
    )
    z_s = tz - altitude_m

    # Ray intersection: s = (z_s - t_z) / d_z = -altitude_m / d_z
    s = (z_s - tz) / dz
    if s < 0.0 or s > max_range_m:
        return None

    # 5. Position p = t + s · d
    px = tx + s * dx
    py = ty + s * dy
    pz = z_s  # tz + s * dz
    depth_m = -pz

    # 6. Uncertainty (first-order per §10.7):
    # σ_xy = sqrt(σ_nav² + (s·σ_px/f_x)² + (σ_alt·|d_xy|/|d_z|)²)
    d_xy = math.sqrt(dx * dx + dy * dy)
    abs_dz = abs(dz)

    term_nav = sigma_nav_m**2
    term_px = ((s * sigma_px) / fx) ** 2
    term_alt = ((sigma_alt_m * d_xy) / abs_dz) ** 2

    sigma_xy = math.sqrt(term_nav + term_px + term_alt)

    return GeolocationResult(
        x=px,
        y=py,
        z=pz,
        depth_m=depth_m,
        range_m=s,
        sigma_xy_m=sigma_xy,
    )

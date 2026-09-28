"""Navigation sensor error models (T1.3, plan §9.2) — ROS-free.

Conventions fixed for T1.3 (agreed with the human):

- The horizontal position error is an independent random walk per axis in the
  map frame. The per-axis standard deviation after D metres travelled is
  sigma(D) = sqrt(initial_pos_var + (position_drift_rate * D)^2).
- Feeding a ground-truth displacement of arc length d at distance D consumes
  variance sigma(D+d)^2 - sigma(D)^2 = rate^2 * ((D+d)^2 - D^2), so the total
  error variance after any path depends only on the path length travelled.
- The heading error is the constant heading_bias_rad (a deterministic bias,
  not a random process); its reported variance is the constant
  initial_heading_var.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ErrorState:
    """Snapshot of the accumulated navigation error."""

    ex_m: float
    ey_m: float
    position_variance: float
    distance_travelled_m: float


class OdomErrorModel:
    """Horizontal position random walk plus a constant heading bias."""

    def __init__(
        self,
        position_drift_rate: float,
        heading_bias_rad: float,
        initial_pos_var: float,
        initial_heading_var: float,
        seed: int,
    ) -> None:
        if position_drift_rate < 0.0:
            raise ValueError("position_drift_rate must be >= 0")
        if initial_pos_var < 0.0:
            raise ValueError("initial_pos_var must be >= 0")
        if initial_heading_var < 0.0:
            raise ValueError("initial_heading_var must be >= 0")
        self._drift_rate = float(position_drift_rate)
        self._heading_bias_rad = float(heading_bias_rad)
        self._initial_pos_var = float(initial_pos_var)
        self._initial_heading_var = float(initial_heading_var)
        self._rng = np.random.default_rng(seed)
        self._ex_m = 0.0
        self._ey_m = 0.0
        self._distance_m = 0.0

    @property
    def distance_travelled_m(self) -> float:
        return self._distance_m

    @property
    def heading_bias_rad(self) -> float:
        return self._heading_bias_rad

    def position_error(self) -> tuple[float, float]:
        """Current horizontal error offset (ex, ey) in the map frame."""
        return self._ex_m, self._ey_m

    def position_variance(self) -> float:
        """Per-axis horizontal variance of the error at the distance travelled."""
        return self._initial_pos_var + (self._drift_rate * self._distance_m) ** 2

    def heading_variance(self) -> float:
        return self._initial_heading_var

    def state(self) -> ErrorState:
        return ErrorState(
            ex_m=self._ex_m,
            ey_m=self._ey_m,
            position_variance=self.position_variance(),
            distance_travelled_m=self._distance_m,
        )

    def step(self, dx_m: float, dy_m: float) -> None:
        """Feed the ground-truth displacement since the previous step."""
        d_m = math.hypot(dx_m, dy_m)
        if d_m <= 0.0:
            return
        var_before = self.position_variance()
        self._distance_m += d_m
        var_after = self.position_variance()
        step_sigma = math.sqrt(var_after - var_before)
        self._ex_m += float(self._rng.normal(0.0, step_sigma))
        self._ey_m += float(self._rng.normal(0.0, step_sigma))

    def noisy_pose(
        self, x_m: float, y_m: float, yaw_rad: float
    ) -> tuple[float, float, float]:
        """Apply the accumulated error and the heading bias to a true pose."""
        return x_m + self._ex_m, y_m + self._ey_m, yaw_rad + self._heading_bias_rad


def depth_noise_sample(rng: np.random.Generator, sigma_m: float) -> float:
    """One depth-sensor noise sample in metres (plan §9.2)."""
    return float(rng.normal(0.0, sigma_m))


def quaternion_about_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a planar rotation about +Z."""
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw about +Z from a quaternion, in (-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

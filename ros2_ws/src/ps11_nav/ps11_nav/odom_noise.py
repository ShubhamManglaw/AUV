"""Odom error model with random walk position drift and heading bias (§9.2, T1.3)."""

from __future__ import annotations

import math
import numpy as np


class OdomNoiseModel:
    """Calculates noisy odometry from ground truth pose.

    Error model:
    - Horizontal position random walk std grows at position_drift_rate of distance travelled.
    - Constant heading bias of heading_bias_rad.
    - Full 6x6 covariance matrix populated with current modelled variance.
    """

    def __init__(
        self,
        position_drift_rate: float = 0.005,
        heading_bias_rad: float = 0.00872665,
        initial_covariance_pos: float = 0.01,
        initial_covariance_heading: float = 0.001,
        seed: int | None = None,
    ) -> None:
        self.drift_rate = float(position_drift_rate)
        self.heading_bias = float(heading_bias_rad)
        self.initial_cov_pos = float(initial_covariance_pos)
        self.initial_cov_heading = float(initial_covariance_heading)
        self.rng = np.random.default_rng(seed)

        self.total_distance = 0.0
        self.last_gt_x: float | None = None
        self.last_gt_y: float | None = None

        # Sample initial position error from initial covariance
        init_std = math.sqrt(self.initial_cov_pos)
        self.pos_noise = self.rng.normal(0.0, init_std, size=2)

    def reset(self, seed: int | None = None) -> None:
        """Reset internal accumulator and random generator."""
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.total_distance = 0.0
        self.last_gt_x = None
        self.last_gt_y = None
        init_std = math.sqrt(self.initial_cov_pos)
        self.pos_noise = self.rng.normal(0.0, init_std, size=2)

    def update(
        self,
        gt_x: float,
        gt_y: float,
        gt_z: float,
        gt_roll: float,
        gt_pitch: float,
        gt_yaw: float,
    ) -> tuple[float, float, float, float, float, float, list[float]]:
        """Update error model with new ground truth pose and return noisy pose + 36-element cov."""
        if self.last_gt_x is not None and self.last_gt_y is not None:
            dx = gt_x - self.last_gt_x
            dy = gt_y - self.last_gt_y
            ds = math.hypot(dx, dy)
            if ds > 0.0:
                s_prev = self.total_distance
                s_curr = s_prev + ds
                self.total_distance = s_curr

                # Incremental variance such that sum of variances = (drift_rate * s_curr)^2
                var_inc = (self.drift_rate**2) * (s_curr**2 - s_prev**2)
                if var_inc > 0.0:
                    step_std = math.sqrt(var_inc)
                    self.pos_noise += self.rng.normal(0.0, step_std, size=2)

        self.last_gt_x = gt_x
        self.last_gt_y = gt_y

        noisy_x = gt_x + float(self.pos_noise[0])
        noisy_y = gt_y + float(self.pos_noise[1])
        noisy_z = gt_z

        noisy_roll = gt_roll
        noisy_pitch = gt_pitch
        noisy_yaw = gt_yaw + self.heading_bias
        # Wrap yaw to [-pi, pi]
        noisy_yaw = (noisy_yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Current modelled covariance
        current_pos_var = (
            self.initial_cov_pos + (self.drift_rate * self.total_distance) ** 2
        )
        cov = [0.0] * 36
        cov[0] = current_pos_var  # x
        cov[7] = current_pos_var  # y
        cov[14] = 0.0004  # z (0.02^2)
        cov[21] = 0.001  # roll
        cov[28] = 0.001  # pitch
        cov[35] = self.initial_cov_heading  # yaw

        return (
            noisy_x,
            noisy_y,
            noisy_z,
            noisy_roll,
            noisy_pitch,
            noisy_yaw,
            cov,
        )

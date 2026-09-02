"""Unit tests for odom_noise error model (§9.2, T1.3)."""

import math
from ps11_nav.odom_noise import OdomNoiseModel


def test_odom_noise_straight_run_100m():
    """Verify horizontal error after a 100 m straight run is of the order of 0.5 m."""
    model = OdomNoiseModel(
        position_drift_rate=0.005,
        heading_bias_rad=0.00872665,
        initial_covariance_pos=0.01,
        initial_covariance_heading=0.001,
        seed=42,
    )

    steps = 2000
    step_length = 0.05  # 100 m total distance
    gt_x = 0.0
    gt_y = 0.0

    # Initial call at origin
    model.update(gt_x=0.0, gt_y=0.0, gt_z=-12.5, gt_roll=0.0, gt_pitch=0.0, gt_yaw=0.0)

    for _ in range(steps):
        gt_x += step_length
        noisy_x, noisy_y, _, _, _, noisy_yaw, cov = model.update(
            gt_x=gt_x,
            gt_y=gt_y,
            gt_z=-12.5,
            gt_roll=0.0,
            gt_pitch=0.0,
            gt_yaw=0.0,
        )

    # Total distance must be 100 m
    assert math.isclose(model.total_distance, 100.0, abs_tol=1e-6)

    # Horizontal position error relative to GT
    dx = noisy_x - gt_x
    dy = noisy_y - gt_y
    horizontal_error = math.hypot(dx, dy)

    # At 100 m with 0.5% drift, expected error is order of 0.5 m
    assert 0.2 < horizontal_error < 1.5, (
        f"Horizontal error {horizontal_error:.3f} m out of expected order of 0.5 m"
    )

    # Heading bias check
    assert math.isclose(noisy_yaw, 0.00872665, abs_tol=1e-6)

    # Covariance check: pos var = 0.01 + (0.005 * 100)^2 = 0.01 + 0.25 = 0.26
    expected_pos_var = 0.01 + (0.005 * 100.0) ** 2
    assert math.isclose(cov[0], expected_pos_var, abs_tol=1e-6)
    assert math.isclose(cov[7], expected_pos_var, abs_tol=1e-6)
    assert math.isclose(cov[35], 0.001, abs_tol=1e-6)

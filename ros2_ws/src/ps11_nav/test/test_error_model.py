"""Unit tests for the navigation sensor error models (T1.3, plan §9.2).

ROS-free tests: no rclpy import, no ROS graph. The fixed-seed acceptance test
prints the realised 100 m horizontal error — that printed value is the
evidence for the "about 0.5 m" acceptance criterion, so run pytest with -s.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from ps11_nav.error_model import (
    OdomErrorModel,
    depth_noise_sample,
    quaternion_about_yaw,
    yaw_from_quaternion,
)

DRIFT_RATE = 0.005
HEADING_BIAS_RAD = 0.00872665
INIT_POS_VAR = 0.01
INIT_HEADING_VAR = 0.001


def _make_model(seed: int) -> OdomErrorModel:
    return OdomErrorModel(
        position_drift_rate=DRIFT_RATE,
        heading_bias_rad=HEADING_BIAS_RAD,
        initial_pos_var=INIT_POS_VAR,
        initial_heading_var=INIT_HEADING_VAR,
        seed=seed,
    )


def test_zero_displacement_leaves_state_unchanged() -> None:
    model = _make_model(seed=7)
    before = model.state()
    model.step(0.0, 0.0)
    assert model.state() == before


def test_horizontal_error_after_100m_is_order_half_meter(
    capsys: pytest.CaptureFixture,
) -> None:
    model = _make_model(seed=42)
    for _ in range(100):
        model.step(1.0, 0.0)
    ex_m, ey_m = model.position_error()
    horizontal_m = math.hypot(ex_m, ey_m)
    print(
        f"100 m fixed-seed horizontal error: {horizontal_m:.4f} m "
        f"(ex={ex_m:.4f} m, ey={ey_m:.4f} m)"
    )
    assert 0.15 <= horizontal_m <= 1.0
    assert model.distance_travelled_m == pytest.approx(100.0)
    assert model.position_variance() == pytest.approx(
        INIT_POS_VAR + (DRIFT_RATE * 100.0) ** 2
    )


def test_same_seed_reproducible_and_different_seed_differs() -> None:
    path = [(1.0, 0.1), (0.8, -0.2), (1.2, 0.0), (0.5, 0.5)] * 25
    first = _make_model(seed=123)
    second = _make_model(seed=123)
    other = _make_model(seed=124)
    for dx_m, dy_m in path:
        first.step(dx_m, dy_m)
        second.step(dx_m, dy_m)
        other.step(dx_m, dy_m)
    assert first.state() == second.state()
    assert first.position_error() != other.position_error()


def test_variance_grows_monotonically_with_distance() -> None:
    model = _make_model(seed=3)
    var_start = model.position_variance()
    model.step(100.0, 0.0)
    var_100 = model.position_variance()
    model.step(100.0, 0.0)
    var_200 = model.position_variance()
    assert var_start < var_100 < var_200
    assert var_100 == pytest.approx(INIT_POS_VAR + (DRIFT_RATE * 100.0) ** 2)
    assert var_200 == pytest.approx(INIT_POS_VAR + (DRIFT_RATE * 200.0) ** 2)


def test_increment_variance_matches_model_formula() -> None:
    rate = DRIFT_RATE
    d_m = 50.0
    step_m = 1.0
    expected_var = (rate**2) * ((d_m + step_m) ** 2 - d_m**2)
    increments_x = []
    increments_y = []
    for seed in range(1000, 1500):
        model = OdomErrorModel(rate, 0.0, 0.0, 0.0, seed=seed)
        model.step(d_m, 0.0)
        ex0, ey0 = model.position_error()
        model.step(step_m, 0.0)
        ex1, ey1 = model.position_error()
        increments_x.append(ex1 - ex0)
        increments_y.append(ey1 - ey0)
    assert float(np.var(increments_x, ddof=1)) == pytest.approx(expected_var, rel=0.25)
    assert float(np.var(increments_y, ddof=1)) == pytest.approx(expected_var, rel=0.25)


def test_heading_bias_is_constant_and_position_offset_applies() -> None:
    model = _make_model(seed=11)
    for yaw_rad in (0.0, 1.0, -2.5):
        _, _, noisy_yaw = model.noisy_pose(10.0, 20.0, yaw_rad)
        assert noisy_yaw == pytest.approx(yaw_rad + HEADING_BIAS_RAD)
    model.step(2.0, 1.0)
    x_m, y_m, _ = model.noisy_pose(10.0, 20.0, 0.0)
    ex_m, ey_m = model.position_error()
    assert x_m == pytest.approx(10.0 + ex_m)
    assert y_m == pytest.approx(20.0 + ey_m)


def test_covariance_accessors_match_modelled_variance() -> None:
    model = _make_model(seed=5)
    model.step(100.0, 0.0)
    assert model.position_variance() == pytest.approx(0.26)
    assert model.heading_variance() == pytest.approx(INIT_HEADING_VAR)


def test_depth_noise_statistics() -> None:
    rng = np.random.default_rng(seed=99)
    samples = np.array([depth_noise_sample(rng, 0.02) for _ in range(20000)])
    assert abs(float(samples.mean())) < 0.001
    assert 0.018 <= float(samples.std(ddof=1)) <= 0.022


def test_yaw_quaternion_round_trip() -> None:
    assert quaternion_about_yaw(0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    for yaw_rad in (0.5, math.pi / 2, 3.0, -3.0):
        qx, qy, qz, qw = quaternion_about_yaw(yaw_rad)
        norm = qx * qx + qy * qy + qz * qz + qw * qw
        assert math.isclose(norm, 1.0, abs_tol=1e-12)
        assert yaw_from_quaternion(qx, qy, qz, qw) == pytest.approx(yaw_rad)


def test_negative_model_parameters_raise() -> None:
    with pytest.raises(ValueError):
        OdomErrorModel(-0.001, 0.0, 0.01, 0.001, seed=1)
    with pytest.raises(ValueError):
        OdomErrorModel(0.005, 0.0, -0.01, 0.001, seed=1)

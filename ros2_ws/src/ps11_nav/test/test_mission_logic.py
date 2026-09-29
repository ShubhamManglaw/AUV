"""Unit tests for the T1.5 waypoint-follower mission logic (plan §9.1).

ROS-free: no rclpy import, no ROS graph. Covers the planner, the angle and
controller maths, the depth-control sign (Amendment 1), the body-frame
command semantics (Amendment 2) and the mission state machine.
"""

from __future__ import annotations

import math
from itertools import pairwise

import pytest
from ps11_nav.mission_logic import (
    STATE_FAULT,
    STATE_IDLE,
    STATE_RETURN,
    STATE_SURVEY,
    STATE_TRANSIT,
    Command,
    ControlConfig,
    LawnmowerConfig,
    MissionFSM,
    depth_rate,
    forward_speed,
    generate_lawnmower,
    heading_rate,
    leg_y_positions,
    reached,
    wrap_angle,
)

DEMO = LawnmowerConfig(
    area_length_m=50.0,
    area_width_m=30.0,
    area_center_x_m=0.0,
    area_center_y_m=0.0,
    leg_spacing_m=4.0,
)


def make_config(**overrides: float) -> ControlConfig:
    values: dict[str, float] = {
        "k_yaw": 1.5,
        "max_yaw_rate_rad_s": 1.0,
        "yaw_gate_rad": 0.5,
        "k_depth": 0.5,
        "max_vz_mps": 0.2,
        "slowdown_radius_m": 2.0,
        "waypoint_tolerance_m": 2.0,
        "survey_speed_mps": 1.0,
        "turn_speed_mps": 0.5,
        "progress_timeout_s": 20.0,
        "progress_min_m": 0.05,
    }
    values.update(overrides)
    return ControlConfig(**values)


def run_perfect_mission(
    fsm: MissionFSM, dt: float = 0.05, max_s: float = 1500.0
) -> float:
    """Drive the FSM with a perfect kinematic vehicle; return sim seconds."""
    x, y, yaw = 0.0, 0.0, 0.0
    t = 0.0
    while not fsm.mission_complete and t < max_s:
        target_x, target_y = fsm.current_waypoint()
        cmd = fsm.tick(x, y, yaw, 12.5, 12.5, dt)
        target_yaw = math.atan2(target_y - y, target_x - x)
        yaw = wrap_angle(target_yaw)
        x += cmd.linear_x * math.cos(yaw) * dt
        y += cmd.linear_x * math.sin(yaw) * dt
        t += dt
    if not fsm.mission_complete:
        raise AssertionError("FSM never completed the mission")
    return t


# ---------------------------------------------------------------- planner


def test_leg_positions_are_symmetric_and_bounded() -> None:
    ys = leg_y_positions(DEMO)
    span = (len(ys) - 1) * DEMO.leg_spacing_m
    lower = DEMO.area_center_y_m - DEMO.area_width_m / 2.0
    upper = DEMO.area_center_y_m + DEMO.area_width_m / 2.0
    assert ys[0] == pytest.approx(DEMO.area_center_y_m - span / 2.0)
    assert ys[-1] == pytest.approx(DEMO.area_center_y_m + span / 2.0)
    assert (ys[0] - lower) == pytest.approx(upper - ys[-1])
    for y in ys:
        assert lower <= y <= upper


def test_leg_spacing_matches_configuration() -> None:
    ys = leg_y_positions(DEMO)
    for a, b in pairwise(ys):
        assert (b - a) == pytest.approx(DEMO.leg_spacing_m)


def test_leg_count_fits_width_without_blind_assumption() -> None:
    # 30 m width / 4 m spacing -> 8 legs (span 28 m, 1 m margins each side).
    assert len(leg_y_positions(DEMO)) == 8
    # Width not divisible by spacing: 5 m / 4 m -> 2 legs spanning 4 m.
    narrow = LawnmowerConfig(50.0, 5.0, 0.0, 0.0, 4.0)
    ys = leg_y_positions(narrow)
    assert len(ys) == 2
    assert (ys[1] - ys[0]) == pytest.approx(4.0)
    # Width smaller than spacing -> a single centred leg.
    tiny = LawnmowerConfig(50.0, 3.0, 0.0, 0.0, 4.0)
    assert leg_y_positions(tiny) == [0.0]


def test_generation_is_deterministic_and_bounded() -> None:
    assert generate_lawnmower(DEMO) == generate_lawnmower(DEMO)
    plan, _stats = generate_lawnmower(DEMO)
    half_l = DEMO.area_length_m / 2.0
    half_w = DEMO.area_width_m / 2.0
    for x, y in plan:
        assert -half_l <= x <= half_l
        assert -half_w <= y <= half_w


def test_boustrophedon_ordering_alternates_direction() -> None:
    plan, _stats = generate_lawnmower(DEMO)
    assert plan[0] == pytest.approx((-25.0, -14.0))
    assert plan[1] == pytest.approx((25.0, -14.0))
    assert plan[2] == pytest.approx((25.0, -10.0))
    assert plan[3] == pytest.approx((-25.0, -10.0))
    # Consecutive waypoints on one leg share y; turn hops share x.
    x0, y0 = plan[0]
    x1, y1 = plan[1]
    assert y1 == pytest.approx(y0)
    assert x1 != x0
    x2, y2 = plan[2]
    assert x2 == pytest.approx(x1)
    assert y2 == pytest.approx(y1 + DEMO.leg_spacing_m)


def test_planned_lengths_and_waypoint_count() -> None:
    _plan, stats = generate_lawnmower(DEMO)
    assert stats["leg_count"] == 8
    # 16 survey waypoints + 1 appended return point = 17 in the executed route.
    assert stats["waypoint_count"] == 17
    # 8 legs x 50 m + 7 turn hops x 4 m = 428 m (calculated, not measured).
    assert stats["survey_length_m"] == pytest.approx(8 * 50.0 + 7 * 4.0)
    # Total geometric route adds centre->first and last->centre legs.
    transit = math.hypot(25.0, 14.0)
    assert stats["route_length_m"] == pytest.approx(428.0 + 2 * transit)


def test_return_waypoint_is_area_centre_when_enabled() -> None:
    plan, stats = generate_lawnmower(DEMO, return_to_center=True)
    assert stats["return_point"] == pytest.approx((0.0, 0.0))
    assert plan[-1] == pytest.approx((0.0, 0.0))


def test_return_waypoint_omitted_when_disabled() -> None:
    plan, stats = generate_lawnmower(DEMO, return_to_center=False)
    assert stats["return_point"] is None
    assert plan[-1] != pytest.approx((0.0, 0.0))


def test_invalid_planner_config_rejected() -> None:
    with pytest.raises(ValueError):
        LawnmowerConfig(50.0, 30.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        LawnmowerConfig(-50.0, 30.0, 0.0, 0.0, 4.0)
    with pytest.raises(ValueError):
        LawnmowerConfig(50.0, 0.0, 0.0, 0.0, 4.0)


def test_invalid_control_config_rejected() -> None:
    with pytest.raises(ValueError):
        make_config(k_yaw=0.0)
    with pytest.raises(ValueError):
        make_config(max_yaw_rate_rad_s=-1.0)
    with pytest.raises(ValueError):
        make_config(progress_timeout_s=0.0)


# ---------------------------------------------------- angle / controller


def test_angle_wrap() -> None:
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(4.0 * math.pi + 0.5) == pytest.approx(0.5)
    assert wrap_angle(-4.0 * math.pi - 0.5) == pytest.approx(-0.5)


def test_heading_toward_cardinal_targets() -> None:
    # Vehicle at the origin facing +x (yaw 0); targets north/east/south/west.
    cases = {
        (0.0, 10.0): math.pi / 2,  # north
        (10.0, 0.0): 0.0,  # east
        (0.0, -10.0): -math.pi / 2,  # south
        (-10.0, 0.0): math.pi,  # west
    }
    for (tx, ty), expected in cases.items():
        yaw_error = wrap_angle(math.atan2(ty, tx) - 0.0)
        assert yaw_error == pytest.approx(expected)
        angular_z = heading_rate(yaw_error, k_yaw=1.0, max_yaw_rate_rad_s=10.0)
        assert angular_z == pytest.approx(expected)


def test_yaw_error_wraps_across_pi_boundary() -> None:
    # Facing +170 deg, target -170 deg: the short way is +20 deg, not -340.
    yaw_error = wrap_angle(math.radians(-170.0) - math.radians(170.0))
    assert yaw_error == pytest.approx(math.radians(20.0))
    assert heading_rate(yaw_error, 1.0, 10.0) > 0.0


def test_yaw_rate_saturates() -> None:
    assert heading_rate(10.0 * math.pi, 1.5, 1.0) == pytest.approx(1.0)
    assert heading_rate(-10.0 * math.pi, 1.5, 1.0) == pytest.approx(-1.0)
    assert heading_rate(0.1, 1.5, 1.0) == pytest.approx(0.15)


def test_forward_speed_yaw_gate_reduces_speed() -> None:
    # Small heading error: full survey speed far from the waypoint.
    assert forward_speed(50.0, 0.1, 1.0, 0.5, 2.0, 0.5) == pytest.approx(1.0)
    # Large heading error: gated down to turn speed.
    assert forward_speed(50.0, 0.9, 1.0, 0.5, 2.0, 0.5) == pytest.approx(0.5)


def test_forward_speed_slowdown_near_waypoint() -> None:
    # Linear slowdown inside the slowdown radius, zero at the waypoint.
    assert forward_speed(1.0, 0.0, 1.0, 0.5, 2.0, 0.5) == pytest.approx(0.5)
    assert forward_speed(2.0, 0.0, 1.0, 0.5, 2.0, 0.5) == pytest.approx(1.0)
    assert forward_speed(0.0, 0.0, 1.0, 0.5, 2.0, 0.5) == pytest.approx(0.0)


def test_waypoint_reach_detection() -> None:
    assert reached(1.9, 2.0)
    assert reached(2.0, 2.0)
    assert not reached(2.1, 2.0)


# ----------------------------------------------------------------- depth


def test_depth_too_deep_commands_positive_z() -> None:
    # Measured 12.6 m, target 12.5 m: too deep -> ascend (+z). (Amendment 1)
    vz = depth_rate(12.6, 12.5, 0.5, 0.2)
    assert vz > 0.0
    assert vz == pytest.approx(0.05)


def test_depth_too_shallow_commands_negative_z() -> None:
    vz = depth_rate(12.4, 12.5, 0.5, 0.2)
    assert vz < 0.0
    assert vz == pytest.approx(-0.05)


def test_depth_at_target_commands_zero_z() -> None:
    assert depth_rate(12.5, 12.5, 0.5, 0.2) == pytest.approx(0.0)


def test_depth_vertical_speed_saturates() -> None:
    assert depth_rate(20.0, 12.5, 0.5, 0.2) == pytest.approx(0.2)
    assert depth_rate(1.0, 12.5, 0.5, 0.2) == pytest.approx(-0.2)


# ---------------------------------------------------------- state machine


def build_fsm() -> tuple[MissionFSM, ControlConfig]:
    plan, _stats = generate_lawnmower(DEMO)
    cfg = make_config()
    return MissionFSM(plan, cfg, has_return_point=True), cfg


def test_fsm_starts_idle_and_commands_zero_before_start() -> None:
    fsm, _cfg = build_fsm()
    assert fsm.state == STATE_IDLE
    cmd = fsm.tick(0.0, 0.0, 0.0, 12.5, 12.5, 0.05)
    assert cmd == Command(0.0, 0.0, 0.0, 0.0)
    assert fsm.state == STATE_IDLE


def test_fsm_idle_to_transit_on_start() -> None:
    fsm, _cfg = build_fsm()
    fsm.start_mission()
    assert fsm.state == STATE_TRANSIT
    assert fsm.current_waypoint() == pytest.approx((-25.0, -14.0))


def test_fsm_transit_to_survey_at_first_waypoint() -> None:
    fsm, _cfg = build_fsm()
    fsm.start_mission()
    first = fsm.current_waypoint()
    fsm.tick(first[0], first[1], 0.0, 12.5, 12.5, 0.05)
    assert fsm.state == STATE_SURVEY
    assert fsm.current_waypoint() == pytest.approx((25.0, -14.0))


def test_fsm_survey_waypoints_advance_in_order() -> None:
    plan, _stats = generate_lawnmower(DEMO)
    fsm, _cfg = build_fsm()
    fsm.start_mission()
    visited = [fsm.current_waypoint()]
    for waypoint in plan[1:]:
        x, y = visited[-1]
        fsm.tick(x, y, 0.0, 12.5, 12.5, 0.05)
        visited.append(fsm.current_waypoint())
    assert visited == [pytest.approx(w) for w in plan]


def test_fsm_full_mission_transit_survey_return_idle() -> None:
    fsm, _cfg = build_fsm()
    fsm.start_mission()
    assert fsm.state == STATE_TRANSIT
    states = [fsm.state]
    x, y, yaw = 0.0, 0.0, 0.0
    dt = 0.05
    t = 0.0
    while fsm.state != STATE_IDLE and t < 1500.0:
        target_x, target_y = fsm.current_waypoint()
        cmd = fsm.tick(x, y, yaw, 12.5, 12.5, dt)
        yaw = wrap_angle(math.atan2(target_y - y, target_x - x))
        x += cmd.linear_x * math.cos(yaw) * dt
        y += cmd.linear_x * math.sin(yaw) * dt
        t += dt
        if fsm.state != states[-1]:
            states.append(fsm.state)
    assert fsm.state == STATE_IDLE
    assert fsm.mission_complete
    assert states == [STATE_TRANSIT, STATE_SURVEY, STATE_RETURN, STATE_IDLE]
    # After completion the FSM keeps commanding zero.
    assert fsm.tick(5.0, 5.0, 0.0, 12.5, 12.5, dt) == Command(0.0, 0.0, 0.0, 0.0)


def test_fsm_state_values_follow_heartbeat_contract() -> None:
    # Plan §11.2 heartbeat state field; codec.py validates the field as 0-7.
    assert STATE_IDLE == 0
    assert STATE_TRANSIT == 1
    assert STATE_SURVEY == 2
    assert STATE_RETURN == 4
    assert STATE_FAULT == 5


def test_fsm_stuck_watchdog_triggers_fault_with_zero_command() -> None:
    fsm, cfg = build_fsm()
    fsm.start_mission()
    dt = 0.05
    t = 0.0
    cmd = fsm.tick(0.0, 0.0, 0.0, 12.5, 12.5, dt)
    while fsm.state != STATE_FAULT and t <= cfg.progress_timeout_s + 1.0:
        cmd = fsm.tick(0.0, 0.0, 0.0, 12.5, 12.5, dt)
        t += dt
    assert fsm.state == STATE_FAULT
    assert cmd == Command(0.0, 0.0, 0.0, 0.0)
    # FAULT is terminal: still zero command, state unchanged.
    for _ in range(10):
        assert fsm.tick(0.0, 0.0, 0.0, 12.5, 12.5, dt) == Command(0.0, 0.0, 0.0, 0.0)
    assert fsm.state == STATE_FAULT


def test_fsm_watchdog_resets_on_progress_and_waypoint_change() -> None:
    fsm, _cfg = build_fsm()
    fsm.start_mission()
    dt = 0.05
    # Close 0.5 m toward the first waypoint every 5 s of sim time: each
    # improvement is >= progress_min_m, so the watchdog must never fire.
    steps_per_leg = int(5.0 / dt)
    x, y = 0.0, 0.0
    for _ in range(40):
        for _ in range(steps_per_leg):
            fsm.tick(x, y, 0.0, 12.5, 12.5, dt)
            assert fsm.state != STATE_FAULT
        x -= 0.5
    assert fsm.state in (STATE_TRANSIT, STATE_SURVEY)


def test_fsm_command_is_body_frame_toward_map_waypoint() -> None:
    # Amendment 2: a map-frame waypoint due north of an east-facing vehicle
    # must produce a left turn (+angular.z), forward-only body motion
    # (linear.y == 0) and turn-speed gating while the heading error is large.
    plan, _stats = generate_lawnmower(DEMO)
    cfg = make_config()
    fsm = MissionFSM(plan, cfg, has_return_point=True)
    fsm.start_mission()
    cmd = fsm.tick(-25.0, -20.0, 0.0, 12.5, 12.5, 0.05)
    assert cmd.angular_z > 0.0
    assert cmd.linear_x == pytest.approx(cfg.turn_speed_mps)
    assert cmd.linear_y == 0.0

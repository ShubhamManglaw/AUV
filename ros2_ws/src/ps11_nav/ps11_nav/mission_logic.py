"""ROS-free mission logic for the T1.5 waypoint follower (plan §9.1).

Pure logic only — no rclpy imports. The ROS wrapper is
waypoint_follower_node.py.

Contracts used (verified against the repository before coding):

- Mission state values are the heartbeat ``state`` field values from plan
  §11.2 (0 idle, 1 transit, 2 survey, 3 inspect, 4 return, 5 fault);
  ``ps11_telemetry/codec.py`` validates the field as 3 bits (0-7).
- Frame semantics (Amendment 2): waypoints and odometry positions are in the
  ``map`` frame; commands are body-frame (linear.x forward, angular.z yaw
  rate). The controller turns toward the map-frame waypoint and drives along
  body X; map-frame dx/dy is never used as a linear command.
- Depth sign (Amendment 1): ENU z is up-positive and depth = -z, so a
  measured depth greater than the target means too deep and commands +z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Heartbeat state field values (plan §11.2; codec.py validates 0-7).
STATE_IDLE = 0
STATE_TRANSIT = 1
STATE_SURVEY = 2
STATE_INSPECT = 3  # unused in M1
STATE_RETURN = 4
STATE_FAULT = 5


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    angle = angle_rad
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass(frozen=True)
class LawnmowerConfig:
    """Survey area and leg spacing (all values from mission.yaml)."""

    area_length_m: float
    area_width_m: float
    area_center_x_m: float
    area_center_y_m: float
    leg_spacing_m: float

    def __post_init__(self) -> None:
        if self.area_length_m <= 0.0:
            raise ValueError("area_length_m must be > 0")
        if self.area_width_m <= 0.0:
            raise ValueError("area_width_m must be > 0")
        if self.leg_spacing_m <= 0.0:
            raise ValueError("leg_spacing_m must be > 0")

    @property
    def x_min_m(self) -> float:
        return self.area_center_x_m - self.area_length_m / 2.0

    @property
    def x_max_m(self) -> float:
        return self.area_center_x_m + self.area_length_m / 2.0


@dataclass(frozen=True)
class ControlConfig:
    """Controller gains and limits (all values from mission.yaml)."""

    k_yaw: float
    max_yaw_rate_rad_s: float
    yaw_gate_rad: float
    k_depth: float
    max_vz_mps: float
    slowdown_radius_m: float
    waypoint_tolerance_m: float
    survey_speed_mps: float
    turn_speed_mps: float
    progress_timeout_s: float
    progress_min_m: float

    def __post_init__(self) -> None:
        positive = {
            "k_yaw": self.k_yaw,
            "max_yaw_rate_rad_s": self.max_yaw_rate_rad_s,
            "yaw_gate_rad": self.yaw_gate_rad,
            "k_depth": self.k_depth,
            "max_vz_mps": self.max_vz_mps,
            "slowdown_radius_m": self.slowdown_radius_m,
            "waypoint_tolerance_m": self.waypoint_tolerance_m,
            "survey_speed_mps": self.survey_speed_mps,
            "turn_speed_mps": self.turn_speed_mps,
            "progress_timeout_s": self.progress_timeout_s,
            "progress_min_m": self.progress_min_m,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be > 0")


def leg_y_positions(cfg: LawnmowerConfig) -> list[float]:
    """Leg Y coordinates: as many legs as fit at the configured spacing,
    centred so the residual width is split symmetrically.

    n = floor(width / spacing) + 1 legs span (n-1)*spacing <= width; the
    leftover width is distributed equally to both sides.
    """
    n_legs = math.floor(cfg.area_width_m / cfg.leg_spacing_m) + 1
    span_m = (n_legs - 1) * cfg.leg_spacing_m
    first_y = cfg.area_center_y_m - span_m / 2.0
    return [first_y + i * cfg.leg_spacing_m for i in range(n_legs)]


def generate_lawnmower(
    cfg: LawnmowerConfig, return_to_center: bool = True
) -> tuple[list[tuple[float, float]], dict[str, object]]:
    """Generate the boustrophedon lawnmower plan deterministically.

    Legs run along the area length (x); even-indexed legs travel x_min ->
    x_max, odd-indexed legs x_max -> x_min. Turn hops between legs are the
    lateral waypoint-to-waypoint segments. With return_to_center the final
    waypoint is the area centre (the launch point) and has_return_point is
    True so MissionFSM treats it as the RETURN target.

    Returns (waypoints, stats). All lengths are geometric calculations, not
    measured mission durations.
    """
    ys = leg_y_positions(cfg)
    waypoints: list[tuple[float, float]] = []
    for i, y in enumerate(ys):
        if i % 2 == 0:
            waypoints.append((cfg.x_min_m, y))
            waypoints.append((cfg.x_max_m, y))
        else:
            waypoints.append((cfg.x_max_m, y))
            waypoints.append((cfg.x_min_m, y))

    survey_length = cfg.area_length_m * len(ys) + cfg.leg_spacing_m * (len(ys) - 1)
    first = waypoints[0]
    transit_leg = math.hypot(
        first[0] - cfg.area_center_x_m, first[1] - cfg.area_center_y_m
    )
    return_point: tuple[float, float] | None = (
        (cfg.area_center_x_m, cfg.area_center_y_m) if return_to_center else None
    )
    # Return leg runs from the last SURVEY waypoint to the return point and
    # must be measured before the return point is appended.
    last_survey = waypoints[-1]
    return_leg = (
        math.hypot(last_survey[0] - return_point[0], last_survey[1] - return_point[1])
        if return_point is not None
        else 0.0
    )
    if return_point is not None:
        waypoints.append(return_point)

    stats: dict[str, object] = {
        "leg_count": len(ys),
        "waypoint_count": len(waypoints),
        "survey_length_m": survey_length,
        "route_length_m": survey_length + transit_leg + return_leg,
        "return_point": return_point,
        "has_return_point": return_point is not None,
    }
    return waypoints, stats


@dataclass(frozen=True)
class Command:
    """Body-frame command components (geometry_msgs/Twist fields)."""

    linear_x: float
    linear_y: float
    linear_z: float
    angular_z: float


def heading_rate(
    yaw_error_rad: float, k_yaw: float, max_yaw_rate_rad_s: float
) -> float:
    """Proportional heading control with saturation."""
    return clamp(k_yaw * yaw_error_rad, -max_yaw_rate_rad_s, max_yaw_rate_rad_s)


def forward_speed(
    distance_m: float,
    abs_yaw_error_rad: float,
    survey_speed_mps: float,
    turn_speed_mps: float,
    slowdown_radius_m: float,
    yaw_gate_rad: float,
) -> float:
    """Forward speed: constant survey speed, linearly slowed inside the
    slowdown radius (plan §9.1 "slow down within 2 m of a waypoint"), and
    capped at turn speed while the heading error exceeds the yaw gate.
    """
    speed = survey_speed_mps * clamp(distance_m / slowdown_radius_m, 0.0, 1.0)
    if abs_yaw_error_rad > yaw_gate_rad:
        speed = min(speed, turn_speed_mps)
    return speed


def depth_rate(
    measured_depth_m: float, target_depth_m: float, k_depth: float, max_vz_mps: float
) -> float:
    """Proportional depth control (Amendment 1).

    depth = -z with z up-positive, so measured > target means too deep and
    the vehicle must command positive z to ascend.
    """
    depth_error = measured_depth_m - target_depth_m
    return clamp(k_depth * depth_error, -max_vz_mps, max_vz_mps)


def reached(distance_m: float, tolerance_m: float) -> bool:
    return distance_m <= tolerance_m


class MissionFSM:
    """Mission state machine over the planned waypoints.

    Waypoint index -> state mapping: index 0 is TRANSIT (drive to the first
    survey waypoint), indices 1..n_survey-1 are SURVEY, the appended return
    point (has_return_point=True) is RETURN, and completing it returns to
    IDLE (mission complete). FAULT is terminal for the run.

    The stuck watchdog uses vehicle-side navigation only: the best distance
    to the current waypoint must improve by at least progress_min_m before
    progress_timeout_s elapses, otherwise the FSM faults with zero command.
    The watchdog resets whenever the current waypoint changes.
    """

    ZERO = Command(0.0, 0.0, 0.0, 0.0)

    def __init__(
        self,
        waypoints: list[tuple[float, float]],
        config: ControlConfig,
        has_return_point: bool = False,
    ) -> None:
        if not waypoints:
            raise ValueError("waypoints must not be empty")
        self._waypoints = list(waypoints)
        self._cfg = config
        self._return_index: int | None = (
            len(self._waypoints) - 1 if has_return_point else None
        )
        self._started = False
        self._mission_complete = False
        self._faulted = False
        self._wp_index = 0
        self._best_distance_m: float | None = None
        self._progress_timer_s = 0.0

    @property
    def state(self) -> int:
        if self._faulted:
            return STATE_FAULT
        if not self._started or self._mission_complete:
            return STATE_IDLE
        if self._wp_index == 0:
            return STATE_TRANSIT
        if self._return_index is not None and self._wp_index >= self._return_index:
            return STATE_RETURN
        return STATE_SURVEY

    @property
    def mission_complete(self) -> bool:
        return self._mission_complete

    def current_waypoint(self) -> tuple[float, float]:
        if self._wp_index >= len(self._waypoints):
            raise IndexError("no current waypoint (mission complete)")
        return self._waypoints[self._wp_index]

    def start_mission(self) -> None:
        if self._mission_complete or self._faulted:
            return
        self._started = True

    def _reset_watchdog(self, distance_m: float) -> None:
        self._best_distance_m = distance_m
        self._progress_timer_s = 0.0

    def _advance(self) -> None:
        self._wp_index += 1
        self._best_distance_m = None
        self._progress_timer_s = 0.0
        if self._wp_index >= len(self._waypoints):
            self._mission_complete = True

    def tick(
        self,
        nav_x_m: float,
        nav_y_m: float,
        nav_yaw_rad: float,
        measured_depth_m: float,
        target_depth_m: float,
        dt_s: float,
    ) -> Command:
        """One control tick; returns the body-frame command (zero when
        idle, complete or faulted)."""
        if not self._started or self._mission_complete or self._faulted:
            return self.ZERO
        if self._wp_index >= len(self._waypoints):
            return self.ZERO

        wp_x, wp_y = self._waypoints[self._wp_index]
        distance = math.hypot(wp_x - nav_x_m, wp_y - nav_y_m)

        # Stuck watchdog (vehicle-side navigation only).
        if (
            self._best_distance_m is None
            or self._best_distance_m - distance >= self._cfg.progress_min_m
        ):
            self._reset_watchdog(distance)
        else:
            self._progress_timer_s += max(dt_s, 0.0)
        if self._progress_timer_s > self._cfg.progress_timeout_s:
            self._faulted = True
            return self.ZERO

        if reached(distance, self._cfg.waypoint_tolerance_m):
            self._advance()
            if self._mission_complete:
                return self.ZERO
            wp_x, wp_y = self._waypoints[self._wp_index]
            distance = math.hypot(wp_x - nav_x_m, wp_y - nav_y_m)

        target_yaw = math.atan2(wp_y - nav_y_m, wp_x - nav_x_m)
        yaw_error = wrap_angle(target_yaw - nav_yaw_rad)
        return Command(
            linear_x=forward_speed(
                distance,
                abs(yaw_error),
                self._cfg.survey_speed_mps,
                self._cfg.turn_speed_mps,
                self._cfg.slowdown_radius_m,
                self._cfg.yaw_gate_rad,
            ),
            linear_y=0.0,
            linear_z=depth_rate(
                measured_depth_m,
                target_depth_m,
                self._cfg.k_depth,
                self._cfg.max_vz_mps,
            ),
            angular_z=heading_rate(
                yaw_error, self._cfg.k_yaw, self._cfg.max_yaw_rate_rad_s
            ),
        )

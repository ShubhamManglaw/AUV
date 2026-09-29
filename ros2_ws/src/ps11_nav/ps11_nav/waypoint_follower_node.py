"""waypoint_follower ROS 2 node (§9.1, T1.5).

Thin wrapper around the ROS-free mission_logic module. Consumes ONLY
/vehicle/nav/odom and /vehicle/depth (H2: no /sim/* subscriptions — ground
truth is evaluation-only), and publishes:

- /vehicle/cmd_vel (geometry_msgs/Twist) at control_rate_hz — body-frame
  commands for the kinematic VelocityControl interface, which latches the
  last command, so zero Twist is published whenever the node is waiting for
  sensors, idle, complete, faulted or shutting down.
- /vehicle/mission/state (std_msgs/UInt8) at state_rate_hz — heartbeat state
  values per plan §11.2.

Startup is deterministic: the node stays IDLE with zero commands until both
a valid odometry and a valid depth message have arrived; the target depth is
captured from the first depth message. M1 mission start is automatic on
initialization (the plan defines no user start command) — transition from
initialized IDLE to TRANSIT.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ps11_common.params import load_yaml
from ps11_interfaces.msg import Depth
from rclpy.node import Node
from std_msgs.msg import UInt8

from ps11_nav.error_model import yaw_from_quaternion
from ps11_nav.mission_logic import (
    Command,
    ControlConfig,
    LawnmowerConfig,
    MissionFSM,
    generate_lawnmower,
)


class WaypointFollowerNode(Node):
    """Drive the lawnmower survey from vehicle-side navigation only."""

    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        mission = load_yaml("mission.yaml")
        scenario = str(mission.get("scenario", "demo"))
        lm = mission["lawnmower"]
        ctl = mission["control"]

        planner_cfg = LawnmowerConfig(
            area_length_m=float(lm["area_length_m"]),
            area_width_m=float(lm["area_width_m"]),
            area_center_x_m=float(lm["area_center_x_m"]),
            area_center_y_m=float(lm["area_center_y_m"]),
            leg_spacing_m=float(lm["leg_spacing_m"]),
        )
        control_cfg = ControlConfig(
            k_yaw=float(ctl["k_yaw"]),
            max_yaw_rate_rad_s=float(ctl["max_yaw_rate_rad_s"]),
            yaw_gate_rad=float(ctl["yaw_gate_rad"]),
            k_depth=float(ctl["k_depth"]),
            max_vz_mps=float(ctl["max_vz_mps"]),
            slowdown_radius_m=float(lm["slowdown_radius_m"]),
            waypoint_tolerance_m=float(lm["waypoint_tolerance_m"]),
            survey_speed_mps=float(lm["survey_speed_mps"]),
            turn_speed_mps=float(lm["turn_speed_mps"]),
            progress_timeout_s=float(ctl["progress_timeout_s"]),
            progress_min_m=float(ctl["progress_min_m"]),
        )
        return_to_center = bool(lm.get("return_to_center", True))
        waypoints, stats = generate_lawnmower(planner_cfg, return_to_center)
        self._fsm = MissionFSM(waypoints, control_cfg, return_to_center)

        self._control_rate_hz = float(ctl["control_rate_hz"])
        self._state_rate_hz = float(ctl["state_rate_hz"])
        if self._control_rate_hz <= 0.0 or self._state_rate_hz <= 0.0:
            raise ValueError("control_rate_hz and state_rate_hz must be > 0")

        # Vehicle-side inputs (the only navigation sources, H2-safe).
        self._nav: tuple[float, float, float] | None = None  # x, y, yaw
        self._measured_depth_m: float | None = None
        self._target_depth_m: float | None = None
        self._last_tick: float | None = None

        self.create_subscription(Odometry, "/vehicle/nav/odom", self._on_odom, 10)
        self.create_subscription(Depth, "/vehicle/depth", self._on_depth, 10)
        self._cmd_pub = self.create_publisher(Twist, "/vehicle/cmd_vel", 10)
        self._state_pub = self.create_publisher(UInt8, "/vehicle/mission/state", 10)

        self.create_timer(1.0 / self._control_rate_hz, self._on_control_timer)
        self.create_timer(1.0 / self._state_rate_hz, self._on_state_timer)

        # Calculated route facts (not measured mission durations).
        self.get_logger().info(
            f"Initialized waypoint_follower (scenario={scenario}): "
            f"legs={stats['leg_count']}, waypoints={stats['waypoint_count']}, "
            f"planned survey length={stats['survey_length_m']:.1f} m "
            f"(calculated), planned total geometric route="
            f"{stats['route_length_m']:.1f} m (calculated)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._nav = (
            float(p.x),
            float(p.y),
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def _on_depth(self, msg: Depth) -> None:
        self._measured_depth_m = float(msg.depth_m)
        if self._target_depth_m is None:
            self._target_depth_m = float(msg.depth_m)
            self.get_logger().info(
                f"Captured target depth {self._target_depth_m:.3f} m from "
                "first /vehicle/depth message"
            )
            self._fsm.start_mission()
            self.get_logger().info(
                "Mission started automatically (M1): IDLE -> TRANSIT"
            )

    def _publish_command(self, command: Command) -> None:
        twist = Twist()
        twist.linear.x = command.linear_x
        twist.linear.y = command.linear_y
        twist.linear.z = command.linear_z
        twist.angular.z = command.angular_z
        self._cmd_pub.publish(twist)

    def _on_control_timer(self) -> None:
        if self._nav is None or self._measured_depth_m is None:
            # Not initialized: deterministic zero command (VelocityControl
            # latches the previous command).
            self._publish_command(Command(0.0, 0.0, 0.0, 0.0))
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt_s = 0.0 if self._last_tick is None else max(now - self._last_tick, 0.0)
        self._last_tick = now

        nav_x, nav_y, nav_yaw = self._nav
        command = self._fsm.tick(
            nav_x,
            nav_y,
            nav_yaw,
            self._measured_depth_m,
            float(self._target_depth_m),
            dt_s,
        )
        if self._fsm.state == 5:  # STATE_FAULT
            self.get_logger().error(
                "Mission FAULT: no progress toward the current waypoint for "
                "progress_timeout_s",
                throttle_duration_sec=10.0,
            )
        self._publish_command(command)

    def _on_state_timer(self) -> None:
        msg = UInt8()
        msg.data = self._fsm.state
        self._state_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # VelocityControl latches: best-effort zero command on the way out.
        try:
            node._publish_command(Command(0.0, 0.0, 0.0, 0.0))
        except rclpy.executors.ExternalShutdownException:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

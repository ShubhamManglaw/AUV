"""ROS 2 node for waypoint_follower (§9.1, T1.5)."""

from __future__ import annotations

import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8

from ps11_common.params import load_yaml
from ps11_nav.lawnmower import Waypoint, generate_lawnmower_waypoints
from ps11_nav.odom_noise_node import euler_from_quaternion

# Mission states from §11.2 HEARTBEAT
STATE_IDLE = 0
STATE_TRANSIT = 1
STATE_SURVEY = 2
STATE_RETURN = 4


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        # Load parameters from mission.yaml (§9.1, no hard-coded constants)
        mission_cfg = load_yaml("mission.yaml")
        scenario_name = mission_cfg.get("scenario", "demo")
        scenarios = mission_cfg.get("scenarios", {})
        sc_cfg = scenarios.get(scenario_name, mission_cfg.get("lawnmower", {}))

        origin_x = float(sc_cfg.get("survey_origin_x_m", 5.0))
        origin_y = float(sc_cfg.get("survey_origin_y_m", -10.0))
        area_x = float(sc_cfg.get("area_x_m", 40.0))
        area_y = float(sc_cfg.get("area_y_m", 20.0))
        leg_spacing = float(sc_cfg.get("leg_spacing_m", 4.0))
        altitude = float(sc_cfg.get("altitude_m", 2.5))
        seabed_z = float(sc_cfg.get("seabed_z_m", -15.0))

        self.survey_speed = float(sc_cfg.get("survey_speed_mps", 1.0))
        self.turn_speed = float(sc_cfg.get("turn_speed_mps", 0.5))
        self.waypoint_tolerance = float(sc_cfg.get("waypoint_tolerance_m", 2.0))

        # Generate lawnmower waypoints in map frame
        self.waypoints = generate_lawnmower_waypoints(
            origin_x=origin_x,
            origin_y=origin_y,
            area_x=area_x,
            area_y=area_y,
            leg_spacing=leg_spacing,
            altitude=altitude,
            seabed_z=seabed_z,
        )

        self.target_z = seabed_z + altitude
        self.home_waypoint = Waypoint(0.0, 0.0, self.target_z)

        # State machine
        self.state = STATE_TRANSIT
        self.current_wp_idx = 0

        # Current estimated pose from /vehicle/nav/odom (NEVER /sim/* per H2!)
        self.current_x: float | None = None
        self.current_y: float | None = None
        self.current_z: float | None = None
        self.current_yaw: float | None = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_odom = self.create_subscription(
            Odometry,
            "/vehicle/nav/odom",
            self._on_nav_odom,
            qos,
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            "/vehicle/cmd_vel",
            qos,
        )
        self.pub_state = self.create_publisher(
            UInt8,
            "/vehicle/mission/state",
            qos,
        )

        # Control loop at 20 Hz (§14.1)
        self.cmd_timer = self.create_timer(0.05, self._on_control_loop)
        # State publisher at 1 Hz (§14.1)
        self.state_timer = self.create_timer(1.0, self._on_state_timer)

        self.get_logger().info(
            f"waypoint_follower initialized: {len(self.waypoints)} survey waypoints, "
            f"target altitude {altitude}m (z={self.target_z}m)"
        )

    def _on_nav_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.current_x = p.x
        self.current_y = p.y
        self.current_z = p.z
        _, _, self.current_yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)

    def _on_state_timer(self) -> None:
        msg = UInt8()
        msg.data = self.state
        self.pub_state.publish(msg)

    def _on_control_loop(self) -> None:
        if (
            self.current_x is None
            or self.current_y is None
            or self.current_z is None
            or self.current_yaw is None
        ):
            return

        twist = Twist()

        if self.state == STATE_IDLE:
            self.pub_cmd_vel.publish(twist)
            return

        # Determine target waypoint based on mission state
        if self.state == STATE_TRANSIT:
            target_wp = self.waypoints[0]
            dist_to_wp = math.hypot(
                target_wp.x - self.current_x, target_wp.y - self.current_y
            )
            if dist_to_wp < self.waypoint_tolerance:
                self.get_logger().info(
                    f"Transit complete at ({self.current_x:.2f}, {self.current_y:.2f}). Starting SURVEY."
                )
                self.state = STATE_SURVEY
                self.current_wp_idx = 1
                self._on_state_timer()
                target_wp = self.waypoints[1]

        elif self.state == STATE_SURVEY:
            target_wp = self.waypoints[self.current_wp_idx]
            prev_wp = self.waypoints[self.current_wp_idx - 1]
            dist_to_wp = math.hypot(
                target_wp.x - self.current_x, target_wp.y - self.current_y
            )

            # Waypoint reached check:
            # - Straight legs (odd idx): along X, reached when within 1.5m of endpoint or crossed perpendicular plane
            # - Step-overs (even idx): along Y, reached only when within 0.6m of the target Y line or crossed it
            is_straight_leg = self.current_wp_idx % 2 == 1
            leg_dx = target_wp.x - prev_wp.x
            leg_dy = target_wp.y - prev_wp.y

            if is_straight_leg:
                dist_x = abs(self.current_x - target_wp.x)
                past_x = (self.current_x - target_wp.x) * leg_dx >= 0.0
                wp_reached = (dist_x < 1.5) or past_x
            else:
                dist_y = abs(self.current_y - target_wp.y)
                past_y = (self.current_y - target_wp.y) * leg_dy >= 0.0
                wp_reached = (dist_y < 0.6) or past_y

            if wp_reached:
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.get_logger().info("Survey complete. Transitioning to RETURN.")
                    self.state = STATE_RETURN
                    self._on_state_timer()
                    target_wp = self.home_waypoint
                else:
                    target_wp = self.waypoints[self.current_wp_idx]

        elif self.state == STATE_RETURN:
            target_wp = self.home_waypoint
            dist_to_wp = math.hypot(
                target_wp.x - self.current_x, target_wp.y - self.current_y
            )
            if dist_to_wp < self.waypoint_tolerance:
                self.get_logger().info(
                    f"Return complete at ({self.current_x:.2f}, {self.current_y:.2f}). Mission IDLE."
                )
                self.state = STATE_IDLE
                self._on_state_timer()
                self.pub_cmd_vel.publish(twist)
                return

        # Calculate guidance to target_wp
        dist_to_target = math.hypot(
            target_wp.x - self.current_x, target_wp.y - self.current_y
        )

        if self.state == STATE_SURVEY and (self.current_wp_idx % 2 == 1):
            # Straight survey leg: Line-Of-Sight (LOS) guidance for tight cross-track control
            wp_prev = self.waypoints[self.current_wp_idx - 1]
            wp_curr = self.waypoints[self.current_wp_idx]
            alpha = math.atan2(wp_curr.y - wp_prev.y, wp_curr.x - wp_prev.x)
            dx = self.current_x - wp_prev.x
            dy = self.current_y - wp_prev.y
            cross_track = -dx * math.sin(alpha) + dy * math.cos(alpha)
            delta_los = 4.0
            if dist_to_target > 2.0:
                desired_yaw = wrap_angle(alpha - math.atan2(cross_track, delta_los))
            else:
                desired_yaw = math.atan2(
                    target_wp.y - self.current_y, target_wp.x - self.current_x
                )
            speed = self.survey_speed
        elif self.state == STATE_SURVEY:
            # Turn / step-over between legs
            desired_yaw = math.atan2(
                target_wp.y - self.current_y, target_wp.x - self.current_x
            )
            speed = self.turn_speed
        else:
            # Transit or Return
            desired_yaw = math.atan2(
                target_wp.y - self.current_y, target_wp.x - self.current_x
            )
            speed = self.survey_speed

        yaw_error = wrap_angle(desired_yaw - self.current_yaw)

        # Proportional heading control (clamped)
        kp_yaw = 2.0
        yaw_rate = max(-1.2, min(1.2, kp_yaw * yaw_error))

        # Speed control: modulate speed during turns
        turn_factor = max(0.5, math.cos(yaw_error))
        v_forward = speed * turn_factor

        # Proportional depth / altitude control
        kp_z = 1.0
        z_error = target_wp.z - self.current_z
        v_z = max(-0.5, min(0.5, kp_z * z_error))

        twist.linear.x = float(v_forward)
        twist.linear.y = 0.0
        twist.linear.z = float(v_z)
        twist.angular.z = float(yaw_rate)

        self.pub_cmd_vel.publish(twist)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

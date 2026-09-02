"""Acceptance test runner for T1.3 and T1.5 (lawnmower demo mission)."""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from ps11_common.params import load_yaml
from ps11_interfaces.msg import Depth
from ps11_nav.lawnmower import generate_lawnmower_waypoints, point_to_segment_distance
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8

STATE_NAMES = {
    0: "IDLE",
    1: "TRANSIT",
    2: "SURVEY",
    4: "RETURN",
}


class LawnmowerMissionTracker(Node):
    def __init__(self) -> None:
        super().__init__("lawnmower_mission_tracker")
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
                )
            ]
        )

        self.gt_track: list[
            tuple[float, float, float, float, int]
        ] = []  # (t, x, y, z, state)
        self.state_history: list[tuple[float, int]] = []  # (t, state)
        self.altitude_samples: list[float] = []
        self.depth_samples: list[float] = []

        self.current_state: int = -1

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "/sim/gt/odom", self._on_gt_odom, qos)
        self.create_subscription(UInt8, "/vehicle/mission/state", self._on_state, qos)
        self.create_subscription(Range, "/vehicle/altitude", self._on_altitude, qos)
        self.create_subscription(Depth, "/vehicle/depth", self._on_depth, qos)

    def _on_gt_odom(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self.gt_track.append((t, p.x, p.y, p.z, self.current_state))

    def _on_state(self, msg: UInt8) -> None:
        t = time.time()
        if msg.data != self.current_state:
            old_name = STATE_NAMES.get(self.current_state, str(self.current_state))
            new_name = STATE_NAMES.get(msg.data, str(msg.data))
            print(f"State Transition: {old_name} -> {new_name}")
            self.current_state = msg.data
            self.state_history.append((t, self.current_state))

    def _on_altitude(self, msg: Range) -> None:
        if not math.isnan(msg.range):
            self.altitude_samples.append(msg.range)

    def _on_depth(self, msg: Depth) -> None:
        self.depth_samples.append(msg.depth_m)


def main() -> None:
    print("==========================================================")
    print("PS11-AUV: T1.3 & T1.5 Acceptance Verification (Demo Mission)")
    print("==========================================================")

    out_dir = Path("results/bench")
    out_dir.mkdir(parents=True, exist_ok=True)
    sim_log = open(out_dir / "sim_mission.log", "w")  # noqa: SIM115
    nav_log = open(out_dir / "nav_mission.log", "w")  # noqa: SIM115

    env = os.environ.copy()

    # Clean up before start
    subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)

    use_gui = "--gui" in sys.argv
    gui_val = "true" if use_gui else "false"

    print(f"Starting Gazebo simulation (mode:=kinematic, gui:={gui_val})...")
    sim_proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ps11_gazebo",
            "sim.launch.py",
            "mode:=kinematic",
            f"gui:={gui_val}",
        ],
        env=env,
        stdout=sim_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    print("Waiting 5s for Gazebo to load...")
    time.sleep(5.0)

    print("Starting navigation nodes (ps11_nav nav.launch.py)...")
    nav_proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ps11_nav",
            "nav.launch.py",
            "use_sim_time:=true",
        ],
        env=env,
        stdout=nav_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    time.sleep(2.0)

    rclpy.init()
    tracker = LawnmowerMissionTracker()

    start_wall_time = time.time()
    max_duration_sec = 480.0  # Up to 8 min timeout
    last_status_time = 0.0

    try:
        print("\nMonitoring mission progress...")
        while rclpy.ok() and (time.time() - start_wall_time < max_duration_sec):
            try:
                rclpy.spin_once(tracker, timeout_sec=0.05)
            except Exception:
                break

            now = time.time()
            if now - last_status_time >= 15.0:
                last_status_time = now
                elapsed = now - start_wall_time
                s_name = STATE_NAMES.get(
                    tracker.current_state, str(tracker.current_state)
                )
                if tracker.gt_track:
                    _, cur_x, cur_y, cur_z, _ = tracker.gt_track[-1]
                    print(
                        f"  [{elapsed:5.1f}s] State: {s_name:<7} | Pos: ({cur_x:5.1f}, {cur_y:5.1f}, {cur_z:5.1f})"
                    )
                else:
                    print(
                        f"  [{elapsed:5.1f}s] State: {s_name:<7} | Waiting for odometry..."
                    )

            # Mission completes when reaching IDLE (0) after having completed TRANSIT (1), SURVEY (2), and RETURN (4)
            seen_states = {s for _, s in tracker.state_history}
            if tracker.current_state == 0 and {1, 2, 4}.issubset(seen_states):
                print(
                    "\nMission completed! Vehicle has returned and reached IDLE state."
                )
                break

        total_elapsed = time.time() - start_wall_time
        print(
            f"\nTotal elapsed wall-clock time: {total_elapsed:.1f} s ({total_elapsed / 60.0:.2f} min)"
        )

        # Verify initial spawn checks (§9.2, T1.3)
        print("\n--- [T1.3 Checks: Initial Sensor Outputs] ---")
        init_alt = tracker.altitude_samples[:20]
        init_depth = tracker.depth_samples[:20]

        mean_alt = float(sum(init_alt) / len(init_alt)) if init_alt else float("nan")
        mean_depth = (
            float(sum(init_depth) / len(init_depth)) if init_depth else float("nan")
        )

        print(
            f"Initial /vehicle/altitude: {mean_alt:.2f} m (expected ≈ 2.5 m above seabed at -15)"
        )
        print(
            f"Initial /vehicle/depth:    {mean_depth:.2f} m (expected ≈ 12.5 m at z = -12.5)"
        )

        assert 2.0 < mean_alt < 3.0, f"Altitude {mean_alt:.2f}m not ≈ 2.5m"
        assert 12.0 < mean_depth < 13.0, f"Depth {mean_depth:.2f}m not ≈ 12.5m"
        print("T1.3 Initial sensor checks PASSED.")

        # Verify state transitions (§9.1, T1.5)
        print("\n--- [T1.5 Checks: State Machine Transitions] ---")
        visited_states = [s for _, s in tracker.state_history]
        print(
            f"Observed states: {[STATE_NAMES.get(s, str(s)) for s in visited_states]}"
        )

        # Expected: TRANSIT (1) -> SURVEY (2) -> RETURN (4) -> IDLE (0)
        assert 1 in visited_states, "TRANSIT state (1) not reached"
        assert 2 in visited_states, "SURVEY state (2) not reached"
        assert 4 in visited_states, "RETURN state (4) not reached"
        assert 0 in visited_states, "IDLE state (0) not reached"
        print("State machine transitions PASSED (TRANSIT -> SURVEY -> RETURN -> IDLE).")

        # Load planned survey waypoints and legs
        mission_cfg = load_yaml("mission.yaml")
        sc_cfg = mission_cfg.get("scenarios", {}).get(
            "demo", mission_cfg.get("lawnmower", {})
        )
        waypoints = generate_lawnmower_waypoints(
            origin_x=float(sc_cfg.get("survey_origin_x_m", 5.0)),
            origin_y=float(sc_cfg.get("survey_origin_y_m", -10.0)),
            area_x=float(sc_cfg.get("area_x_m", 40.0)),
            area_y=float(sc_cfg.get("area_y_m", 20.0)),
            leg_spacing=float(sc_cfg.get("leg_spacing_m", 4.0)),
            altitude=float(sc_cfg.get("altitude_m", 2.5)),
            seabed_z=float(sc_cfg.get("seabed_z_m", -15.0)),
        )

        legs: list[tuple[Waypoint, Waypoint]] = []
        for i in range(0, len(waypoints), 2):
            legs.append((waypoints[i], waypoints[i + 1]))

        # Calculate cross-track error along straight legs (excluding turns)
        print("\n--- [T1.5 Checks: Cross-Track Error on Planned Legs] ---")
        cross_track_errors: list[float] = []

        # Survey points: points recorded while in SURVEY state (2)
        survey_pts = [(x, y) for (_, x, y, _, s) in tracker.gt_track if s == 2]
        if not survey_pts:
            survey_pts = [(x, y) for (_, x, y, _, _) in tracker.gt_track]

        for px, py in survey_pts:
            # Exclude turn / step-over regions near the ends of the survey area
            # Planned legs run along X from 5.0 to 45.0; U-turns occur at x < 10.0 and x > 40.0
            if px < 10.0 or px > 40.0:
                continue

            # Minimum distance to any planned leg
            min_dist = min(
                point_to_segment_distance(px, py, w1.x, w1.y, w2.x, w2.y)
                for w1, w2 in legs
            )
            cross_track_errors.append(min_dist)

        max_cross_track = max(cross_track_errors) if cross_track_errors else 0.0
        mean_cross_track = (
            sum(cross_track_errors) / len(cross_track_errors)
            if cross_track_errors
            else 0.0
        )

        print(f"Survey Points Evaluated (away from turns): {len(cross_track_errors)}")
        print(f"Mean Cross-Track Distance:                 {mean_cross_track:.3f} m")
        print(
            f"Max Cross-Track Distance:                  {max_cross_track:.3f} m (requirement: <= 1.5 m)"
        )

        # Plot planned path vs ground truth track
        print("\n--- Generating Lawnmower Trajectory Plot ---")
        fig, ax = plt.subplots(figsize=(10, 6))

        # Planned path
        plan_x = [0.0] + [w.x for w in waypoints] + [0.0]
        plan_y = [0.0] + [w.y for w in waypoints] + [0.0]
        ax.plot(plan_x, plan_y, "r--o", label="Planned Path", alpha=0.7, markersize=5)

        # Ground truth track
        gt_x = [x for _, x, _, _, _ in tracker.gt_track]
        gt_y = [y for _, _, y, _, _ in tracker.gt_track]
        ax.plot(
            gt_x, gt_y, "b-", label="Ground Truth Track (/sim/gt/odom)", linewidth=1.5
        )

        # Launch origin
        ax.plot(0, 0, "g^", markersize=10, label="Launch Origin (0,0)")

        ax.set_title("PS11-AUV: Lawnmower Survey Trajectory (Kinematic Mode)")
        ax.set_xlabel("East X (m)")
        ax.set_ylabel("North Y (m)")
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(loc="upper right")
        ax.axis("equal")

        plot_path = out_dir / "lawnmower.png"
        fig.savefig(str(plot_path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved trajectory plot to {plot_path}")

        assert max_cross_track <= 1.5, (
            f"Cross-track error {max_cross_track:.3f} m exceeds 1.5 m limit"
        )
        print("Cross-track error requirement PASSED (<= 1.5 m).")

        print("\nALL T1.3 / T1.5 ACCEPTANCE CHECKS PASSED!")

    finally:
        print("\nShutting down navigation and simulation...")
        try:
            tracker.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        for proc in [nav_proc, sim_proc]:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass

        time.sleep(2.0)
        for proc in [nav_proc, sim_proc]:
            if proc.poll() is None:
                try:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    pass

        sim_log.close()
        nav_log.close()

        subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)
        print("Cleanup done.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Verification runner for Task T1.4 (seabed decal generator).

Launches kinematic Gazebo simulation, navigates vehicle directly over 3 objects
from world_objects.yaml, captures down-looking camera frames at 2.5 m altitude,
and saves them to results/bench/seabed_view_1.png, _2.png, _3.png.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ps11_common.params import load_yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = WORKSPACE_ROOT / "results" / "bench"
WORLD_OBJECTS_YAML = (
    WORKSPACE_ROOT
    / "ros2_ws"
    / "src"
    / "ps11_bringup"
    / "config"
    / "world_objects.yaml"
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle (radians) from quaternion (ENU)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SeabedViewCaptureNode(Node):
    """ROS 2 node that navigates vehicle above target objects and captures images."""

    def __init__(self) -> None:
        super().__init__("seabed_view_capture")
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
                )
            ]
        )

        self.cur_x: float = 0.0
        self.cur_y: float = 0.0
        self.cur_z: float = -12.5
        self.cur_yaw: float = 0.0
        self.has_odom: bool = False

        self.latest_image: np.ndarray | None = None
        self.image_count: int = 0

        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, "/vehicle/cmd_vel", qos_rel)

        self.create_subscription(Odometry, "/sim/gt/odom", self._on_odom, qos_rel)
        # Support both SensorDataQoS (BEST_EFFORT) and Default QoS (RELIABLE)
        self.create_subscription(
            Image,
            "/vehicle/camera/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/vehicle/camera/image_raw",
            self._on_image,
            qos_rel,
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.cur_x = float(p.x)
        self.cur_y = float(p.y)
        self.cur_z = float(p.z)
        self.cur_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.has_odom = True

    def _on_image(self, msg: Image) -> None:
        # Expected rgb8 format
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if len(data) == msg.height * msg.width * 3:
            img_rgb = data.reshape((msg.height, msg.width, 3))
            self.latest_image = img_rgb.copy()
            self.image_count += 1

    def drive_to(
        self,
        target_x: float,
        target_y: float,
        target_z: float = -12.5,
        target_yaw: float = 0.0,
        tolerance: float = 0.35,
        timeout_s: float = 40.0,
    ) -> bool:
        """Drive vehicle to target waypoint using proportional velocity control."""
        t_start = time.time()
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.time() - t_start < timeout_s:
            rclpy.spin_once(self, timeout_sec=dt)
            if not self.has_odom:
                continue

            dx = target_x - self.cur_x
            dy = target_y - self.cur_y
            dz = target_z - self.cur_z
            dist_xy = math.hypot(dx, dy)

            # Error in heading
            dyaw = math.atan2(
                math.sin(target_yaw - self.cur_yaw),
                math.cos(target_yaw - self.cur_yaw),
            )

            if dist_xy < tolerance and abs(dz) < 0.2:
                # Stop vehicle
                self.stop()
                return True

            # Calculate world velocity vector
            speed = min(1.3, max(0.25, 0.9 * dist_xy))
            vx_w = speed * (dx / max(dist_xy, 1e-3))
            vy_w = speed * (dy / max(dist_xy, 1e-3))
            vz_w = max(-0.5, min(0.5, 0.8 * dz))

            # Transform to vehicle body frame (base_link)
            cos_psi = math.cos(self.cur_yaw)
            sin_psi = math.sin(self.cur_yaw)
            vx_b = vx_w * cos_psi + vy_w * sin_psi
            vy_b = -vx_w * sin_psi + vy_w * cos_psi

            wz = max(-0.8, min(0.8, 1.5 * dyaw))

            cmd = Twist()
            cmd.linear.x = float(vx_b)
            cmd.linear.y = float(vy_b)
            cmd.linear.z = float(vz_w)
            cmd.angular.z = float(wz)
            self.cmd_pub.publish(cmd)

        self.stop()
        return False

    def stop(self) -> None:
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture seabed camera views above objects"
    )
    parser.add_argument("--gui", action="store_true", help="Launch Gazebo with GUI")
    parser.add_argument(
        "--config",
        type=str,
        default=str(WORLD_OBJECTS_YAML),
        help="Path to world_objects.yaml",
    )
    args = parser.parse_args()

    obj_file = Path(args.config)
    if not obj_file.exists():
        print(
            f"Error: world_objects.yaml not found at {obj_file}. Run tools/make_seabed.py first."
        )
        return 1

    obj_data = load_yaml(str(obj_file))
    objects = obj_data.get("objects", [])
    if len(objects) < 3:
        print(f"Error: need at least 3 objects in {obj_file}")
        return 1

    # Select 3 distinct objects: 1 debris, 1 starfish, 1 other marine life
    debris_objs = [o for o in objects if o["class"] == "debris"]
    starfish_objs = [o for o in objects if o["class"] == "starfish"]
    other_objs = [o for o in objects if o["class"] not in ("debris", "starfish")]

    targets = [
        debris_objs[0] if debris_objs else objects[0],
        starfish_objs[0] if starfish_objs else objects[1],
        other_objs[0] if other_objs else objects[2],
    ]

    BENCH_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("PS11 AUV — Seabed Decal Verification Runner (T1.4)")
    print("=" * 70)
    for idx, tgt in enumerate(targets, start=1):
        print(
            f"Target {idx}: ID {tgt['id']} | Class: {tgt['class']:10s} | "
            f"Pos: ({tgt['x']:.2f}, {tgt['y']:.2f}, {tgt['z']:.1f}) | Size: {tgt['size_m']:.2f} m"
        )

    # 1. Launch simulation subprocess redirecting to log file to avoid pipe buffer deadlock
    sim_log_path = BENCH_DIR / "sim_seabed.log"
    sim_log = open(sim_log_path, "w")  # noqa: SIM115
    env = os.environ.copy()

    sim_cmd = [
        "ros2",
        "launch",
        "ps11_gazebo",
        "sim.launch.py",
        f"gui:={'true' if args.gui else 'false'}",
        "x:=0.0",
        "y:=0.0",
        "z:=-12.5",
    ]
    print(f"\n[test_seabed_views] Starting simulation: {' '.join(sim_cmd)}")
    sim_proc = subprocess.Popen(
        sim_cmd,
        env=env,
        stdout=sim_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    print("[test_seabed_views] Waiting 5s for Gazebo Harmonic initialization...")
    time.sleep(5.0)

    rclpy.init()
    node = SeabedViewCaptureNode()

    try:
        # Wait for odom and camera stream
        print("[test_seabed_views] Waiting for Gazebo clock, odom, and camera...")
        t_wait = time.time()
        while time.time() - t_wait < 45.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.has_odom and node.latest_image is not None:
                break

        if not node.has_odom:
            print("Error: Timed out waiting for /sim/gt/odom!")
            return 1
        if node.latest_image is None:
            print("Error: Timed out waiting for /vehicle/camera/image_raw!")
            return 1

        print("[test_seabed_views] Simulation ready! Commencing targeted flyovers...")

        # 2. Fly over each of the 3 targets and save camera view
        for idx, tgt in enumerate(targets, start=1):
            ox = float(tgt["x"])
            oy = float(tgt["y"])

            # Camera is pitched 45° down (+X forward).
            # Altitude is 2.5 m (veh_z = -12.5 m, ground_z = -15.0 m).
            # Ground intersection is at x_veh + 2.5 m.
            # To center object in frame: x_veh = ox - 2.5 m, y_veh = oy, yaw = 0.0
            vx_target = ox - 2.5
            vy_target = oy
            vz_target = -12.5

            print(
                f"\n--- Navigating to Object {idx} ({tgt['class']}) at ({ox:.2f}, {oy:.2f}) ---"
            )
            print(
                f"Vehicle waypoint: ({vx_target:.2f}, {vy_target:.2f}, {vz_target:.1f})"
            )

            success = node.drive_to(
                vx_target,
                vy_target,
                vz_target,
                target_yaw=0.0,
                tolerance=0.30,
                timeout_s=40.0,
            )
            if not success:
                print(
                    f"Warning: Could not reach waypoint for Object {idx} within timeout!"
                )

            # Stabilize for 1.0 second and capture fresh frame
            t_settle = time.time()
            while time.time() - t_settle < 1.0:
                rclpy.spin_once(node, timeout_sec=0.1)

            # Grab image
            captured_rgb = node.latest_image
            assert captured_rgb is not None, f"No image received for Object {idx}"

            # Convert RGB to BGR for OpenCV saving
            captured_bgr = cv2.cvtColor(captured_rgb, cv2.COLOR_RGB2BGR)

            out_path = BENCH_DIR / f"seabed_view_{idx}.png"
            cv2.imwrite(str(out_path), captured_bgr)
            print(
                f" Saved camera view: {out_path} ({captured_bgr.shape[1]}x{captured_bgr.shape[0]})"
            )

        print("\n" + "=" * 70)
        print("All 3 seabed views successfully captured:")
        for idx in range(1, 4):
            img_file = BENCH_DIR / f"seabed_view_{idx}.png"
            assert img_file.exists() and img_file.stat().st_size > 5000
            print(f"  - {img_file} ({img_file.stat().st_size} bytes)")
        print("=" * 70)
        return 0

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

        # Terminate sim subprocess
        if sim_proc.poll() is None:
            try:
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
                sim_proc.wait(timeout=5)
            except (ProcessLookupError, PermissionError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        if not sim_log.closed:
            sim_log.close()


if __name__ == "__main__":
    sys.exit(main())

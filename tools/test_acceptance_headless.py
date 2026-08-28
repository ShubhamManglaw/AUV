#!/usr/bin/env python3
"""Automated acceptance test runner for T1.2 running headless for SSH/CI."""

import os
import signal
import subprocess
import time
from pathlib import Path

import cv2
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ps11_common.image_utils import image_to_numpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


class T12AcceptanceNode(Node):
    def __init__(self):
        super().__init__("t1_2_acceptance_verifier")
        self.odom_msgs: list[Odometry] = []
        self.image_msgs: list[Image] = []
        self.depth_msgs: list[Image] = []
        self.imu_msgs: list[Imu] = []

        self.create_subscription(Odometry, "/sim/gt/odom", self._on_odom, 10)
        self.create_subscription(Image, "/vehicle/camera/image_raw", self._on_image, 10)
        self.create_subscription(Image, "/vehicle/camera/depth", self._on_depth, 10)
        self.create_subscription(Imu, "/vehicle/imu", self._on_imu, 10)

        self.cmd_pub = self.create_publisher(Twist, "/vehicle/cmd_vel", 10)

    def _on_odom(self, msg: Odometry):
        self.odom_msgs.append(msg)

    def _on_image(self, msg: Image):
        self.image_msgs.append(msg)

    def _on_depth(self, msg: Image):
        self.depth_msgs.append(msg)

    def _on_imu(self, msg: Imu):
        self.imu_msgs.append(msg)


def main():
    print("==========================================================")
    print("PS11-AUV: T1.2 Acceptance Verification (Headless)")
    print("==========================================================")

    out_dir = Path("results/bench")
    out_dir.mkdir(parents=True, exist_ok=True)
    sim_log = open(out_dir / "sim_launch.log", "w")  # noqa: SIM115

    env = os.environ.copy()
    print("Launching sim.launch.py (mode:=kinematic, gui:=false)...")
    sim_proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ps11_gazebo",
            "sim.launch.py",
            "mode:=kinematic",
            "gui:=false",
        ],
        env=env,
        stdout=sim_log,
        stderr=subprocess.STDOUT,
    )

    print("Waiting 5s for Gazebo to load and vehicle to spawn at (0, 0, -12.5)...")
    time.sleep(5.0)

    try:
        # Check GPU usage
        print("\n--- [1] Checking GPU usage with nvidia-smi ---")
        smi = subprocess.run(
            ["nvidia-smi"],
            capture_output=True,
            text=True,
            check=False,
        )
        print(smi.stdout.strip())

        # Collect sensor messages over 5s
        print("\n--- [2] Measuring Topic Rates and Sensor Outputs (5s window) ---")
        rclpy.init()
        node = T12AcceptanceNode()

        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(node, timeout_sec=0.05)

        img_hz = len(node.image_msgs) / 5.0
        depth_hz = len(node.depth_msgs) / 5.0
        imu_hz = len(node.imu_msgs) / 5.0
        odom_hz = len(node.odom_msgs) / 5.0

        print(f"/vehicle/camera/image_raw rate: {img_hz:.1f} Hz (expected ~10 Hz)")
        print(f"/vehicle/camera/depth rate:     {depth_hz:.1f} Hz (expected ~10 Hz)")
        print(f"/vehicle/imu rate:              {imu_hz:.1f} Hz (expected ~100 Hz)")
        print(f"/sim/gt/odom rate:              {odom_hz:.1f} Hz (expected ~50 Hz)")

        # Save camera snapshot
        if node.image_msgs:
            latest_img = node.image_msgs[-1]
            img_np = image_to_numpy(latest_img)
            bgr_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            cam_path = out_dir / "sim_camera.png"
            cv2.imwrite(str(cam_path), bgr_np)
            print(
                f"\nSaved camera snapshot to {cam_path} ({bgr_np.shape[1]}x{bgr_np.shape[0]})"
            )
        else:
            print("\nWARNING: No image received yet on /vehicle/camera/image_raw")

        # Forward motion test: linear.x = 0.5 for 5 seconds
        print(
            "\n--- [3] Kinematic Motion Verification (Twist linear.x = 0.5 over 5s) ---"
        )
        initial_pos = None
        if node.odom_msgs:
            p = node.odom_msgs[-1].pose.pose.position
            initial_pos = (p.x, p.y, p.z)
            print(
                f"Initial Odometry Pose (/sim/gt/odom): x={p.x:+.3f}, y={p.y:+.3f}, z={p.z:+.3f}"
            )
        else:
            print("Initial Odometry Pose: No messages received yet")

        twist_cmd = Twist()
        twist_cmd.linear.x = 0.5

        drive_start = time.time()
        while time.time() - drive_start < 5.0:
            node.cmd_pub.publish(twist_cmd)
            rclpy.spin_once(node, timeout_sec=0.05)

        # Stop vehicle
        node.cmd_pub.publish(Twist())
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        final_pos = None
        if node.odom_msgs:
            p = node.odom_msgs[-1].pose.pose.position
            final_pos = (p.x, p.y, p.z)
            print(
                f"Final Odometry Pose (/sim/gt/odom):   x={p.x:+.3f}, y={p.y:+.3f}, z={p.z:+.3f}"
            )

        if initial_pos and final_pos:
            dx = final_pos[0] - initial_pos[0]
            print(
                f"Forward Displacement: dx = {dx:+.3f} m (expected ~ +2.5 m at 0.5 m/s for 5s)"
            )

        node.destroy_node()
        rclpy.shutdown()

    finally:
        print("\nShutting down simulation...")
        sim_proc.send_signal(signal.SIGINT)
        try:
            sim_proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            sim_proc.kill()
        sim_log.close()
        print("Simulation stopped cleanly.")


if __name__ == "__main__":
    main()

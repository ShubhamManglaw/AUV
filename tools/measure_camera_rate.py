#!/usr/bin/env python3
"""Diagnostic script to measure camera and sensor publication rates (§8.3, T1.2)."""

import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu


class RateDiagnosticNode(Node):
    def __init__(self):
        super().__init__("rate_diagnostic_node")
        self.image_count = 0
        self.depth_count = 0
        self.info_count = 0
        self.imu_count = 0
        self.odom_count = 0

        self.create_subscription(
            Image,
            "/vehicle/camera/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/vehicle/camera/depth",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/vehicle/camera/camera_info",
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/vehicle/imu",
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/sim/gt/odom",
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _on_image(self, msg: Image):
        self.image_count += 1

    def _on_depth(self, msg: Image):
        self.depth_count += 1

    def _on_info(self, msg: CameraInfo):
        self.info_count += 1

    def _on_imu(self, msg: Imu):
        self.imu_count += 1

    def _on_odom(self, msg: Odometry):
        self.odom_count += 1


def main():
    # 0. Clean up any stale processes before starting
    subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)

    print("==========================================================")
    print("PS11-AUV: Camera Rate Diagnostic (SensorDataQoS over 10s)")
    print("==========================================================")

    env = os.environ.copy()
    sim_log = open("/tmp/sim_diagnostic.log", "w")

    # Start ros2 launch in a new process group
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
        start_new_session=True,
    )

    print("Waiting 6s for simulation to initialize and vehicle to spawn...")
    time.sleep(6.0)

    try:
        # Measure using rclpy node with SensorDataQoS
        rclpy.init()
        node = RateDiagnosticNode()

        print("Counting messages over 10.0 seconds window...")
        duration = 10.0
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.05)

        elapsed = time.time() - start

        img_rate = node.image_count / elapsed
        depth_rate = node.depth_count / elapsed
        info_rate = node.info_count / elapsed
        imu_rate = node.imu_count / elapsed
        odom_rate = node.odom_count / elapsed

        print("\n--- Diagnostic Results (rclpy SensorDataQoS) ---")
        print(f"Elapsed time:                 {elapsed:.2f} s")
        print(
            f"/vehicle/camera/image_raw:    {node.image_count} msgs -> {img_rate:.2f} Hz"
        )
        print(
            f"/vehicle/camera/depth:        {node.depth_count} msgs -> {depth_rate:.2f} Hz"
        )
        print(
            f"/vehicle/camera/camera_info:  {node.info_count} msgs -> {info_rate:.2f} Hz"
        )
        print(
            f"/vehicle/imu:                 {node.imu_count} msgs -> {imu_rate:.2f} Hz"
        )
        print(
            f"/sim/gt/odom:                 {node.odom_count} msgs -> {odom_rate:.2f} Hz"
        )

        # Also measure ros2 topic info for publishers
        for top in [
            "/vehicle/camera/image_raw",
            "/vehicle/camera/depth",
            "/vehicle/imu",
            "/sim/gt/odom",
        ]:
            info_res = subprocess.run(
                ["ros2", "topic", "info", top],
                capture_output=True,
                text=True,
                check=False,
            )
            print(f"\n--- {top} topic info ---")
            print(info_res.stdout.strip())

        node.destroy_node()
        rclpy.shutdown()

    finally:
        print("\nTerminating simulation process group...")
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

        # Wait up to 10s
        deadline = time.time() + 10.0
        while time.time() < deadline and sim_proc.poll() is None:
            time.sleep(0.5)

        if sim_proc.poll() is None:
            try:
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
                time.sleep(2.0)
            except ProcessLookupError:
                pass

        if sim_proc.poll() is None:
            try:
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
                time.sleep(1.0)
            except ProcessLookupError:
                pass

        sim_log.close()
        subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)
        print("Done.")


if __name__ == "__main__":
    main()

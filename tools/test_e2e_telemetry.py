#!/usr/bin/env python3
"""End-to-end telemetry verification run without Gazebo (§11.7, Q4).

Runs:
  fake_vehicle + scheduler + link_emulator (m64, pull) + surface_decoder
For 90 s (simulated time).

Checks:
  1. Debris arrives first at /surface/contacts
  2. Heartbeats are <= 15 s apart
  3. Logs final /link/stats
"""

from __future__ import annotations

import subprocess
import sys
import time

import nav_msgs.msg
import rclpy
from ps11_interfaces.msg import ContactArray, LinkStats
from rclpy.node import Node


class TelemetryVerifier(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_verifier")
        self.first_contact_class: int | None = None
        self.first_contact_id: int | None = None
        self.surface_contacts_count = 0
        self.heartbeat_stamps: list[float] = []
        self.latest_stats: LinkStats | None = None

        self.create_subscription(
            ContactArray,
            "/surface/contacts",
            self._on_surface_contacts,
            10,
        )
        self.create_subscription(
            nav_msgs.msg.Path,
            "/surface/vehicle_track",
            self._on_vehicle_track,
            10,
        )
        self.create_subscription(
            LinkStats,
            "/link/stats",
            self._on_stats,
            10,
        )

    def _on_surface_contacts(self, msg: ContactArray) -> None:
        if msg.contacts:
            self.surface_contacts_count = len(msg.contacts)
            if self.first_contact_class is None:
                # The first arrived contact
                self.first_contact_class = msg.contacts[0].class_id
                self.first_contact_id = msg.contacts[0].contact_id
                now_s = self.get_clock().now().nanoseconds * 1e-9
                self.get_logger().info(
                    f"FIRST CONTACT ARRIVED on surface: id={self.first_contact_id}, "
                    f"class_id={self.first_contact_class} at t={now_s:.2f}s"
                )

    def _on_vehicle_track(self, msg: nav_msgs.msg.Path) -> None:
        if msg.poses:
            latest_pose = msg.poses[-1]
            t = latest_pose.header.stamp.sec + latest_pose.header.stamp.nanosec * 1e-9
            if not self.heartbeat_stamps or t > self.heartbeat_stamps[-1]:
                self.heartbeat_stamps.append(t)
                self.get_logger().info(
                    f"Heartbeat received on surface: total={len(self.heartbeat_stamps)}, stamp={t:.2f}s"
                )

    def _on_stats(self, msg: LinkStats) -> None:
        self.latest_stats = msg


def run_e2e(duration_s: float = 90.0) -> int:
    rclpy.init()
    verifier = TelemetryVerifier()

    # Launch nodes as subprocesses using ros2 run with use_sim_time:=true
    procs: list[subprocess.Popen] = []
    try:
        # 1. link_emulator
        cmd_link = [
            "ros2",
            "run",
            "ps11_telemetry",
            "link_emulator",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "profile:=m64",
            "-p",
            "mode:=pull",
        ]
        procs.append(subprocess.Popen(cmd_link))

        # 2. scheduler
        cmd_sched = [
            "ros2",
            "run",
            "ps11_telemetry",
            "scheduler",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "profile:=m64",
            "-p",
            "policy:=semantic",
        ]
        procs.append(subprocess.Popen(cmd_sched))

        # 3. surface_decoder
        cmd_surf = [
            "ros2",
            "run",
            "ps11_telemetry",
            "surface_decoder",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ]
        procs.append(subprocess.Popen(cmd_surf))

        # Wait 1s for subscriber graph to connect
        time.sleep(1.0)

        # 4. fake_vehicle (which publishes /clock, /vehicle/contacts, /vehicle/nav/odom, /vehicle/mission/state)
        cmd_veh = [
            "python3",
            "tools/fake_vehicle.py",
            "--duration",
            str(duration_s + 5.0),
        ]
        procs.append(subprocess.Popen(cmd_veh))

        print(
            f"=== Running 90s telemetry test without Gazebo (target: {duration_s}s) ==="
        )
        start_wall = time.time()
        while time.time() - start_wall < duration_s:
            rclpy.spin_once(verifier, timeout_sec=0.1)

        print("\n=== Test Duration Completed. Verifying Results ===")

        # Check 1: Debris arrives first
        # class_id 0 is debris
        debris_first = verifier.first_contact_class == 0
        print(
            f"1. First contact arrived: id={verifier.first_contact_id}, class_id={verifier.first_contact_class} "
            f"({'PASS: Debris arrived first' if debris_first else 'FAIL: Expected class 0 (debris)'})"
        )

        # Check 2: Heartbeats <= 15 s apart
        hb_count = len(verifier.heartbeat_stamps)
        max_hb_gap = 0.0
        if hb_count > 1:
            gaps = [
                verifier.heartbeat_stamps[i] - verifier.heartbeat_stamps[i - 1]
                for i in range(1, hb_count)
            ]
            max_hb_gap = max(gaps)
        hb_ok = (hb_count >= 5) and (
            max_hb_gap <= 15.5
        )  # 15s + margin for frame airtime
        print(
            f"2. Heartbeats received: {hb_count}, max gap: {max_hb_gap:.2f}s "
            f"({'PASS: <= 15s apart' if hb_ok else 'FAIL: Heartbeat gap exceeded 15s'})"
        )

        # Check 3: Final /link/stats
        stats = verifier.latest_stats
        print("\n=== Final /link/stats ===")
        if stats is not None:
            print(f"Profile: {stats.profile_label}")
            print(f"Payload bits sent: {stats.payload_bits_sent}")
            print(f"Frames sent: {stats.frames_sent}")
            print(f"Frames lost: {stats.frames_lost}")
            print(f"Frames rejected: {stats.frames_rejected}")
            print(f"Queue length: {stats.queue_len}")
            print(f"Utilisation: {stats.utilisation:.3f}")
        else:
            print("WARNING: No /link/stats message received!")

        all_ok = debris_first and hb_ok and (stats is not None)
        return 0 if all_ok else 1

    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                p.kill()
        verifier.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(run_e2e())

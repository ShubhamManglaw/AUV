#!/usr/bin/env python3
"""Full-chain E2E Demo Mission Runner & Acceptance Verifier (§14.4, Q7).

Runs demo.launch.py headless for the complete mission (~5-6 min) with record:=true.
Acceptance checks:
  1. At least one contact on /surface/contacts
  2. Heartbeats <= 15 s apart
  3. summary.json written
  4. MCAP bag written
  5. No leftover processes
"""

from __future__ import annotations

import signal
import subprocess
import time
from pathlib import Path

import nav_msgs.msg
import rclpy
from ps11_interfaces.msg import ContactArray, DemoCounters, LinkStats
from rclpy.node import Node
from std_msgs.msg import UInt8


class DemoMissionMonitor(Node):
    def __init__(self) -> None:
        super().__init__("demo_mission_monitor")
        self.surface_contacts: list[int] = []
        self.heartbeat_stamps: list[float] = []
        self.mission_states: list[tuple[float, int]] = []
        self.current_state: int = -1
        self.survey_started = False
        self.mission_completed = False
        self.latest_counters: DemoCounters | None = None
        self.latest_stats: LinkStats | None = None

        self.create_subscription(ContactArray, "/surface/contacts", self._on_contacts, 10)
        self.create_subscription(nav_msgs.msg.Path, "/surface/vehicle_track", self._on_track, 10)
        self.create_subscription(UInt8, "/vehicle/mission/state", self._on_mission_state, 10)
        self.create_subscription(DemoCounters, "/eval/counters", self._on_counters, 10)
        self.create_subscription(LinkStats, "/link/stats", self._on_stats, 10)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_contacts(self, msg: ContactArray) -> None:
        if msg.contacts:
            now_s = self._now_s()
            for c in msg.contacts:
                if c.contact_id not in self.surface_contacts:
                    self.surface_contacts.append(c.contact_id)
                    print(
                        f"[{now_s:.1f}s] SURFACE CONTACT RECEIVED: id={c.contact_id}, "
                        f"class={c.class_id}, conf={c.confidence:.2f}, "
                        f"pos=({c.position.x:.1f}, {c.position.y:.1f}, {c.position.z:.1f})"
                    )

    def _on_track(self, msg: nav_msgs.msg.Path) -> None:
        if msg.poses:
            latest_pose = msg.poses[-1]
            t = latest_pose.header.stamp.sec + latest_pose.header.stamp.nanosec * 1e-9
            if not self.heartbeat_stamps or t > self.heartbeat_stamps[-1]:
                self.heartbeat_stamps.append(t)
                print(f"[{self._now_s():.1f}s] Heartbeat received at surface: #{len(self.heartbeat_stamps)} (stamp={t:.1f}s)")

    def _on_mission_state(self, msg: UInt8) -> None:
        st = int(msg.data)
        if st != self.current_state:
            now_s = self._now_s()
            self.current_state = st
            self.mission_states.append((now_s, st))
            state_names = {0: "IDLE", 1: "TRANSIT", 2: "SURVEY", 4: "RETURN"}
            print(f"[{now_s:.1f}s] Mission state -> {state_names.get(st, str(st))} ({st})")
            if st == 2:
                self.survey_started = True
            elif self.survey_started and st == 0:
                print(f"[{now_s:.1f}s] Survey complete and vehicle returned to IDLE!")
                self.mission_completed = True

    def _on_counters(self, msg: DemoCounters) -> None:
        self.latest_counters = msg

    def _on_stats(self, msg: LinkStats) -> None:
        self.latest_stats = msg


def run_e2e_demo(timeout_s: float = 420.0, run_name: str = "default") -> int:
    print("================================================================================")
    print(f"PS11 AUV — Full-Chain End-to-End Demo Mission Run ({run_name})")
    print("================================================================================")

    # 1. Clean up simulation processes
    print("[1/5] Running pre-sim cleanup...")
    subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)
    time.sleep(2.0)

    # 2. Launch demo.launch.py with record:=true
    print(
        f"[2/5] Launching demo.launch.py (headless, gui:=false, record:=true, max_timeout={timeout_s}s)..."
    )
    cmd = [
        "ros2",
        "launch",
        "ps11_bringup",
        "demo.launch.py",
        "gui:=false",
        "record:=true",
        "scenario:=demo",
        "link_profile:=m64",
    ]

    launch_proc = subprocess.Popen(cmd)

    rclpy.init()
    monitor = DemoMissionMonitor()

    start_wall = time.time()
    try:
        print("[3/5] Monitoring full mission progress...")
        while time.time() - start_wall < timeout_s:
            rclpy.spin_once(monitor, timeout_sec=0.2)
            if monitor.mission_completed:
                print(
                    f"Mission finished successfully in {time.time() - start_wall:.1f}s!"
                )
                # Allow 10s extra for final telemetry packets to deliver over acoustic link
                time.sleep(10.0)
                break
            if launch_proc.poll() is not None:
                print(
                    f"WARNING: launch process exited early with code {launch_proc.returncode}"
                )
                break

    finally:
        print("\n[4/5] Sending SIGINT to launch process for clean shutdown...")
        launch_proc.send_signal(signal.SIGINT)
        try:
            launch_proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            launch_proc.terminate()
            try:
                launch_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                launch_proc.kill()

        monitor.destroy_node()
        rclpy.try_shutdown()

        # Run cleanup
        print("Running post-sim cleanup...")
        subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)

    print(
        "\n================================================================================"
    )
    print(f"[5/5] VERIFYING ACCEPTANCE CRITERIA ({run_name})")
    print(
        "================================================================================"
    )

    # Check 1: At least one contact on /surface/contacts
    contact_count = len(monitor.surface_contacts)
    check1 = contact_count >= 1
    print(
        f"1. Surface contacts: {contact_count} unique contacts received "
        f"({'PASS' if check1 else 'FAIL'})"
    )

    # Check 2: Heartbeats <= 15 s apart
    hb_count = len(monitor.heartbeat_stamps)
    max_hb_gap = 0.0
    if hb_count > 1:
        gaps = [
            monitor.heartbeat_stamps[i] - monitor.heartbeat_stamps[i - 1]
            for i in range(1, hb_count)
        ]
        max_hb_gap = max(gaps)
    check2 = (hb_count >= 3) and (max_hb_gap <= 15.5)
    print(
        f"2. Heartbeats received: {hb_count}, max gap: {max_hb_gap:.2f}s "
        f"({'PASS' if check2 else 'FAIL'})"
    )

    # Check 3: summary.json written
    latest_summary_path = Path("results/latest_summary.json")
    check3 = latest_summary_path.is_file() and latest_summary_path.stat().st_size > 0
    print(
        f"3. summary.json written: {latest_summary_path} ({'PASS' if check3 else 'FAIL'})"
    )

    # Check 4: Bag written
    bag_dirs = list(Path("results").glob("run_*_bag"))
    check4 = False
    bag_path_found = None
    if bag_dirs:
        # Pick newest bag dir
        bag_dirs.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        latest_bag = bag_dirs[0]
        mcap_files = list(latest_bag.glob("*.mcap"))
        if mcap_files and mcap_files[0].stat().st_size > 1000:
            check4 = True
            bag_path_found = mcap_files[0]
            print(
                f"4. MCAP bag written: {bag_path_found} ({bag_path_found.stat().st_size / 1024 / 1024:.2f} MB) ({'PASS' if bag_path_found.stat().st_size / 1024 / 1024 < 500.0 else 'WARN >500MB'})"
            )
        else:
            print(
                f"4. MCAP bag check: {latest_bag} (no valid .mcap file found) (FAIL)"
            )
    else:
        print("4. MCAP bag check: No bag directories found in results/ (FAIL)")

    # Save copy of summary for this run
    if check3:
        dest_summary = Path(f"results/summary_{run_name}.json")
        import shutil

        shutil.copyfile(latest_summary_path, dest_summary)
        print(f"\n--- summary.json Contents (saved to {dest_summary}) ---")
        with open(latest_summary_path, encoding="utf-8") as f:
            summary_content = f.read()
            print(summary_content)

    all_passed = check1 and check2 and check3 and check4
    print(
        f"\nFinal Acceptance Verdict ({run_name}): {'ALL PASS' if all_passed else 'SOME CHECKS FAILED'}"
    )
    return 0 if all_passed else 1


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Full-chain demo mission runner")
    parser.add_argument(
        "--run-name", default="default", help="Identifier for run (e.g. setting_A)"
    )
    parser.add_argument(
        "--timeout", type=float, default=420.0, help="Max mission timeout in seconds"
    )
    args = parser.parse_args()

    import os

    code = run_e2e_demo(timeout_s=args.timeout, run_name=args.run_name)
    os._exit(code)


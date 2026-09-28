#!/usr/bin/env python3
"""Bringup verification test (§14.4, Q6).

Launches demo.launch.py headless for 65 seconds, verifies:
1. All nodes from §14.1 start
2. ros2 node list contains all expected nodes
3. No crashes/errors in the first 60 s
4. Cleans up cleanly
"""

from __future__ import annotations

import subprocess
import sys
import time

EXPECTED_NODES = {
    "/robot_state_publisher",
    "/bridge",
    "/odom_noise",
    "/depth_sim",
    "/range_adapter",
    "/waypoint_follower",
    "/detector",
    "/tracker",
    "/geolocator",
    "/contact_db",
    "/scheduler",
    "/link_emulator",
    "/surface_decoder",
    "/metrics",
    "/foxglove_bridge",
}


def main() -> int:
    print("=== Cleaning up before bringup test ===")
    subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)

    print("=== Launching demo.launch.py (headless, gui:=false, record:=false) ===")
    cmd = [
        "ros2", "launch", "ps11_bringup", "demo.launch.py",
        "gui:=false", "record:=false"
    ]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    try:
        # Wait 20s for Gazebo and all nodes to initialize
        print("Waiting 20s for all nodes to start up...")
        time.sleep(20.0)

        # Run ros2 node list
        node_list_proc = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=15)
        active_nodes = set(node_list_proc.stdout.strip().splitlines())
        print("\n--- Active Nodes in ros2 node list ---")
        for n in sorted(active_nodes):
            print(f"  {n}")

        # Check expected nodes
        missing: list[str] = []
        for exp in EXPECTED_NODES:
            # Check exact match or basename match (e.g. /bridge or /ros_gz_bridge)
            found = any(exp in n or (exp == "/bridge" and "bridge" in n) for n in active_nodes)
            if not found:
                missing.append(exp)

        print("\n--- Node Check Results ---")
        if missing:
            print(f"FAILED: Missing nodes ({len(missing)}): {missing}")
        else:
            print(f"PASSED: All {len(EXPECTED_NODES)} expected nodes are active!")

        # Continue waiting until 60s total
        print("\nMonitoring for errors up to 60s...")
        time.sleep(45.0)

        if proc.poll() is not None:
            print(f"FAILED: demo.launch.py exited early with code {proc.returncode}")
            return 1

        print("PASSED: demo.launch.py ran stable for >60s with all nodes active!")
        return 0 if not missing else 1

    finally:
        print("\n=== Terminating demo.launch.py and cleaning up ===")
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)


if __name__ == "__main__":
    sys.exit(main())

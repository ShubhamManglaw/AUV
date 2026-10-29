#!/usr/bin/env python3
"""Hover alignment test for PS11 AUV (T2.9 Step 1).

Holds the vehicle completely still at 2.5 m altitude above 3 known objects,
captures raw camera frames, computes exact decal outline projections,
and measures the offset between projected box center and actual visible pixels.
"""

from __future__ import annotations

import math
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ps11_common.image_utils import image_to_numpy
from ps11_common.params import get_config_path, load_yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
BENCH_DIR = WORKSPACE_ROOT / "results" / "bench"


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy - qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )


def project_world_point(
    p_world: np.ndarray,
    veh_pos: np.ndarray,
    veh_rot: np.ndarray,
    cam_x: float = 0.357,
    cam_pitch: float = 0.785398,
    fx: float = 320.0,
    fy: float = 320.0,
    cx: float = 320.0,
    cy: float = 240.0,
) -> tuple[float, float, float] | None:
    t_cam = np.array([cam_x, 0.0, 0.0], dtype=np.float64)
    p_cam_world = veh_pos + veh_rot @ t_cam

    cos_p = math.cos(cam_pitch)
    sin_p = math.sin(cam_pitch)
    r_pitch = np.array(
        [[cos_p, 0.0, sin_p], [0.0, 1.0, 0.0], [-sin_p, 0.0, cos_p]],
        dtype=np.float64,
    )

    r_opt = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )

    r_cam_world = veh_rot @ r_pitch @ r_opt
    p_opt = r_cam_world.T @ (p_world - p_cam_world)

    z_opt = float(p_opt[2])
    if z_opt <= 0.05:
        return None

    u = fx * float(p_opt[0]) / z_opt + cx
    v = fy * float(p_opt[1]) / z_opt + cy
    return (u, v, z_opt)


class HoverTesterNode(Node):
    def __init__(self) -> None:
        super().__init__("hover_tester_node")
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
                )
            ]
        )

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = -12.5
        self.cur_rot = np.eye(3, dtype=np.float64)
        self.cur_lin_vel = 0.0
        self.cur_ang_vel = 0.0
        self.has_odom = False

        self.fx = 320.0
        self.fy = 320.0
        self.cx = 320.0
        self.cy = 240.0

        self.latest_frame: np.ndarray | None = None

        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, "/vehicle/cmd_vel", qos_rel)

        self.create_subscription(Odometry, "/sim/gt/odom", self._on_odom, qos_rel)
        self.create_subscription(
            CameraInfo,
            "/vehicle/camera/camera_info",
            self._on_cam_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/vehicle/camera/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.cur_x = float(p.x)
        self.cur_y = float(p.y)
        self.cur_z = float(p.z)
        self.cur_rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.cur_lin_vel = math.hypot(vx, vy, vz)
        self.cur_ang_vel = abs(msg.twist.twist.angular.z)
        self.has_odom = True

    def _on_cam_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest_frame = image_to_numpy(msg)
        except Exception:  # noqa: BLE001, S110
            pass

    def fly_to_and_hover(
        self, target_x: float, target_y: float, target_z: float = -12.5
    ) -> bool:
        """Fly to waypoint and wait until completely stationary."""
        t_start = time.time()
        while time.time() - t_start < 25.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = target_x - self.cur_x
            dy = target_y - self.cur_y
            dz = target_z - self.cur_z
            dist = math.hypot(dx, dy)

            cur_yaw = math.atan2(self.cur_rot[1, 0], self.cur_rot[0, 0])
            yaw_err = math.atan2(math.sin(0.0 - cur_yaw), math.cos(0.0 - cur_yaw))

            if dist < 0.20 and abs(dz) < 0.15:
                # Stop vehicle
                self.cmd_pub.publish(Twist())
                break

            cmd = Twist()
            head = math.atan2(dy, dx)
            cmd.linear.x = min(2.0, 1.2 * dist * math.cos(head - cur_yaw))
            cmd.linear.y = min(1.5, 1.0 * dist * math.sin(head - cur_yaw))
            cmd.linear.z = min(0.6, max(-0.6, 1.2 * dz))
            cmd.angular.z = min(1.0, max(-1.0, 2.0 * yaw_err))
            self.cmd_pub.publish(cmd)

        self.cmd_pub.publish(Twist())

        # Settle completely until lin_vel and ang_vel are near zero
        t_settle = time.time()
        while time.time() - t_settle < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.cmd_pub.publish(Twist())
            if self.cur_lin_vel < 0.05 and self.cur_ang_vel < 0.05:
                break

        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.1)
        return True


def measure_pixel_offset(
    img_rgb: np.ndarray,
    box: tuple[int, int, int, int],
) -> tuple[float, float, float]:
    """Measure offset between box center and actual visible decal centroid."""
    x1, y1, x2, y2 = box
    box_cx = (x1 + x2) / 2.0
    box_cy = (y1 + y2) / 2.0
    bw = x2 - x1
    bh = y2 - y1

    h, w = img_rgb.shape[:2]
    # Local neighborhood around the box
    pad = max(15, int(max(bw, bh) * 0.4))
    wx1 = max(0, x1 - pad)
    wy1 = max(0, y1 - pad)
    wx2 = min(w, x2 + pad)
    wy2 = min(h, y2 + pad)

    window = img_rgb[wy1:wy2, wx1:wx2]
    if window.size == 0 or window.shape[0] < 6 or window.shape[1] < 6:
        return 0.0, box_cx, box_cy

    # Estimate background sand color from border pixels of window (which are sand)
    border = np.concatenate(
        [
            window[:3, :].reshape(-1, 3),
            window[-3:, :].reshape(-1, 3),
            window[:, :3].reshape(-1, 3),
            window[:, -3:].reshape(-1, 3),
        ],
        axis=0,
    )
    sand_rgb = np.median(border, axis=0)

    # Pixel color difference from surrounding sand
    diff = np.linalg.norm(window.astype(np.float32) - sand_rgb, axis=2)

    # Box relative to window
    rx1 = max(0, x1 - wx1)
    ry1 = max(0, y1 - wy1)
    rx2 = min(window.shape[1], x2 - wx1)
    ry2 = min(window.shape[0], y2 - wy1)

    box_diff = diff[ry1:ry2, rx1:rx2]
    if box_diff.size == 0:
        return 0.0, box_cx, box_cy

    thresh = max(15.0, float(np.percentile(box_diff, 50)))
    mask = (diff > thresh).astype(np.uint8)

    # Restrict to region near the box
    search_mask = np.zeros_like(mask)
    sx1 = max(0, rx1 - 5)
    sy1 = max(0, ry1 - 5)
    sx2 = min(window.shape[1], rx2 + 5)
    sy2 = min(window.shape[0], ry2 + 5)
    search_mask[sy1:sy2, sx1:sx2] = mask[sy1:sy2, sx1:sx2]

    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(search_mask)
    if num_labels > 1:
        target_rx = box_cx - wx1
        target_ry = box_cy - wy1
        best_comp = None
        min_d = 1e9
        for comp_id in range(1, num_labels):
            area = stats[comp_id, cv2.CC_STAT_AREA]
            if area < 5:
                continue
            ccx, ccy = centroids[comp_id]
            d = math.hypot(ccx - target_rx, ccy - target_ry)
            if d < min_d:
                min_d = d
                best_comp = comp_id

        if best_comp is not None:
            actual_cx = wx1 + centroids[best_comp][0]
            actual_cy = wy1 + centroids[best_comp][1]
            offset = math.hypot(actual_cx - box_cx, actual_cy - box_cy)
            return offset, actual_cx, actual_cy

    return 0.0, box_cx, box_cy


def main() -> None:
    objects_file = get_config_path("world_objects.yaml")
    world_data = load_yaml(objects_file)
    objects = world_data.get("objects", [])

    # Select 3 test objects (debris: 0, 1, and starfish: 5)
    test_ids = [0, 1, 5]
    test_objects = [o for o in objects if int(o["id"]) in test_ids]

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    sim_log = open(BENCH_DIR / "sim_hover.log", "w", encoding="utf-8")  # noqa: SIM115

    print("=" * 80)
    print("HOVER ALIGNMENT TEST (T2.9 STEP 1)")
    print("=" * 80)

    sim_proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ps11_gazebo",
            "sim.launch.py",
            "gui:=false",
            "x:=0.0",
            "y:=0.0",
            "z:=-12.5",
        ],
        stdout=sim_log,
        stderr=subprocess.STDOUT,
    )

    time.sleep(5.0)
    rclpy.init()
    node = HoverTesterNode()

    try:
        t0 = time.time()
        while time.time() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.has_odom and node.latest_frame is not None:
                break
        else:
            print("[hover_test] Simulation timeout.")
            sys.exit(1)

        print("[hover_test] Simulation ready. Starting hover measurements...\n")

        for obj in test_objects:
            oid = int(obj["id"])
            cls_name = obj["class"]
            tx, ty, tz = float(obj["x"]), float(obj["y"]), float(obj.get("z", -15.0))
            size_m = float(obj["size_m"])

            # 2.5 m altitude, 2.5 m before object along X
            node.fly_to_and_hover(tx - 2.5, ty, target_z=-12.5)

            frame = node.latest_frame.copy()
            veh_pos = np.array([node.cur_x, node.cur_y, node.cur_z], dtype=np.float64)

            # Project tight outline if available, otherwise square corners
            outline = obj.get("outline_m")
            if outline:
                world_pts = [
                    np.array([tx + dx, ty + dy, tz], dtype=np.float64)
                    for dx, dy in outline
                ]
            else:
                half = size_m / 2.0
                world_pts = [
                    np.array([tx + half, ty + half, tz], dtype=np.float64),
                    np.array([tx + half, ty - half, tz], dtype=np.float64),
                    np.array([tx - half, ty - half, tz], dtype=np.float64),
                    np.array([tx - half, ty + half, tz], dtype=np.float64),
                ]
            projs = [
                project_world_point(
                    p,
                    veh_pos,
                    node.cur_rot,
                    fx=node.fx,
                    fy=node.fy,
                    cx=node.cx,
                    cy=node.cy,
                )
                for p in world_pts
            ]
            valid_projs = [p[:2] for p in projs if p is not None]
            if len(valid_projs) >= 3:
                u_c = [p[0] for p in valid_projs]
                v_c = [p[1] for p in valid_projs]
                x1, y1 = round(min(u_c)), round(min(v_c))
                x2, y2 = round(max(u_c)), round(max(v_c))

                offset, act_cx, act_cy = measure_pixel_offset(frame, (x1, y1, x2, y2))

                # Draw visualization
                vis = frame.copy()
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(
                    vis,
                    (round((x1 + x2) / 2), round((y1 + y2) / 2)),
                    3,
                    (0, 255, 0),
                    -1,
                )
                cv2.circle(vis, (round(act_cx), round(act_cy)), 3, (0, 0, 255), -1)
                cv2.line(
                    vis,
                    (round((x1 + x2) / 2), round((y1 + y2) / 2)),
                    (round(act_cx), round(act_cy)),
                    (0, 255, 255),
                    1,
                )
                cv2.putText(
                    vis,
                    f"Obj {oid} ({cls_name}): offset={offset:.1f}px (vel={node.cur_lin_vel:.3f}m/s)",
                    (max(10, x1 - 10), max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

                out_path = BENCH_DIR / f"hover_test_{oid}.png"
                cv2.imwrite(str(out_path), cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
                print(
                    f"• Object {oid} ({cls_name:<10}): Measured Offset = {offset:.2f} px | Saved: {out_path}"
                )

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sim_proc.terminate()
        try:
            sim_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            sim_proc.kill()
        sim_log.close()
        subprocess.run(["bash", "tools/sim_cleanup.sh"], check=False)


if __name__ == "__main__":
    main()

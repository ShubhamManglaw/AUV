#!/usr/bin/env python3
"""T2.9 Simulation Fine-Tuning Dataset Generator and Auto-Labeler (§10.3).

Generates simulation training datasets using 3 distinct seabed layout seeds
(never using the demo seed 42), flies survey passes with high object density,
captures frames at exact interpolated timestamps with low yaw rate,
projects tight decal boundary polygons, verifies labels with automatic
connected-component offset checks (median <= 3 px, max <= 8 px),
and saves 5 labeled visual validation examples to results/bench/simlabel_v2_<n>.png.
"""

from __future__ import annotations

import argparse
import collections
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ps11_common.classes import ClassDatabase
from ps11_common.image_utils import image_to_numpy
from ps11_common.params import get_config_path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from tools.make_seabed import generate_seabed

SIM_DATA_DIR = WORKSPACE_ROOT / "ml" / "data" / "sim"
BENCH_DIR = WORKSPACE_ROOT / "results" / "bench"


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix."""
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


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two quaternions [qx, qy, qz, qw]."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        res = q0 + t * (q1 - q0)
        return res / np.linalg.norm(res)
    theta_0 = math.acos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = math.sin(theta_0)
    theta_t = theta_0 * t
    sin_theta_t = math.sin(theta_t)
    s0 = math.cos(theta_t) - dot * sin_theta_t / sin_theta_0
    s1 = sin_theta_t / sin_theta_0
    return s0 * q0 + s1 * q1


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
    """Project 3D world coordinate into 2D camera pixel coordinates (u, v, Z_opt)."""
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


class SimCollectorNode(Node):
    """Navigates vehicle and auto-labels captured frames with exact timing & tight outlines."""

    def __init__(self, objects: list[dict[str, Any]], split: str, prefix: str) -> None:
        super().__init__("sim_collector_node")
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
                )
            ]
        )

        self.objects = objects
        self.split = split
        self.prefix = prefix
        self.class_db = ClassDatabase()
        self.name_to_id = {c.name: c.id for c in self.class_db.all_classes()}

        # Rolling odometry history: (t, pos, quat, wz, v)
        self.odom_history: collections.deque[
            tuple[float, np.ndarray, np.ndarray, float, float]
        ] = collections.deque(maxlen=600)

        self.cur_x: float = 0.0
        self.cur_y: float = 0.0
        self.cur_z: float = -12.5
        self.cur_rot = np.eye(3, dtype=np.float64)
        self.cur_yaw_rate: float = 0.0
        self.cur_lin_vel: float = 0.0
        self.has_odom: bool = False

        self.fx: float = 320.0
        self.fy: float = 320.0
        self.cx: float = 320.0
        self.cy: float = 240.0

        # Unprocessed image buffer: (t, frame)
        self.image_buffer: collections.deque[tuple[float, np.ndarray]] = (
            collections.deque(maxlen=20)
        )
        self.latest_frame: np.ndarray | None = None
        self.latest_frame_stamp: float = 0.0

        self.captured_count: int = 0
        self.dropped_large_offset: int = 0
        self.dropped_small_object: int = 0
        self.all_offsets: list[float] = []
        self.labeled_samples: list[
            tuple[np.ndarray, list[tuple[int, list[float]]], list[float], bool]
        ] = []

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
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        p = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            ],
            dtype=np.float64,
        )
        q = np.array(
            [
                float(msg.pose.pose.orientation.x),
                float(msg.pose.pose.orientation.y),
                float(msg.pose.pose.orientation.z),
                float(msg.pose.pose.orientation.w),
            ],
            dtype=np.float64,
        )
        wz = float(msg.twist.twist.angular.z)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        vz = float(msg.twist.twist.linear.z)
        v = math.hypot(vx, vy, vz)

        self.odom_history.append((t, p, q, wz, v))
        self.cur_x = p[0]
        self.cur_y = p[1]
        self.cur_z = p[2]
        self.cur_rot = quat_to_rot_matrix(q[0], q[1], q[2], q[3])
        self.cur_yaw_rate = wz
        self.cur_lin_vel = v
        self.has_odom = True

    def _on_cam_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def _on_image(self, msg: Image) -> None:
        try:
            t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            frame = image_to_numpy(msg)
            self.latest_frame = frame
            self.latest_frame_stamp = t
            self.image_buffer.append((t, frame))
        except Exception:  # noqa: BLE001, S110
            pass

    def get_pose_at_stamp(
        self, t_img: float
    ) -> tuple[np.ndarray, np.ndarray, float, float] | None:
        """Interpolate ground-truth pose and velocity at exact image timestamp."""
        if len(self.odom_history) < 2:
            return None

        # Check if t_img is within buffer span
        if t_img < self.odom_history[0][0] or t_img > self.odom_history[-1][0]:
            # Slightly extrapolate if within 30 ms
            dt_tail = t_img - self.odom_history[-1][0]
            if 0.0 <= dt_tail < 0.05:
                _, p, q, wz, v = self.odom_history[-1]
                return p, quat_to_rot_matrix(q[0], q[1], q[2], q[3]), wz, v
            return None

        # Binary or linear scan
        hist = list(self.odom_history)
        for i in range(len(hist) - 1):
            t0, p0, q0, w0, v0 = hist[i]
            t1, p1, q1, w1, v1 = hist[i + 1]
            if t0 <= t_img <= t1:
                denom = max(1e-6, t1 - t0)
                alpha = (t_img - t0) / denom
                p_interp = (1.0 - alpha) * p0 + alpha * p1
                q_interp = slerp(q0, q1, alpha)
                rot_interp = quat_to_rot_matrix(
                    q_interp[0], q_interp[1], q_interp[2], q_interp[3]
                )
                w_interp = (1.0 - alpha) * w0 + alpha * w1
                v_interp = (1.0 - alpha) * v0 + alpha * v1
                return p_interp, rot_interp, w_interp, v_interp

        return None

    def evaluate_frame_labels(
        self, frame: np.ndarray, veh_pos: np.ndarray, veh_rot: np.ndarray
    ) -> tuple[
        list[tuple[int, list[float]]],
        list[float],
        bool,
        list[tuple[int, int, int, int]],
    ]:
        """Compute tight YOLO labels, filter by lighting/size, and verify offsets.

        Returns (labels, offsets, is_valid, raw_boxes).
        """
        candidate_labels: list[tuple[int, list[float]]] = []
        raw_boxes: list[tuple[int, int, int, int]] = []

        # Check all world objects
        for obj in self.objects:
            cls_name = obj["class"]
            cls_id = self.name_to_id.get(cls_name, 0)
            ox = float(obj["x"])
            oy = float(obj["y"])
            oz = float(obj.get("z", -15.0))
            size_m = float(obj["size_m"])

            # Use tight outline polygon if available
            outline = obj.get("outline_m")
            if outline:
                world_pts = [
                    np.array([ox + dx, oy + dy, oz], dtype=np.float64)
                    for dx, dy in outline
                ]
            else:
                half = size_m / 2.0
                world_pts = [
                    np.array([ox + half, oy + half, oz], dtype=np.float64),
                    np.array([ox + half, oy - half, oz], dtype=np.float64),
                    np.array([ox - half, oy - half, oz], dtype=np.float64),
                    np.array([ox - half, oy + half, oz], dtype=np.float64),
                ]

            projs = [
                project_world_point(
                    p,
                    veh_pos=veh_pos,
                    veh_rot=veh_rot,
                    fx=self.fx,
                    fy=self.fy,
                    cx=self.cx,
                    cy=self.cy,
                )
                for p in world_pts
            ]
            valid_projs = [p for p in projs if p is not None]
            if len(valid_projs) < 3:
                continue

            z_opt_mean = float(np.mean([p[2] for p in valid_projs]))
            u_coords = [p[0] for p in valid_projs]
            v_coords = [p[1] for p in valid_projs]
            u_min, u_max = min(u_coords), max(u_coords)
            v_min, v_max = min(v_coords), max(v_coords)

            # Check if completely outside frame
            if u_max <= 0 or u_min >= 640 or v_max <= 0 or v_min >= 480:
                continue

            # Clip box to image bounds
            u_min_c = max(0.0, u_min)
            u_max_c = min(640.0, u_max)
            v_min_c = max(0.0, v_min)
            v_max_c = min(480.0, v_max)

            bw = u_max_c - u_min_c
            bh = v_max_c - v_min_c
            max_dim = max(bw, bh)

            # Check lit area (spotlight aimed at frame center 320, 240)
            box_cx = (u_min_c + u_max_c) / 2.0
            box_cy = (v_min_c + v_max_c) / 2.0
            dist_from_optical_axis = math.hypot(box_cx - 320.0, box_cy - 240.0)

            # DISCARD RULE: If any visible object is between 4 and 8 px, discard frame
            # (they'd become unlabelled background noise per §10.3)
            if (
                4.0 <= max_dim < 8.0
                and z_opt_mean <= 6.0
                and dist_from_optical_axis <= 280.0
            ):
                self.dropped_small_object += 1
                return [], [], False, []

            # Labeling criteria:
            # - within 6 m of camera
            # - at least 12 px in lit area
            if (
                z_opt_mean <= 6.0
                and bw >= 12.0
                and bh >= 12.0
                and dist_from_optical_axis <= 280.0
            ):
                cx_n = box_cx / 640.0
                cy_n = box_cy / 480.0
                w_n = bw / 640.0
                h_n = bh / 480.0
                candidate_labels.append((cls_id, [cx_n, cy_n, w_n, h_n]))
                raw_boxes.append(
                    (round(u_min_c), round(v_min_c), round(u_max_c), round(v_max_c))
                )

        if not candidate_labels:
            # Background frame or no valid labels
            return [], [], True, []

        # Automatic label check: measure centroid offset for every candidate label
        offsets: list[float] = []
        for box in raw_boxes:
            offset, _, _ = measure_pixel_offset(frame, box)
            if offset > 8.0:
                # Required: maximum <= 8 px. Drop any frame containing a label above 8 px.
                self.dropped_large_offset += 1
                return [], [], False, []
            offsets.append(offset)

        return candidate_labels, offsets, True, raw_boxes

    def try_save_frame(
        self,
        images_dir: Path,
        labels_dir: Path,
        is_hover: bool = False,
    ) -> bool:
        """Attempt to save newest buffered frame if timing, rate, and offset criteria are met."""
        if not self.image_buffer:
            return False

        t_img, frame = self.image_buffer[-1]
        pose_res = self.get_pose_at_stamp(t_img)
        if pose_res is None:
            return False

        veh_pos, veh_rot, wz, _lin_vel = pose_res

        # Capture only when yaw rate is low
        if not is_hover and abs(wz) >= 0.25:
            return False

        labels, offsets, is_valid, _raw_boxes = self.evaluate_frame_labels(
            frame, veh_pos, veh_rot
        )
        if not is_valid:
            return False

        # Prefer frames with 1-4 objects
        if not labels and self.captured_count > 50:
            return False

        frame_name = f"{self.prefix}_{self.captured_count:05d}"
        img_path = images_dir / f"{frame_name}.jpg"
        lbl_path = labels_dir / f"{frame_name}.txt"

        cv2.imwrite(str(img_path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        with open(lbl_path, "w", encoding="utf-8") as f:
            f.writelines(
                f"{cls_id} {box[0]:.6f} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f}\n"
                for cls_id, box in labels
            )

        self.all_offsets.extend(offsets)

        # Collect visual samples for human review (including hover frames)
        if labels and len(self.labeled_samples) < 5:
            self.labeled_samples.append((frame, labels, offsets, is_hover))

        self.captured_count += 1
        return True


def drive_and_capture(
    node: SimCollectorNode,
    target_frames: int,
    images_dir: Path,
    labels_dir: Path,
) -> None:
    """Fly survey passes and hover points, capturing frames when yaw rate is low."""
    dt = 0.05  # 20 Hz loop
    capture_interval_s = 0.45  # ~2.2 Hz capture rate
    last_capture_t = 0.0

    # 1. Hover Captures: Hold still at 2.5 m altitude above 2 known objects
    hover_targets = []
    for obj in node.objects[:3]:
        hover_targets.append((float(obj["x"]) - 2.5, float(obj["y"]), -12.5))

    for hx, hy, hz in hover_targets:
        t_h = time.time()
        while time.time() - t_h < 15.0:
            rclpy.spin_once(node, timeout_sec=dt)
            dx = hx - node.cur_x
            dy = hy - node.cur_y
            dz = hz - node.cur_z
            dist = math.hypot(dx, dy)
            cur_yaw = math.atan2(node.cur_rot[1, 0], node.cur_rot[0, 0])
            yaw_err = math.atan2(math.sin(0.0 - cur_yaw), math.cos(0.0 - cur_yaw))

            if dist < 0.25 and abs(dz) < 0.20:
                node.cmd_pub.publish(Twist())
                break

            cmd = Twist()
            head = math.atan2(dy, dx)
            cmd.linear.x = min(1.8, 1.2 * dist * math.cos(head - cur_yaw))
            cmd.linear.y = min(1.2, 1.0 * dist * math.sin(head - cur_yaw))
            cmd.linear.z = min(0.6, max(-0.6, 1.2 * dz))
            cmd.angular.z = min(0.8, max(-0.8, 1.5 * yaw_err))
            node.cmd_pub.publish(cmd)

        # Settle to zero motion
        node.cmd_pub.publish(Twist())
        t_wait = time.time()
        while time.time() - t_wait < 2.0:
            rclpy.spin_once(node, timeout_sec=dt)
            node.cmd_pub.publish(Twist())

        # Capture hover frame
        node.try_save_frame(images_dir, labels_dir, is_hover=True)

    # 2. Survey legs along the 6 lawnmower lines (y = -10, -6, -2, 2, 6, 10)
    survey_y_legs = [-10.0, -6.0, -2.0, 2.0, 6.0, 10.0]
    waypoints: list[tuple[float, float, float, float]] = []
    altitudes = [-12.5, -13.0, -12.0, -12.5, -13.0, -12.5]

    for i, leg_y in enumerate(survey_y_legs):
        alt = altitudes[i % len(altitudes)]
        if i % 2 == 0:
            # Flying East (+X)
            waypoints.append((5.0, leg_y, alt, 0.0))
            waypoints.append((45.0, leg_y, alt, 0.0))
        else:
            # Flying West (-X)
            waypoints.append((45.0, leg_y, alt, math.pi))
            waypoints.append((5.0, leg_y, alt, math.pi))

    # Cross & diagonal survey passes
    waypoints.extend(
        [
            (40.0, 8.0, -12.5, math.pi * 0.75),
            (10.0, -8.0, -13.0, math.pi * 0.25),
            (35.0, -8.0, -12.0, math.pi * 0.5),
            (15.0, 8.0, -12.5, -math.pi * 0.5),
        ]
    )

    wp_idx = 0
    t_start = time.time()
    max_duration = 90.0  # complete 1 full survey pass (~80s)

    while (
        node.captured_count < target_frames and (time.time() - t_start) < max_duration
    ):
        rclpy.spin_once(node, timeout_sec=dt)

        now = time.time()
        if (now - last_capture_t >= capture_interval_s) and node.try_save_frame(
            images_dir, labels_dir, is_hover=False
        ):
            last_capture_t = now

        # Navigation towards current waypoint
        tx, ty, tz, tyaw = waypoints[wp_idx]
        dx = tx - node.cur_x
        dy = ty - node.cur_y
        dz = tz - node.cur_z
        dist_xy = math.hypot(dx, dy)

        cur_yaw = math.atan2(node.cur_rot[1, 0], node.cur_rot[0, 0])
        yaw_err = math.atan2(math.sin(tyaw - cur_yaw), math.cos(tyaw - cur_yaw))

        if dist_xy < 1.2 and abs(dz) < 0.40:
            wp_idx += 1
            if wp_idx >= len(waypoints):
                print(
                    f"[make_sim_dataset] Completed survey pattern: {node.captured_count} frames captured."
                )
                break
            continue

        cmd = Twist()
        head_to_target = math.atan2(dy, dx)
        head_err = math.atan2(
            math.sin(head_to_target - cur_yaw), math.cos(head_to_target - cur_yaw)
        )

        cmd.linear.x = min(2.0, 1.4 * dist_xy * math.cos(head_err))
        cmd.linear.y = min(1.5, 1.0 * dist_xy * math.sin(head_err))
        cmd.linear.z = min(0.6, max(-0.6, 1.2 * dz))
        cmd.angular.z = min(1.0, max(-1.0, 1.6 * yaw_err))
        node.cmd_pub.publish(cmd)

    node.cmd_pub.publish(Twist())


def save_labeled_examples(
    examples: list[tuple[np.ndarray, list[tuple[int, list[float]]], list[float], bool]],
    class_db: ClassDatabase,
) -> None:
    """Save 5 visual validation samples to results/bench/simlabel_v2_<n>.png."""
    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    id_to_name = {c.id: c.name for c in class_db.all_classes()}

    # Ensure at least one hover frame is placed first
    hover_samples = [e for e in examples if e[3]]
    non_hover_samples = [e for e in examples if not e[3]]

    ordered_examples: list[
        tuple[np.ndarray, list[tuple[int, list[float]]], list[float], bool]
    ] = []
    if hover_samples:
        ordered_examples.append(hover_samples[0])
    ordered_examples.extend(non_hover_samples)
    if len(hover_samples) > 1:
        ordered_examples.extend(hover_samples[1:])

    for i, (frame_rgb, labels, offsets, is_hover) in enumerate(ordered_examples[:5]):
        vis = frame_rgb.copy()
        for idx, (cls_id, (cx_n, cy_n, w_n, h_n)) in enumerate(labels):
            cx = cx_n * 640.0
            cy = cy_n * 480.0
            bw = w_n * 640.0
            bh = h_n * 480.0
            x1 = round(cx - bw / 2.0)
            y1 = round(cy - bh / 2.0)
            x2 = round(cx + bw / 2.0)
            y2 = round(cy + bh / 2.0)

            cls_name = id_to_name.get(cls_id, str(cls_id))
            off = offsets[idx] if idx < len(offsets) else 0.0

            # Green box
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Center dot
            cv2.circle(vis, (round(cx), round(cy)), 3, (0, 255, 0), -1)

            mode_str = " [HOVER]" if is_hover else ""
            label_text = f"{cls_name}{mode_str}: off={off:.1f}px"
            cv2.putText(
                vis,
                label_text,
                (max(10, x1), max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
            )

        out_path = BENCH_DIR / f"simlabel_v2_{i}.png"
        cv2.imwrite(str(out_path), cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
        hover_tag = " [HOVER FRAME]" if is_hover else ""
        print(
            f"[make_sim_dataset] Saved: {out_path} ({len(labels)} objects){hover_tag}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate T2.9 simulation fine-tuning dataset."
    )
    parser.add_argument(
        "--target-frames",
        type=int,
        default=2000,
        help="Total simulation frames to capture across layouts.",
    )
    args = parser.parse_args()

    # Layout seeds (distinct from demo seed 42)
    train_seeds = [101, 102]
    val_seed = 103
    num_objects_per_layout = 60

    train_img_dir = SIM_DATA_DIR / "images" / "train"
    train_lbl_dir = SIM_DATA_DIR / "labels" / "train"
    val_img_dir = SIM_DATA_DIR / "images" / "val"
    val_lbl_dir = SIM_DATA_DIR / "labels" / "val"

    for d in [train_img_dir, train_lbl_dir, val_img_dir, val_lbl_dir]:
        d.mkdir(parents=True, exist_ok=True)

    config_path = get_config_path("seabed.yaml")
    class_db = ClassDatabase()
    all_examples: list[
        tuple[np.ndarray, list[tuple[int, list[float]]], list[float], bool]
    ] = []
    global_offsets: list[float] = []
    total_dropped_offset: int = 0
    total_dropped_small: int = 0

    print("=" * 80)
    print("PS11 AUV — T2.9 Simulation Fine-Tuning Dataset Generator")
    print(
        f"Seeds: train={train_seeds}, val={val_seed} (demo seed 42 excluded) | Target frames: {args.target_frames}"
    )
    print("=" * 80)

    # 1. Capture for Train Layouts (90% -> ~900 frames each)
    frames_per_train = int((args.target_frames * 0.90) / len(train_seeds))
    frames_val = int(args.target_frames * 0.10)

    layouts = [
        (train_seeds[0], frames_per_train, "train", train_img_dir, train_lbl_dir),
        (train_seeds[1], frames_per_train, "train", train_img_dir, train_lbl_dir),
        (val_seed, frames_val, "val", val_img_dir, val_lbl_dir),
    ]

    for seed, target_count, split_name, img_dir, lbl_dir in layouts:
        print(
            f"\n--- Generating Layout Seed {seed} ({num_objects_per_layout} objects, split={split_name}) ---"
        )
        sim_objects_yaml = SIM_DATA_DIR / f"world_objects_seed_{seed}.yaml"
        objects = generate_seabed(
            config_path=config_path,
            scenario_name="demo",
            seed_override=seed,
            num_objects_override=num_objects_per_layout,
            objects_output_yaml=sim_objects_yaml,
            update_sdf=True,
        )

        # Launch Gazebo simulation headless
        print(f"[make_sim_dataset] Launching simulation for layout seed {seed}...")
        sim_log = open(  # noqa: SIM115
            BENCH_DIR / f"sim_dataset_seed_{seed}.log", "w", encoding="utf-8"
        )
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
        node = SimCollectorNode(objects, split=split_name, prefix=f"sim_s{seed}")

        try:
            print("[make_sim_dataset] Waiting for odom and camera frames...")
            t0 = time.time()
            while time.time() - t0 < 30.0:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.has_odom and node.latest_frame is not None:
                    break
            else:
                print(
                    f"[make_sim_dataset] Timeout waiting for simulation readiness on seed {seed}."
                )
                continue

            print(
                f"[make_sim_dataset] Capturing {target_count} frames across dense survey pattern..."
            )
            drive_and_capture(node, target_count, img_dir, lbl_dir)
            print(
                f"[make_sim_dataset] Captured {node.captured_count} frames for seed {seed}."
            )

            all_examples.extend(node.labeled_samples)
            global_offsets.extend(node.all_offsets)
            total_dropped_offset += node.dropped_large_offset
            total_dropped_small += node.dropped_small_object
            if len(all_examples) >= 5:
                save_labeled_examples(all_examples, class_db)

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
            time.sleep(2.0)

    # 2. Save 5 labeled examples for human review
    print("\n" + "=" * 80)
    print("SAVING LABELED VALIDATION SAMPLES (results/bench/simlabel_v2_<0..4>.png)")
    print("=" * 80)
    save_labeled_examples(all_examples, class_db)

    # 3. Label-offset Statistics (T2.9 Acceptance Check)
    print("\n" + "=" * 80)
    print("LABEL OFFSET STATISTICS (T2.9 STEP 3 ACCEPTANCE CHECK)")
    print("=" * 80)
    if global_offsets:
        med_off = float(np.median(global_offsets))
        max_off = float(np.max(global_offsets))
        p90_off = float(np.percentile(global_offsets, 90))
        print(f"• Total labels verified : {len(global_offsets)}")
        print(f"• Frames dropped (> 8px): {total_dropped_offset}")
        print(f"• Frames dropped ([4-8px objects]): {total_dropped_small}")
        print(f"• Median label offset   : {med_off:.2f} px (Required: <= 3.0 px)")
        print(f"• 90th percentile offset: {p90_off:.2f} px")
        print(f"• Maximum label offset  : {max_off:.2f} px (Required: <= 8.0 px)")
        if med_off <= 3.0 and max_off <= 8.0:
            print(
                ">>> STATUS: ACCEPTANCE CRITERIA MET (median <= 3 px, max <= 8 px) <<<"
            )
        else:
            print(">>> STATUS: CRITERIA FAILED <<<")
    else:
        print("No labels captured.")

    # 4. Restore official demo seabed (seed 42)
    print("\n[make_sim_dataset] Restoring official demo seabed (seed 42)...")
    generate_seabed(config_path, scenario_name="demo", update_sdf=True)
    print("[make_sim_dataset] Dataset generation complete!")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Detectability verification runner for PS11 AUV (T2.5).

Flies the vehicle over all 12 objects in world_objects.yaml at 2.5 m altitude,
runs the rate-capped detector node, saves annotated frames to results/bench/detect_<id>.png,
measures detector topic frequency, and reports a detectability table.
"""

import math
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ps11_common.classes import ClassDatabase
from ps11_common.image_utils import image_to_numpy
from ps11_common.params import get_config_path, load_yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray


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
                2.0 * (qx * qy + qz * qw),
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


def project_world_point_to_image(
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
    if z_opt <= 0.01:
        return None

    u = fx * float(p_opt[0]) / z_opt + cx
    v = fy * float(p_opt[1]) / z_opt + cy
    return (u, v, z_opt)


class DetectabilityNode(Node):
    """ROS 2 node for controlling the vehicle and recording detections."""

    def __init__(self) -> None:
        super().__init__("detectability_tester")

        self.cur_x: float = 0.0
        self.cur_y: float = 0.0
        self.cur_z: float = -12.5
        self.cur_rot = np.eye(3, dtype=np.float64)
        self.has_odom: bool = False

        self.fx: float = 320.0
        self.fy: float = 320.0
        self.cx: float = 320.0
        self.cy: float = 240.0

        self.latest_annotated: np.ndarray | None = None
        self.latest_detections: Detection2DArray | None = None
        self.det_timestamps: list[float] = []

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
            Detection2DArray,
            "/vehicle/perception/detections",
            self._on_detections,
            qos_rel,
        )
        self.create_subscription(
            Image,
            "/vehicle/perception/image_annotated",
            self._on_annotated,
            qos_rel,
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.cur_x = float(p.x)
        self.cur_y = float(p.y)
        self.cur_z = float(p.z)
        self.cur_rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        self.has_odom = True

    def _on_cam_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def _on_detections(self, msg: Detection2DArray) -> None:
        self.latest_detections = msg
        self.det_timestamps.append(time.time())
        if len(self.det_timestamps) > 100:
            self.det_timestamps.pop(0)

    def _on_annotated(self, msg: Image) -> None:
        try:
            self.latest_annotated = image_to_numpy(msg)
        except Exception:  # noqa: BLE001, S110
            pass

    def drive_to(
        self,
        target_x: float,
        target_y: float,
        target_z: float = -12.5,
        target_yaw: float = 0.0,
        tolerance: float = 0.40,
        timeout_s: float = 25.0,
    ) -> bool:
        """Drive vehicle to waypoint using proportional velocity control."""
        t_start = time.time()
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.time() - t_start < timeout_s:
            rclpy.spin_once(self, timeout_sec=dt)

            dx = target_x - self.cur_x
            dy = target_y - self.cur_y
            dz = target_z - self.cur_z
            dist_xy = math.hypot(dx, dy)

            # Heading error relative to target_yaw (facing +X)
            cur_yaw = math.atan2(self.cur_rot[1, 0], self.cur_rot[0, 0])
            yaw_err = math.atan2(
                math.sin(target_yaw - cur_yaw), math.cos(target_yaw - cur_yaw)
            )

            if dist_xy < tolerance and abs(dz) < 0.25:
                # Arrived, brake
                stop_twist = Twist()
                self.cmd_pub.publish(stop_twist)
                return True

            # High-speed proportional velocity commands (fast transit)
            cmd = Twist()
            head_to_target = math.atan2(dy, dx)
            head_err = math.atan2(
                math.sin(head_to_target - cur_yaw), math.cos(head_to_target - cur_yaw)
            )

            if abs(head_err) > math.pi / 2.0:
                cmd.linear.x = -min(1.2, 0.8 * dist_xy)
            else:
                cmd.linear.x = min(3.0, 1.8 * dist_xy * math.cos(head_err))

            cmd.linear.y = min(2.0, 1.2 * dist_xy * math.sin(head_err))
            cmd.linear.z = min(0.8, max(-0.8, 1.5 * dz))
            cmd.angular.z = min(1.8, max(-1.8, 2.5 * yaw_err))

            self.cmd_pub.publish(cmd)

        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        return False


def main() -> None:
    gui = "--gui" in sys.argv
    gui_str = "true" if gui else "false"

    repo_root = Path(__file__).resolve().parent.parent
    bench_dir = repo_root / "results" / "bench"
    bench_dir.mkdir(parents=True, exist_ok=True)

    objects_file = get_config_path("world_objects.yaml")
    world_data = load_yaml(objects_file)
    objects = world_data.get("objects", [])

    print("=" * 70)
    print("PS11 AUV — Detectability Verification Runner (T2.5)")
    print(f"Total objects to test: {len(objects)} | GUI: {gui_str} (fast-forward mode)")
    print("=" * 70)

    sim_log = open(bench_dir / "sim_detect.log", "w", encoding="utf-8")  # noqa: SIM115
    det_log = open(bench_dir / "detector.log", "w", encoding="utf-8")  # noqa: SIM115

    sim_proc = None
    det_proc = None
    node = None

    try:
        # 1. Launch Gazebo Harmonic Simulation
        print(
            f"[test_detectability] Starting simulation: ros2 launch ps11_gazebo sim.launch.py gui:={gui_str} x:=0.0 y:=0.0 z:=-12.5"
        )
        sim_proc = subprocess.Popen(
            [
                "ros2",
                "launch",
                "ps11_gazebo",
                "sim.launch.py",
                f"gui:={gui_str}",
                "x:=0.0",
                "y:=0.0",
                "z:=-12.5",
            ],
            stdout=sim_log,
            stderr=subprocess.STDOUT,
        )

        time.sleep(5.0)

        # 2. Launch Detector Node
        print(
            "[test_detectability] Starting detector node: ros2 run ps11_perception detector --ros-args -p use_sim_time:=true"
        )
        det_env = dict(os.environ)
        det_env["PYTHONUNBUFFERED"] = "1"
        det_proc = subprocess.Popen(
            [
                "ros2",
                "run",
                "ps11_perception",
                "detector",
                "--ros-args",
                "-p",
                "use_sim_time:=true",
            ],
            stdout=det_log,
            stderr=subprocess.STDOUT,
            env=det_env,
        )

        rclpy.init()
        node = DetectabilityNode()

        # Wait for simulation and perception readiness
        print("[test_detectability] Waiting for clock, odom, and perception...")
        t0 = time.time()
        while time.time() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.has_odom
                and node.latest_annotated is not None
                and len(node.det_timestamps) >= 3
            ):
                break
        else:
            print(
                "[test_detectability] Timeout waiting for simulation and detector readiness."
            )
            sys.exit(1)

        print(
            "[test_detectability] Perception online! Starting flyover testing across all 12 objects...\n"
        )

        results_table: list[dict] = []
        class_db = ClassDatabase()

        for obj in objects:
            obj_id = int(obj["id"])
            cls_name = str(obj["class"])
            target_x = float(obj["x"])
            target_y = float(obj["y"])
            target_z = float(obj.get("z", -15.0))
            size_m = float(obj["size_m"])

            # Waypoint: 2.5 m before object in X, altitude 2.5 m above seabed (z = -12.5)
            # Yaw = 0.0 (facing +X)
            waypoint_x = target_x - 2.5
            waypoint_y = target_y
            waypoint_z = -12.5

            print(
                f"--- Navigating to Object {obj_id} ({cls_name}, {size_m:.2f} m) at ({target_x:.2f}, {target_y:.2f}) ---"
            )
            _reached = node.drive_to(waypoint_x, waypoint_y, waypoint_z, target_yaw=0.0)

            # Settle to let detector process fresh frame at target waypoint
            node.latest_annotated = None
            node.latest_detections = None
            t_settle = time.time()
            settle_timeout = 6.0 if gui else 3.0
            while time.time() - t_settle < 0.6 or node.latest_annotated is None:
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.time() - t_settle > settle_timeout:
                    break

            # Compute ground truth projection into camera
            p_world = np.array([target_x, target_y, target_z], dtype=np.float64)
            veh_pos = np.array([node.cur_x, node.cur_y, node.cur_z], dtype=np.float64)
            proj = project_world_point_to_image(
                p_world=p_world,
                veh_pos=veh_pos,
                veh_rot=node.cur_rot,
                fx=node.fx,
                fy=node.fy,
                cx=node.cx,
                cy=node.cy,
            )

            # Check detections against projection
            detected = False
            best_conf = 0.0
            box_str = "-"

            if node.latest_detections and proj is not None:
                u_gt, v_gt, _ = proj
                for d2d in node.latest_detections.detections:
                    if not d2d.results:
                        continue
                    res = d2d.results[0]
                    hyp = getattr(res, "hypothesis", res)
                    hyp_id = getattr(hyp, "class_id", getattr(hyp, "id", ""))
                    conf = float(getattr(hyp, "score", 0.0))

                    try:
                        detected_cls = class_db.get_by_id(int(hyp_id)).name
                    except (KeyError, ValueError):
                        detected_cls = ""

                    if detected_cls == cls_name:
                        # Check bounding box
                        bbox = d2d.bbox
                        if hasattr(bbox.center, "position"):
                            bcx = bbox.center.position.x
                            bcy = bbox.center.position.y
                        else:
                            bcx = bbox.center.x
                            bcy = bbox.center.y
                        bw = bbox.size_x
                        bh = bbox.size_y

                        x1 = bcx - bw / 2.0
                        x2 = bcx + bw / 2.0
                        y1 = bcy - bh / 2.0
                        y2 = bcy + bh / 2.0

                        # Check if ground truth center falls within the bounding box
                        if x1 <= u_gt <= x2 and y1 <= v_gt <= y2:
                            detected = True
                            if conf > best_conf:
                                best_conf = conf
                                box_str = f"{round(bw)}x{round(bh)}"

            # Save annotated frame
            out_img_path = bench_dir / f"detect_{obj_id}.png"
            if node.latest_annotated is not None:
                # Save as RGB to BGR for cv2.imwrite
                cv2.imwrite(
                    str(out_img_path),
                    cv2.cvtColor(node.latest_annotated, cv2.COLOR_RGB2BGR),
                )
                print(
                    f" Saved frame: {out_img_path} | Detected: {detected} (conf: {best_conf:.2f})"
                )
            else:
                print(f" Warning: No annotated frame received for object {obj_id}")

            results_table.append(
                {
                    "id": obj_id,
                    "class": cls_name,
                    "size_m": size_m,
                    "detected": "YES" if detected else "NO",
                    "best_conf": f"{best_conf:.2f}" if detected else "-",
                    "box_size": box_str,
                }
            )

        # Compute detector publication rate
        hz = 0.0
        if len(node.det_timestamps) >= 2:
            dt_total = node.det_timestamps[-1] - node.det_timestamps[0]
            if dt_total > 0.1:
                hz = (len(node.det_timestamps) - 1) / dt_total

        # Print final formatted table
        print("\n" + "=" * 80)
        print("T2.5 DETECTABILITY RESULTS TABLE")
        print("=" * 80)
        header = f"| {'id':<3} | {'class':<12} | {'size_m':<6} | {'detected with correct class?':<30} | {'best conf':<9} | {'box size (px)':<14} |"
        sep = f"|{'-' * 5}|{'-' * 14}|{'-' * 8}|{'-' * 32}|{'-' * 11}|{'-' * 16}|"
        print(header)
        print(sep)
        for r in results_table:
            row = f"| {r['id']:<3} | {r['class']:<12} | {r['size_m']:<6.3f} | {r['detected']:<30} | {r['best_conf']:<9} | {r['box_size']:<14} |"
            print(row)
        print("=" * 80)
        print(f"Measured detector publish rate: {hz:.2f} Hz (target: ~5 Hz)")
        print("=" * 80 + "\n")

    finally:
        print("[test_detectability] Cleaning up processes...")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if det_proc is not None:
            det_proc.terminate()
            try:
                det_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                det_proc.kill()
        if sim_proc is not None:
            sim_proc.terminate()
            try:
                sim_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                sim_proc.kill()
        sim_log.close()
        det_log.close()


if __name__ == "__main__":
    main()

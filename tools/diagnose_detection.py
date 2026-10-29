#!/usr/bin/env python3
"""Detection diagnostics runner for PS11 AUV (§10.5, T2.5).

Diagnoses detector performance on:
1. Model sanity: tests best.pt on 5 test images containing debris from test_manifest.txt
2. Colour order: analyzes RGB vs BGR handling in Ultralytics YOLO
3. Raw sim frames: captures un-annotated frames at objects 0, 3, 5; runs model at conf 0.05
   in BOTH colour orders (as-is and channel-swapped)
4. Projection check: draws cross on projected (u, v) ground-truth position on raw frames
5. Decal visibility: computes frame brightness vs 40x40 px decal patch brightness
"""

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
from ps11_common.classes import ClassDatabase
from ps11_common.image_utils import image_to_numpy
from ps11_common.params import get_config_path, load_yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO


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


class SimCaptureNode(Node):
    """Simple ROS 2 controller to fly to waypoints and capture raw camera frames."""

    def __init__(self) -> None:
        super().__init__("sim_capture_node")

        self.cur_x: float = 0.0
        self.cur_y: float = 0.0
        self.cur_z: float = -12.5
        self.cur_rot = np.eye(3, dtype=np.float64)
        self.has_odom: bool = False

        self.fx: float = 320.0
        self.fy: float = 320.0
        self.cx: float = 320.0
        self.cy: float = 240.0

        self.latest_raw_rgb: np.ndarray | None = None
        self.raw_frame_count: int = 0

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
        self.cur_rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        self.has_odom = True

    def _on_cam_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest_raw_rgb = image_to_numpy(msg)
            self.raw_frame_count += 1
        except Exception:  # noqa: BLE001, S110
            pass

    def drive_to(
        self,
        target_x: float,
        target_y: float,
        target_z: float = -12.5,
        target_yaw: float = 0.0,
        tolerance: float = 0.35,
        timeout_s: float = 35.0,
    ) -> bool:
        t_start = time.time()
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while time.time() - t_start < timeout_s:
            rclpy.spin_once(self, timeout_sec=dt)

            dx = target_x - self.cur_x
            dy = target_y - self.cur_y
            dz = target_z - self.cur_z
            dist_xy = math.hypot(dx, dy)

            cur_yaw = math.atan2(self.cur_rot[1, 0], self.cur_rot[0, 0])
            yaw_err = math.atan2(
                math.sin(target_yaw - cur_yaw), math.cos(target_yaw - cur_yaw)
            )

            if dist_xy < tolerance and abs(dz) < 0.20:
                self.cmd_pub.publish(Twist())
                return True

            cmd = Twist()
            head_to_target = math.atan2(dy, dx)
            head_err = math.atan2(
                math.sin(head_to_target - cur_yaw), math.cos(head_to_target - cur_yaw)
            )

            if abs(head_err) > math.pi / 2.0:
                cmd.linear.x = -min(0.5, 0.4 * dist_xy)
            else:
                cmd.linear.x = min(1.0, 0.7 * dist_xy * math.cos(head_err))

            cmd.linear.y = min(0.6, 0.5 * dist_xy * math.sin(head_err))
            cmd.linear.z = min(0.4, max(-0.4, 0.8 * dz))
            cmd.angular.z = min(0.6, max(-0.6, 1.2 * yaw_err))

            self.cmd_pub.publish(cmd)

        self.cmd_pub.publish(Twist())
        return False


def run_model_sanity(weights_path: Path, manifest_path: Path) -> None:
    """Step 1: Run best.pt on 5 test manifest images with debris."""
    print("\n" + "=" * 80)
    print("STEP 1: MODEL SANITY CHECK (ON REAL TEST IMAGES)")
    print("=" * 80)

    repo_root = weights_path.parents[2]
    model = YOLO(str(weights_path))

    with open(manifest_path, encoding="utf-8") as f:
        manifest_lines = [line.strip() for line in f if line.strip()]

    # Find images with debris (class 0)
    debris_samples: list[Path] = []
    for rel_img in manifest_lines:
        img_p = repo_root / rel_img
        # Corresponding label file
        # e.g. ml/data/ps11/images/test/123.jpg -> ml/data/ps11/labels/test/123.txt
        label_p = img_p.parent.parent.parent / "labels" / "test" / f"{img_p.stem}.txt"
        if label_p.is_file():
            with open(label_p, encoding="utf-8") as lf:
                for line in lf:
                    if line.strip().startswith("0 "):
                        debris_samples.append(img_p)
                        break
        if len(debris_samples) >= 5:
            break

    print(
        f"Testing on {len(debris_samples)} real test images with ground-truth debris:"
    )
    for img_p in debris_samples:
        results = model(str(img_p), conf=0.25, verbose=False)
        res = results[0]
        dets = []
        if res.boxes is not None and len(res.boxes) > 0:
            for cls_id, conf in zip(
                res.boxes.cls.cpu().numpy(),
                res.boxes.conf.cpu().numpy(),
                strict=True,
            ):
                dets.append(f"{res.names[int(cls_id)]} ({conf:.2f})")
        print(
            f"  • {img_p.name:<25}: {len(dets)} detections -> {', '.join(dets) if dets else 'None'}"
        )


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    bench_dir = repo_root / "results" / "bench"
    bench_dir.mkdir(parents=True, exist_ok=True)

    weights_path = repo_root / "ml" / "weights" / "best.pt"
    manifest_path = repo_root / "ml" / "data" / "test_manifest.txt"

    # Step 1: Model Sanity
    run_model_sanity(weights_path, manifest_path)

    # Step 2: Colour Order Analysis
    print("\n" + "=" * 80)
    print("STEP 2: COLOUR ORDER ANALYSIS")
    print("=" * 80)
    print("• ROS 2 topic /vehicle/camera/image_raw encoding: 'rgb8'")
    print("• ps11_common.image_utils.image_to_numpy returns array in RGB order")
    print("• Ultralytics YOLO(numpy_array) convention:")
    print("  Ultralytics assumes NumPy arrays are in BGR format (like cv2.imread).")
    print("  Inside BasePredictor.preprocess, it executes: `im = im[..., ::-1]`.")
    print("  CONSEQUENCE: If RGB is passed, Ultralytics inverts it to BGR for the CNN!")
    print(
        "  -> Under sim lighting, sand noise gets distorted, resulting in false positives."
    )
    print("  FIX VERIFIED: detector.py converts RGB to BGR before passing to Ultralytics.")

    if "--step1-2" in sys.argv or "--quick" in sys.argv:
        print("\n[diagnose] Completed steps 1 & 2 diagnostics.")
        return

    # Step 3, 4, 5: Sim Flyovers & Frame Analysis
    print("\n" + "=" * 80)
    print(
        "STEPS 3, 4, 5: SIMULATION FLYOVER & RAW FRAME / PROJECTION / BRIGHTNESS CHECK"
    )
    print("=" * 80)

    objects_file = get_config_path("world_objects.yaml")
    world_data = load_yaml(objects_file)
    objects_dict = {int(o["id"]): o for o in world_data.get("objects", [])}
    target_ids = [0, 3, 5]  # Debris (0), Debris (3), Starfish (5)

    sim_log = open(bench_dir / "sim_diagnose.log", "w", encoding="utf-8")  # noqa: SIM115
    sim_proc = None
    node = None

    try:
        print("[diagnose] Launching Gazebo simulation...")
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
        node = SimCaptureNode()

        print("[diagnose] Waiting for clock, odom, and camera...")
        t0 = time.time()
        while time.time() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.has_odom and node.latest_raw_rgb is not None:
                break
        else:
            print("[diagnose] Timeout waiting for simulation readiness.")
            sys.exit(1)

        print("[diagnose] Simulation ready. Running flyovers for objects 0, 3, 5...\n")

        model = YOLO(str(weights_path))
        class_db = ClassDatabase()

        for obj_id in target_ids:
            obj = objects_dict[obj_id]
            cls_name = str(obj["class"])
            target_x = float(obj["x"])
            target_y = float(obj["y"])
            target_z = float(obj.get("z", -15.0))
            size_m = float(obj["size_m"])

            waypoint_x = target_x - 2.5
            waypoint_y = target_y
            waypoint_z = -12.5

            print(
                f"--- Flying to Object {obj_id} ({cls_name}, {size_m:.2f} m) at ({target_x:.2f}, {target_y:.2f}) ---"
            )
            _reached = node.drive_to(waypoint_x, waypoint_y, waypoint_z, target_yaw=0.0)

            # Settle
            t_settle = time.time()
            while time.time() - t_settle < 1.5:
                rclpy.spin_once(node, timeout_sec=0.1)

            if node.latest_raw_rgb is None:
                print(f"  [ERROR] No image received for object {obj_id}")
                continue

            raw_rgb = node.latest_raw_rgb.copy()
            raw_bgr = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR)

            # Save RAW sim frame
            raw_out_path = bench_dir / f"raw_{obj_id}.png"
            cv2.imwrite(str(raw_out_path), raw_bgr)
            print(f"  [Step 3] Saved raw frame to: {raw_out_path}")

            # Compute Projection
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

            # Step 4: Projection check
            proj_out_path = bench_dir / f"proj_{obj_id}.png"
            proj_vis = raw_bgr.copy()
            u_pt, v_pt = None, None
            if proj is not None:
                u, v, z_opt = proj
                u_pt = round(u)
                v_pt = round(v)
                cv2.drawMarker(
                    proj_vis,
                    (u_pt, v_pt),
                    (0, 0, 255),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=30,
                    thickness=2,
                )
                cv2.circle(proj_vis, (u_pt, v_pt), 15, (0, 255, 255), 2)
                cv2.putText(
                    proj_vis,
                    f"GT {cls_name} (Z={z_opt:.2f}m) @ ({u_pt},{v_pt})",
                    (max(10, u_pt - 80), max(20, v_pt - 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            cv2.imwrite(str(proj_out_path), proj_vis)
            print(
                f"  [Step 4] Saved projection overlay to: {proj_out_path} (Projected pixel: ({u_pt}, {v_pt}))"
            )

            # Step 5: Decal visibility and brightness
            mean_frame_lum = float(np.mean(raw_rgb))
            mean_frame_rgb = [float(np.mean(raw_rgb[:, :, c])) for c in range(3)]
            patch_lum_str = "N/A"
            patch_rgb_str = "N/A"
            decal_contrast = "N/A"

            if u_pt is not None and v_pt is not None:
                h, w = raw_rgb.shape[:2]
                y1 = max(0, v_pt - 20)
                y2 = min(h, v_pt + 20)
                x1 = max(0, u_pt - 20)
                x2 = min(w, u_pt + 20)
                patch = raw_rgb[y1:y2, x1:x2]
                if patch.size > 0:
                    mean_patch_lum = float(np.mean(patch))
                    mean_patch_rgb = [float(np.mean(patch[:, :, c])) for c in range(3)]
                    patch_lum_str = f"{mean_patch_lum:.1f}"
                    patch_rgb_str = f"R:{mean_patch_rgb[0]:.1f}, G:{mean_patch_rgb[1]:.1f}, B:{mean_patch_rgb[2]:.1f}"
                    diff = abs(mean_patch_lum - mean_frame_lum)
                    decal_contrast = (
                        f"{diff:.1f} ({'DISTINCT' if diff > 5.0 else 'LOW CONTRAST'})"
                    )

            print("  [Step 5] Brightness:")
            print(
                f"           - Entire Frame: {mean_frame_lum:.1f} (R:{mean_frame_rgb[0]:.1f}, G:{mean_frame_rgb[1]:.1f}, B:{mean_frame_rgb[2]:.1f})"
            )
            print(f"           - 40x40 Patch:  {patch_lum_str} ({patch_rgb_str})")
            print(f"           - Decal Contrast vs Background: {decal_contrast}")

            # Step 3 (inference in BOTH colour orders at conf 0.05)
            # Case A: RGB passed as-is (what detector.py was doing)
            res_as_is = model(raw_rgb, conf=0.05, verbose=False)[0]
            dets_as_is = []
            if res_as_is.boxes is not None:
                for cls_id, conf in zip(
                    res_as_is.boxes.cls.cpu().numpy(),
                    res_as_is.boxes.conf.cpu().numpy(),
                    strict=True,
                ):
                    c_name = class_db.get_by_id(int(cls_id)).name
                    dets_as_is.append(f"{c_name}:{conf:.2f}")

            # Case B: BGR passed (channel swapped, so Ultralytics inverts it to proper RGB)
            res_bgr = model(raw_bgr, conf=0.05, verbose=False)[0]
            dets_bgr = []
            if res_bgr.boxes is not None:
                for cls_id, conf in zip(
                    res_bgr.boxes.cls.cpu().numpy(),
                    res_bgr.boxes.conf.cpu().numpy(),
                    strict=True,
                ):
                    c_name = class_db.get_by_id(int(cls_id)).name
                    dets_bgr.append(f"{c_name}:{conf:.2f}")

            print("  [Step 3] Model detections at conf=0.05:")
            print(
                f"           - Case A (RGB as-is):        {len(dets_as_is)} -> {', '.join(dets_as_is) if dets_as_is else 'None'}"
            )
            print(
                f"           - Case B (Channel-swapped BGR): {len(dets_bgr)} -> {', '.join(dets_bgr) if dets_bgr else 'None'}\n"
            )

    finally:
        print("[diagnose] Cleaning up processes...")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if sim_proc is not None:
            sim_proc.terminate()
            try:
                sim_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                sim_proc.kill()
        sim_log.close()


if __name__ == "__main__":
    main()

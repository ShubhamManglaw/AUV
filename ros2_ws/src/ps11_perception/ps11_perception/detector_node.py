"""detector ROS 2 node (§10.5, T2.5).

Subscribes to the camera image (SensorDataQoS), keeps only the latest frame,
and runs YOLO11n (ml/weights/best.pt, loaded ONCE at startup) on a timer at
the configured max_rate_hz — so camera frames never queue into an inference
backlog. Publishes:

- /vehicle/perception/detections (vision_msgs/Detection2DArray) with the
  SOURCE image header (stamp + frame_id)
- /vehicle/perception/image_annotated (sensor_msgs/Image, SensorDataQoS):
  boxes + class names + confidence + the H3 simulation banner

Configuration comes from ps11_bringup/config/perception.yaml (loaded via
ps11_common.params — YAML is authoritative, no hard-coded constants). Class
ids/names come from classes.yaml via ps11_common.classes. The input topic
defaults to /vehicle/camera/image_raw because underwater_effect (T2.4) is an
optional M1 task that is not built; if it ships, point the YAML at
/vehicle/camera/image_uw. The Jetson throughput is NOT measured yet, so the
banner labels the configured cap explicitly (H3/H6 — no invented FPS).
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from ps11_common.classes import ClassDatabase
from ps11_common.image_utils import image_to_numpy, numpy_to_image
from ps11_common.params import load_yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from ps11_perception.detector_logic import (
    LatestFrame,
    banner_text,
    rows_to_detections,
    validate_class_map,
)


def _find_repo_file(relative: str) -> Path:
    """Locate a repo file (e.g. ml/weights/best.pt) from both source and
    build-tree layouts by walking up the parent chain."""
    p = Path(relative)
    if p.is_absolute():
        return p
    for base in Path(__file__).resolve().parents:
        candidate = base / p
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"cannot locate {relative} from {__file__}")


class DetectorNode(Node):
    """Rate-capped YOLO detector over the latest camera frame."""

    def __init__(self) -> None:
        super().__init__("detector")

        cfg = load_yaml("perception.yaml")["detector"]
        self._imgsz = int(cfg["imgsz"])
        self._conf = float(cfg["conf_threshold"])
        self._iou = float(cfg["iou_threshold"])
        self._max_rate_hz = float(cfg["max_rate_hz"])
        self._annotate_rate_hz = float(cfg["annotate_rate_hz"])
        self._input_topic = str(cfg.get("input_topic", "/vehicle/camera/image_raw"))
        self._nav_mode = str(cfg.get("nav_mode", "kinematic sim (M1)"))
        if self._max_rate_hz <= 0.0 or self._annotate_rate_hz <= 0.0:
            raise ValueError("max_rate_hz and annotate_rate_hz must be > 0")

        classes = ClassDatabase()  # classes.yaml via ament share
        self._id_to_name = {c.id: c.name for c in classes.all_classes()}

        # Model loads ONCE; device resolved from the environment, logged once.
        import torch
        from ultralytics import YOLO

        model_path = str(_find_repo_file(str(cfg["model_path"])))
        requested = str(cfg["device"])
        if requested == "auto":
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        else:
            device = requested
        self._model = YOLO(model_path)
        self._model.to(device)
        model_names = {int(k): str(v) for k, v in self._model.names.items()}
        validate_class_map(model_names, self._id_to_name)
        jetson_measured = bool(cfg.get("jetson_measured", False))
        self._banner = banner_text(self._nav_mode, self._max_rate_hz, jetson_measured)

        self._latest = LatestFrame()
        self._annotate_every = max(1, round(self._max_rate_hz / self._annotate_rate_hz))
        self._inference_count = 0

        self.create_subscription(
            Image, self._input_topic, self._on_image, qos_profile_sensor_data
        )
        self._det_pub = self.create_publisher(
            Detection2DArray, "/vehicle/perception/detections", 10
        )
        self._ann_pub = self.create_publisher(
            Image, "/vehicle/perception/image_annotated", qos_profile_sensor_data
        )
        self.create_timer(1.0 / self._max_rate_hz, self._on_timer)

        rate_label = "Jetson-measured" if jetson_measured else "NOT JETSON-MEASURED YET"
        self.get_logger().info(
            f"detector: model={model_path} device={device} "
            f"max_rate_hz={self._max_rate_hz} ({rate_label})"
        )
        self.get_logger().info(f"H3 banner: {self._banner}")

    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_numpy(msg)
        except ValueError as exc:
            self.get_logger().warning(f"unsupported camera frame: {exc}")
            return
        self._latest.update(frame, msg.header.stamp, msg.header.frame_id)

    def _on_timer(self) -> None:
        if not self._latest.has_frame:
            return
        frame, stamp, frame_id = self._latest.take()
        self._inference_count += 1

        result = self._model.predict(
            frame,
            imgsz=self._imgsz,
            conf=self._conf,
            iou=self._iou,
            verbose=False,
        )[0]
        rows = []
        boxes = result.boxes
        if boxes is not None:
            for cls_id, conf, xywh in zip(
                boxes.cls.tolist(), boxes.conf.tolist(), boxes.xywh.tolist()
            ):
                rows.append((int(cls_id), float(conf), *xywh))
        detections = rows_to_detections(rows, self._conf, self._id_to_name)

        array = Detection2DArray()
        array.header.stamp = stamp
        array.header.frame_id = frame_id or "camera"
        for det in detections:
            det_msg = Detection2D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.class_id)
            hyp.hypothesis.score = det.confidence
            det_msg.results.append(hyp)
            det_msg.bbox.center.position.x = det.center_x
            det_msg.bbox.center.position.y = det.center_y
            det_msg.bbox.size_x = det.size_x
            det_msg.bbox.size_y = det.size_y
            array.detections.append(det_msg)
        self._det_pub.publish(array)

        if self._inference_count % self._annotate_every == 0:
            self._publish_annotated(frame, detections, stamp, frame_id)

    def _publish_annotated(self, frame, detections, stamp, frame_id) -> None:
        annotated = frame.copy()
        for det in detections:
            x0 = int(det.center_x - det.size_x / 2)
            y0 = int(det.center_y - det.size_y / 2)
            x1 = int(det.center_x + det.size_x / 2)
            y1 = int(det.center_y + det.size_y / 2)
            cv2.rectangle(annotated, (x0, y0), (x1, y1), (255, 80, 40), 2)
            cv2.putText(
                annotated,
                f"{det.class_name} {det.confidence:.2f}",
                (max(x0, 0), max(y0 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            annotated,
            self._banner,
            (8, 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            self._banner,
            (8, 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        msg = numpy_to_image(np.ascontiguousarray(annotated), "rgb8")
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id or "camera"
        self._ann_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = DetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

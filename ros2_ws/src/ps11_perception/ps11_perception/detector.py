"""Pure logic YOLO detector and visual annotator for PS11 AUV (§10.5)."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from ps11_common.classes import ClassDatabase


@dataclass(frozen=True)
class Detection:
    """Bounding box detection result."""

    class_id: int
    class_name: str
    confidence: float
    cx: float  # Center x in pixels
    cy: float  # Center y in pixels
    w: float  # Width in pixels
    h: float  # Height in pixels

    @property
    def xmin(self) -> float:
        return self.cx - self.w / 2.0

    @property
    def ymin(self) -> float:
        return self.cy - self.h / 2.0

    @property
    def xmax(self) -> float:
        return self.cx + self.w / 2.0

    @property
    def ymax(self) -> float:
        return self.cy + self.h / 2.0


def hex_to_rgb(hex_code: str) -> tuple[int, int, int]:
    """Convert hex color string (e.g. '#E4572E') to (R, G, B) tuple."""
    hex_code = hex_code.lstrip("#")
    if len(hex_code) != 6:
        return (255, 255, 255)
    return (
        int(hex_code[0:2], 16),
        int(hex_code[2:4], 16),
        int(hex_code[4:6], 16),
    )


class YOLODetector:
    """Ultralytics YOLO wrapper for onboard inference and annotation."""

    def __init__(
        self,
        model_path: str | Path,
        device: str = "cuda:0",
        conf_threshold: float = 0.35,
        iou_threshold: float = 0.5,
        imgsz: int = 640,
        class_db: ClassDatabase | None = None,
    ) -> None:
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.imgsz = imgsz
        self.class_db = class_db or ClassDatabase()

        # Handle device selection safely
        if device.startswith("cuda") and not torch.cuda.is_available():
            self.device = "cpu"
        else:
            self.device = device

        if not self.model_path.exists():
            raise FileNotFoundError(f"Model weights not found at: {self.model_path}")

        from ultralytics import YOLO

        self.model = YOLO(str(self.model_path))

        # Precompute colors per class ID (RGB)
        self.colors_rgb: dict[int, tuple[int, int, int]] = {}
        for c in self.class_db.all_classes():
            self.colors_rgb[c.id] = hex_to_rgb(c.color)

    def detect(self, img_rgb: np.ndarray) -> list[Detection]:
        """Run YOLO inference on an RGB image and return list of detections.

        Ultralytics assumes NumPy array inputs are in BGR format (like cv2.imread).
        It internally executes `im = im[..., ::-1]` to feed RGB tensors to the CNN.
        Converting img_rgb to BGR ensures Ultralytics correctly produces true RGB tensors.
        """
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        results = self.model(
            img_bgr,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False,
        )

        detections: list[Detection] = []
        if not results:
            return detections

        res: Any = results[0]
        if res.boxes is None or len(res.boxes) == 0:
            return detections

        # Extract tensor data
        boxes_xywh = res.boxes.xywh.cpu().numpy()
        boxes_cls = res.boxes.cls.cpu().numpy()
        boxes_conf = res.boxes.conf.cpu().numpy()

        for xywh, cls_id_raw, conf in zip(
            boxes_xywh, boxes_cls, boxes_conf, strict=True
        ):
            cls_id = int(cls_id_raw)
            try:
                c_info = self.class_db.get_by_id(cls_id)
                cls_name = c_info.name
            except KeyError:
                cls_name = f"unknown_{cls_id}"

            detections.append(
                Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=float(conf),
                    cx=float(xywh[0]),
                    cy=float(xywh[1]),
                    w=float(xywh[2]),
                    h=float(xywh[3]),
                )
            )

        return detections

    def annotate(
        self,
        img_rgb: np.ndarray,
        detections: list[Detection],
        banner_text: str = "SIMULATION | NAV: kinematic (M1) | detector rate capped",
    ) -> np.ndarray:
        """Draw bounding boxes, labels, and the H3 banner on a copy of the RGB image."""
        annotated = img_rgb.copy()
        height, width = annotated.shape[:2]

        # Draw detections
        for det in detections:
            color = self.colors_rgb.get(det.class_id, (255, 255, 0))

            x1 = max(0, round(det.xmin))
            y1 = max(0, round(det.ymin))
            x2 = min(width - 1, round(det.xmax))
            y2 = min(height - 1, round(det.ymax))

            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness=2)

            # Label text
            label = f"{det.class_name} {det.confidence:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.45
            thickness = 1

            (tw, th), baseline = cv2.getTextSize(label, font, font_scale, thickness)
            # Label background box
            label_y1 = max(0, y1 - th - baseline - 4)
            label_y2 = y1
            label_x2 = min(width - 1, x1 + tw + 4)

            cv2.rectangle(
                annotated,
                (x1, label_y1),
                (label_x2, label_y2),
                color,
                thickness=-1,
            )

            # Contrasting text: dark text for bright colors
            text_color = (0, 0, 0) if sum(color) > 380 else (255, 255, 255)
            cv2.putText(
                annotated,
                label,
                (x1 + 2, label_y2 - baseline - 1),
                font,
                font_scale,
                text_color,
                thickness,
                lineType=cv2.LINE_AA,
            )

        # Draw H3 Banner at the top (§1.4, §10.5)
        banner_height = 24
        # Solid dark bar
        cv2.rectangle(
            annotated,
            (0, 0),
            (width, banner_height),
            (25, 25, 25),
            thickness=-1,
        )

        # Centered white banner text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.40
        thickness = 1
        (bw, bh), _ = cv2.getTextSize(banner_text, font, font_scale, thickness)
        tx = max(4, (width - bw) // 2)
        ty = (banner_height + bh) // 2

        cv2.putText(
            annotated,
            banner_text,
            (tx, ty),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            lineType=cv2.LINE_AA,
        )

        return annotated

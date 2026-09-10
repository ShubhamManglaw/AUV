"""Unit tests for YOLO detector logic, H3 banner, and messages (T2.5)."""

from pathlib import Path

import numpy as np
import pytest
from ps11_common.classes import ClassDatabase
from ps11_perception.detector import Detection, YOLODetector, hex_to_rgb
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)


def _find_repo_root() -> Path:
    p = Path(__file__).resolve()
    for parent in p.parents:
        if (parent / "ml" / "weights" / "best.pt").is_file():
            return parent
    raise FileNotFoundError("Could not find repo root with ml/weights/best.pt")


def test_hex_to_rgb() -> None:
    assert hex_to_rgb("#E4572E") == (228, 87, 46)
    assert hex_to_rgb("#F2C14E") == (242, 193, 78)
    assert hex_to_rgb("#9B5DE5") == (155, 93, 229)
    assert hex_to_rgb("#4ECDC4") == (78, 205, 196)


def test_detection_properties() -> None:
    det = Detection(
        class_id=0,
        class_name="debris",
        confidence=0.85,
        cx=320.0,
        cy=240.0,
        w=100.0,
        h=80.0,
    )
    assert det.xmin == 270.0
    assert det.xmax == 370.0
    assert det.ymin == 200.0
    assert det.ymax == 280.0


def test_annotation_with_h3_banner() -> None:
    repo_root = _find_repo_root()
    weights_path = repo_root / "ml" / "weights" / "best.pt"

    detector = YOLODetector(
        model_path=weights_path,
        device="cpu",  # safe for test runner
        class_db=ClassDatabase(),
    )

    img = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = [
        Detection(
            class_id=0,
            class_name="debris",
            confidence=0.92,
            cx=200.0,
            cy=200.0,
            w=80.0,
            h=60.0,
        ),
        Detection(
            class_id=1,
            class_name="starfish",
            confidence=0.78,
            cx=400.0,
            cy=300.0,
            w=50.0,
            h=50.0,
        ),
    ]

    annotated = detector.annotate(
        img,
        detections,
        banner_text="SIMULATION | NAV: kinematic (M1) | detector rate capped",
    )

    assert annotated.shape == (480, 640, 3)
    # Check that banner region has been painted (non-zero)
    banner_region = annotated[:24, :, :]
    assert np.any(banner_region > 0), "Banner region must not be completely black"

    # Check that bounding box / label region has been painted
    box_edge_region = annotated[165:175, 160:240, :]
    assert np.any(box_edge_region > 0), "Bounding box / label region must be drawn"


def test_yolo_inference_on_synthetic_image() -> None:
    repo_root = _find_repo_root()
    weights_path = repo_root / "ml" / "weights" / "best.pt"

    detector = YOLODetector(
        model_path=weights_path,
        device="cpu",
        conf_threshold=0.2,
    )

    dummy_frame = np.full((480, 640, 3), 128, dtype=np.uint8)
    results = detector.detect(dummy_frame)
    assert isinstance(results, list)
    for r in results:
        assert isinstance(r, Detection)
        assert 0 <= r.class_id <= 3
        assert 0.0 <= r.confidence <= 1.0


def test_vision_msgs_compatibility() -> None:
    det_array = Detection2DArray()
    d2d = Detection2D()

    bbox = BoundingBox2D()
    if hasattr(bbox.center, "position"):
        bbox.center.position.x = 320.0
        bbox.center.position.y = 240.0
    else:
        bbox.center.x = 320.0
        bbox.center.y = 240.0
    bbox.size_x = 50.0
    bbox.size_y = 60.0
    d2d.bbox = bbox

    hyp = ObjectHypothesis()
    if hasattr(hyp, "class_id"):
        hyp.class_id = "1"
    else:
        hyp.id = "1"
    hyp.score = 0.88

    hyp_pose = ObjectHypothesisWithPose()
    if hasattr(hyp_pose, "hypothesis"):
        hyp_pose.hypothesis = hyp
    else:
        if hasattr(hyp_pose, "id"):
            hyp_pose.id = "1"
        hyp_pose.score = 0.88

    d2d.results.append(hyp_pose)
    det_array.detections.append(d2d)

    assert len(det_array.detections) == 1
    assert det_array.detections[0].bbox.size_x == pytest.approx(50.0)

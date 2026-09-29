"""Focused unit tests for the T2.5 detector logic (plan §10.5).

Inference is mocked: the logic functions take plain rows, so no YOLO model is
loaded during tests.
"""

from __future__ import annotations

import pytest
from ps11_perception.detector_logic import (
    Detection,
    LatestFrame,
    banner_text,
    rows_to_detections,
    validate_class_map,
)

CONFIGURED = {0: "debris", 1: "starfish", 2: "sea_urchin", 3: "scallop"}


def test_class_map_validation_accepts_matching_map() -> None:
    validate_class_map(
        {0: "debris", 1: "starfish", 2: "sea_urchin", 3: "scallop"}, CONFIGURED
    )


def test_class_map_validation_rejects_unknown_model_ids() -> None:
    with pytest.raises(ValueError, match="not defined in classes.yaml"):
        validate_class_map({0: "debris", 7: "boat"}, CONFIGURED)


def test_model_class_id_maps_to_configured_name() -> None:
    rows = [(2, 0.9, 100.0, 200.0, 40.0, 30.0)]
    dets = rows_to_detections(rows, 0.35, CONFIGURED)
    assert dets[0].class_id == 2
    assert dets[0].class_name == "sea_urchin"


def test_confidence_filtering() -> None:
    rows = [
        (0, 0.9, 10.0, 10.0, 20.0, 20.0),
        (0, 0.35, 30.0, 10.0, 20.0, 20.0),
        (0, 0.3499, 50.0, 10.0, 20.0, 20.0),
    ]
    dets = rows_to_detections(rows, 0.35, CONFIGURED)
    assert [d.confidence for d in dets] == [0.9, 0.35]


def test_bbox_conversion_into_ros_representation() -> None:
    rows = [(1, 0.8, 320.0, 240.0, 64.0, 48.0)]
    dets = rows_to_detections(rows, 0.35, CONFIGURED)
    d = dets[0]
    assert isinstance(d, Detection)
    assert (d.center_x, d.center_y) == (320.0, 240.0)
    assert (d.size_x, d.size_y) == (64.0, 48.0)
    assert 0.0 <= d.confidence <= 1.0


def test_invalid_class_ids_ignored_safely() -> None:
    rows = [
        (0, 0.9, 10.0, 10.0, 20.0, 20.0),
        (5, 0.95, 50.0, 50.0, 20.0, 20.0),  # not in classes.yaml
        (1, 0.7, 70.0, 70.0, 20.0, 20.0),
    ]
    dets = rows_to_detections(rows, 0.35, CONFIGURED)
    assert [d.class_id for d in dets] == [0, 1]
    assert all(d.class_name in CONFIGURED.values() for d in dets)


def test_empty_detections_handled_cleanly() -> None:
    assert rows_to_detections([], 0.35, CONFIGURED) == []
    assert rows_to_detections([(0, 0.1, 1.0, 1.0, 2.0, 2.0)], 0.35, CONFIGURED) == []


def test_latest_frame_slot_never_queues() -> None:
    slot = LatestFrame()
    for k in range(100):
        slot.update(frame=f"frame_{k}", stamp=k, frame_id="camera")
    assert slot.has_frame
    frame, stamp, frame_id = slot.take()
    assert frame == "frame_99" and stamp == 99 and frame_id == "camera"
    assert not slot.has_frame
    assert slot.take() == (None, None, None)


def test_h3_banner_text_labels_unmeasured_rate() -> None:
    text = banner_text("kinematic sim (M1)", 5.0, jetson_measured=False)
    assert "SIMULATION" in text
    assert "NAV: kinematic sim (M1)" in text
    assert "5.0 Hz" in text
    assert "NOT JETSON-MEASURED YET" in text
    assert "Jetson-measured)" not in text.replace("(NOT JETSON-MEASURED YET)", "")


def test_h3_banner_text_marks_measured_rate_only_when_measured() -> None:
    text = banner_text("kinematic sim (M1)", 20.0, jetson_measured=True)
    assert "20.0 Hz" in text
    assert "Jetson-measured" in text
    assert "NOT JETSON-MEASURED" not in text

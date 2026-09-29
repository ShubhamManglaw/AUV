"""ROS-free detector logic for T2.5 (plan §10.5).

Pure post-processing: class-map validation, confidence filtering, bounding-box
conversion into the vision_msgs representation, the H3 simulation banner and
the latest-frame slot that keeps inference backlog-free. No rclpy, no
ultralytics, no numpy — the ROS node extracts plain rows from the model output
and hands them to these functions, so unit tests can mock inference entirely.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    """One detection in pixels, ready for the vision_msgs representation."""

    class_id: int
    class_name: str
    confidence: float
    center_x: float
    center_y: float
    size_x: float
    size_y: float


def validate_class_map(model_names: dict[int, str], configured: dict[int, str]) -> None:
    """Startup guard: every model output class id must exist in the configured
    classes.yaml map (§10.1). Hard-coded ids are forbidden."""
    unknown = sorted(set(model_names) - set(configured))
    if unknown:
        raise ValueError(
            f"model output class ids {unknown} are not defined in classes.yaml "
            f"(configured ids: {sorted(configured)})"
        )


def rows_to_detections(
    rows: list[tuple[int, float, float, float, float, float]],
    conf_threshold: float,
    id_to_name: dict[int, str],
) -> list[Detection]:
    """Convert raw model rows (cls_id, conf, cx, cy, w, h) in pixels.

    Detections below conf_threshold are dropped; class ids missing from the
    configured map are ignored safely (never published).
    """
    out: list[Detection] = []
    for cls_id, conf, cx, cy, w, h in rows:
        if conf < conf_threshold:
            continue
        name = id_to_name.get(cls_id)
        if name is None:
            continue
        out.append(
            Detection(
                class_id=int(cls_id),
                class_name=name,
                confidence=float(conf),
                center_x=float(cx),
                center_y=float(cy),
                size_x=float(w),
                size_y=float(h),
            )
        )
    return out


def banner_text(nav_mode: str, detector_rate_hz: float, jetson_measured: bool) -> str:
    """H3 on-screen banner. The configured cap is always shown; a Jetson
    throughput claim appears ONLY when a measured benchmark exists (H6)."""
    if jetson_measured:
        return (
            f"SIMULATION | NAV: {nav_mode} | "
            f"DETECTOR: {detector_rate_hz:.1f} Hz cap (Jetson-measured)"
        )
    return (
        f"SIMULATION | NAV: {nav_mode} | "
        f"DETECTOR: {detector_rate_hz:.1f} Hz cap (NOT JETSON-MEASURED YET)"
    )


class LatestFrame:
    """Single-slot latest-frame store (plan §10.5 rate-cap pattern).

    Camera callbacks overwrite the slot; the inference timer takes what is
    there. N updates never queue N frames — the slot holds exactly one.
    """

    def __init__(self) -> None:
        self._frame = None
        self._stamp = None
        self._frame_id = None

    def update(self, frame, stamp, frame_id: str) -> None:
        self._frame = frame
        self._stamp = stamp
        self._frame_id = frame_id

    def take(self) -> tuple[object, object, str | None]:
        """Return (frame, stamp, frame_id) and clear the slot."""
        frame, stamp, frame_id = self._frame, self._stamp, self._frame_id
        self._frame = self._stamp = None
        self._frame_id = None
        return frame, stamp, frame_id

    @property
    def has_frame(self) -> bool:
        return self._frame is not None

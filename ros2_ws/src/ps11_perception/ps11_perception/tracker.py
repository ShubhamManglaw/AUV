"""ByteTrack-based multi-object tracker module (ROS-free logic) (§10.6).

Pure tracking logic:
- Uses supervision ByteTrack for association.
- Confirmation: tracks must reach min_hits and have mean confidence >= min_mean_conf.
- Class assignment: majority vote across the track's full detection history.
- Confidence: running mean over the last 10 hits.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import supervision as sv


@dataclass(frozen=True)
class TrackedObject:
    """Confirmed tracked object with smoothed kinematics and majority-vote class."""

    track_id: int
    class_id: int
    confidence: float  # mean over last 10 hits
    u: float  # bbox centre, px
    v: float
    width: float  # px
    height: float  # px
    hits: int


class ObjectTracker:
    """Manages ByteTrack updates, hit counting, and confirmation criteria."""

    def __init__(
        self,
        min_hits: int = 3,
        min_mean_conf: float = 0.30,
        track_activation_threshold: float = 0.25,
        lost_track_buffer: int = 30,
        minimum_matching_threshold: float = 0.8,
        recency_weight_power: float = 2.0,
        debris_priority_boost: float = 1.5,
    ) -> None:
        self.min_hits = min_hits
        self.min_mean_conf = min_mean_conf
        self.recency_weight_power = recency_weight_power
        self.debris_priority_boost = debris_priority_boost
        self.tracker = sv.ByteTrack(
            track_activation_threshold=track_activation_threshold,
            lost_track_buffer=lost_track_buffer,
            minimum_matching_threshold=minimum_matching_threshold,
        )
        # track_id -> history dict: {"class_ids": list[int], "confidences": list[float]}
        self._history: dict[int, dict[str, list[Any]]] = {}

    def update(
        self,
        xyxy: np.ndarray,
        confidence: np.ndarray,
        class_id: np.ndarray,
    ) -> list[TrackedObject]:
        """Update tracker with frame detections and return confirmed tracks.

        Args:
            xyxy: (N, 4) bounding boxes [x1, y1, x2, y2].
            confidence: (N,) detection confidence scores.
            class_id: (N,) integer class IDs.

        Returns:
            List of confirmed TrackedObject instances.
        """
        if len(xyxy) == 0:
            detections = sv.Detections.empty()
        else:
            detections = sv.Detections(
                xyxy=np.asarray(xyxy, dtype=float),
                confidence=np.asarray(confidence, dtype=float),
                class_id=np.asarray(class_id, dtype=int),
            )

        tracked = self.tracker.update_with_detections(detections)

        confirmed_tracks: list[TrackedObject] = []
        if len(tracked) == 0 or tracked.tracker_id is None:
            return confirmed_tracks

        for i in range(len(tracked)):
            t_id = int(tracked.tracker_id[i])
            c_id = int(tracked.class_id[i])
            conf = float(tracked.confidence[i])
            box = tracked.xyxy[i]

            if t_id not in self._history:
                self._history[t_id] = {
                    "class_ids": [],
                    "confidences": [],
                }

            hist = self._history[t_id]
            hist["class_ids"].append(c_id)
            hist["confidences"].append(conf)

            hits = len(hist["class_ids"])
            mean_conf = float(np.mean(hist["confidences"]))

            # Confirmation condition (§10.6):
            # seen in at least min_hits frames with mean confidence >= min_mean_conf
            if hits >= self.min_hits and mean_conf >= self.min_mean_conf:
                # Recency & confidence-weighted class assignment (§10.6):
                # When distant, initial observations classify the object.
                # When moving closer, high-confidence and recent sightings receive
                # progressive favor, with priority boost for defence proxy / debris.
                class_scores: dict[int, float] = {}
                n_obs = len(hist["class_ids"])
                for k in range(n_obs):
                    past_cid = hist["class_ids"][k]
                    past_conf = hist["confidences"][k]
                    recency_norm = (k + 1) / n_obs
                    recency_factor = 1.0 + 3.0 * (
                        recency_norm**self.recency_weight_power
                    )
                    conf_factor = past_conf * past_conf
                    priority_factor = (
                        self.debris_priority_boost if past_cid == 0 else 1.0
                    )
                    score_k = recency_factor * conf_factor * priority_factor
                    class_scores[past_cid] = class_scores.get(past_cid, 0.0) + score_k

                assigned_class = max(class_scores, key=class_scores.get)
                # Confidence is mean over the last 10 hits
                recent_conf = float(np.mean(hist["confidences"][-10:]))

                x1, y1, x2, y2 = box
                w = float(x2 - x1)
                h = float(y2 - y1)
                u = float(x1 + x2) / 2.0
                v = float(y1 + y2) / 2.0

                confirmed_tracks.append(
                    TrackedObject(
                        track_id=t_id,
                        class_id=assigned_class,
                        confidence=recent_conf,
                        u=u,
                        v=v,
                        width=w,
                        height=h,
                        hits=hits,
                    )
                )

        return confirmed_tracks

    def reset(self) -> None:
        """Reset tracker state and history."""
        self.tracker.reset()
        self._history.clear()

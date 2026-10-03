"""Pure logic for evaluation metrics and contact matching (§12.1–12.2).

ROS-free module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class GroundTruthObject:
    """Ground truth object definition from world_objects.yaml."""

    id: int
    name: str
    class_id: int
    x: float
    y: float
    z: float
    size_m: float = 0.5


@dataclass
class ContactObservation:
    """Surface or onboard contact observation."""

    contact_id: int
    class_id: int
    x: float
    y: float
    z: float
    confidence: float
    first_seen_s: float
    last_seen_s: float


@dataclass
class MatchResult:
    """Result of matching surface contacts against ground truth."""

    gt: GroundTruthObject
    contact: ContactObservation
    distance_m: float
    latency_s: float


@dataclass
class EvaluationCounters:
    """Summary of demo evaluation counters."""

    semantic_bits_sent: int = 0
    jpeg_equiv_bits: int = 0
    ratio_vs_jpeg: float = 0.0
    jpeg_airtime_at_link_s: float = 0.0
    contacts_onboard: int = 0
    contacts_at_surface: int = 0
    gt_objects_total: int = 0
    gt_objects_reported: int = 0
    surface_recall: float = 0.0
    mean_position_error_m: float = 0.0
    mean_first_report_latency_s: float = 0.0


def match_contacts_to_gt(
    gt_objects: list[GroundTruthObject],
    surface_contacts: list[ContactObservation],
    max_distance_m: float = 3.0,
    first_view_times: dict[int, float] | None = None,
) -> tuple[list[MatchResult], list[GroundTruthObject], list[ContactObservation]]:
    """Match surface contacts to ground truth objects (§12.2).

    Rules:
    - Same class required
    - Distance <= max_distance_m (3.0 m)
    - 1-to-1 greedy matching prioritized by shortest distance
    """
    first_view_times = first_view_times or {}

    # Find all valid pairs (dist <= max_distance_m, same class)
    candidates: list[tuple[float, int, int]] = []
    for g_idx, g in enumerate(gt_objects):
        for c_idx, c in enumerate(surface_contacts):
            if g.class_id == c.class_id:
                dx = g.x - c.x
                dy = g.y - c.y
                # Horizontal 2D distance per seabed evaluation (§12.2)
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= max_distance_m:
                    candidates.append((dist, g_idx, c_idx))

    # Sort candidates by distance ascending
    candidates.sort(key=lambda x: x[0])

    matched_gt_indices: set[int] = set()
    matched_c_indices: set[int] = set()
    matches: list[MatchResult] = []

    for dist, g_idx, c_idx in candidates:
        if g_idx not in matched_gt_indices and c_idx not in matched_c_indices:
            matched_gt_indices.add(g_idx)
            matched_c_indices.add(c_idx)

            gt_obj = gt_objects[g_idx]
            contact = surface_contacts[c_idx]

            # Latency: time from first view (or contact first seen) to contact report arrival
            t_first_view = first_view_times.get(gt_obj.id, contact.first_seen_s)
            latency = max(0.0, contact.last_seen_s - t_first_view)

            matches.append(
                MatchResult(
                    gt=gt_obj,
                    contact=contact,
                    distance_m=dist,
                    latency_s=latency,
                )
            )

    unmatched_gt = [g for i, g in enumerate(gt_objects) if i not in matched_gt_indices]
    unmatched_contacts = [
        c for i, c in enumerate(surface_contacts) if i not in matched_c_indices
    ]

    return matches, unmatched_gt, unmatched_contacts


def calculate_counters(
    semantic_bits_sent: int,
    jpeg_equiv_bits: int,
    link_bitrate_bps: int,
    contacts_onboard_count: int,
    surface_contacts: list[ContactObservation],
    gt_objects: list[GroundTruthObject],
    first_view_times: dict[int, float] | None = None,
    max_distance_m: float = 3.0,
) -> EvaluationCounters:
    """Compute live and final evaluation counters (§12.2)."""
    matches, _, _ = match_contacts_to_gt(
        gt_objects,
        surface_contacts,
        max_distance_m=max_distance_m,
        first_view_times=first_view_times,
    )

    ratio = (
        float(jpeg_equiv_bits) / float(semantic_bits_sent)
        if semantic_bits_sent > 0
        else 0.0
    )
    airtime = (
        float(jpeg_equiv_bits) / float(link_bitrate_bps)
        if link_bitrate_bps > 0
        else 0.0
    )

    gt_total = len(gt_objects)
    gt_reported = len(matches)
    recall = float(gt_reported) / float(gt_total) if gt_total > 0 else 0.0

    mean_pos_err = sum(m.distance_m for m in matches) / len(matches) if matches else 0.0
    mean_latency = sum(m.latency_s for m in matches) / len(matches) if matches else 0.0

    return EvaluationCounters(
        semantic_bits_sent=semantic_bits_sent,
        jpeg_equiv_bits=jpeg_equiv_bits,
        ratio_vs_jpeg=ratio,
        jpeg_airtime_at_link_s=airtime,
        contacts_onboard=contacts_onboard_count,
        contacts_at_surface=len(surface_contacts),
        gt_objects_total=gt_total,
        gt_objects_reported=gt_reported,
        surface_recall=recall,
        mean_position_error_m=mean_pos_err,
        mean_first_report_latency_s=mean_latency,
    )

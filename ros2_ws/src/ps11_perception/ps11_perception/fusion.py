"""Contact Database and Observation Fusion (ROS-free logic) (§10.8).

Pure fusion logic:
- Association:
  1. Direct track ID match.
  2. Nearest same-class contact within max(gate_m, 3*sigma).
  3. New contact creation (ID 0-255 wrapping).
- Fusion:
  Inverse-variance weighted mean position.
  sigma = max(sigma_floor, 1.0 / sqrt(sum(1/sigma_i^2))).
  confidence = max(confidences).
  class = majority vote across observations.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field
from typing import Any


@dataclass
class FusedContact:
    """Internal fused contact representation."""

    contact_id: int
    class_id: int
    confidence: float
    x: float
    y: float
    z: float
    sigma_xy_m: float
    depth_m: float
    first_seen_stamp: Any  # (sec, nanosec) or float
    last_seen_stamp: Any
    sightings: int
    associated_tracks: set[int] = field(default_factory=set)
    class_votes: list[int] = field(default_factory=list)
    sum_inv_var: float = 0.0
    sum_wx: float = 0.0
    sum_wy: float = 0.0
    sum_wz: float = 0.0


class ContactDatabase:
    """Manages spatial gating, track association, and inverse-variance contact fusion."""

    def __init__(self, gate_m: float = 2.0, sigma_floor_m: float = 0.3) -> None:
        self.gate_m = gate_m
        self.sigma_floor_m = sigma_floor_m
        self._contacts: dict[int, FusedContact] = {}
        self._next_id: int = 0

    @property
    def contacts(self) -> list[FusedContact]:
        return list(self._contacts.values())

    def update(
        self,
        track_id: int,
        class_id: int,
        confidence: float,
        x: float,
        y: float,
        z: float,
        sigma_xy_m: float,
        stamp: Any,
    ) -> FusedContact:
        """Associate an observation and fuse into the contact database.

        Args:
            track_id: Tracker track identifier.
            class_id: Observation class identifier.
            confidence: Observation confidence score.
            x: Map X position (m).
            y: Map Y position (m).
            z: Map Z position (m, negative for underwater).
            sigma_xy_m: Observation spatial uncertainty (m).
            stamp: Timestamp of the observation.

        Returns:
            The updated or newly created FusedContact.
        """
        # Guard against zero or negative sigma
        sigma = max(0.05, float(sigma_xy_m))
        w = 1.0 / (sigma * sigma)

        matched_contact: FusedContact | None = None

        # Rule 1: Check if track_id is already linked to an existing contact
        for c in self._contacts.values():
            if track_id in c.associated_tracks:
                matched_contact = c
                break

        # Rule 2: Gating with nearest same-class contact
        if matched_contact is None:
            best_dist = float("inf")
            for c in self._contacts.values():
                if c.class_id == class_id:
                    dx = x - c.x
                    dy = y - c.y
                    dist = math.sqrt(dx * dx + dy * dy)
                    gate = max(self.gate_m, 3.0 * c.sigma_xy_m)
                    if dist <= gate and dist < best_dist:
                        best_dist = dist
                        matched_contact = c

        # Rule 3: Create new contact if not matched
        if matched_contact is None:
            c_id = self._next_id
            self._next_id = (self._next_id + 1) % 256

            matched_contact = FusedContact(
                contact_id=c_id,
                class_id=class_id,
                confidence=float(confidence),
                x=float(x),
                y=float(y),
                z=float(z),
                sigma_xy_m=max(self.sigma_floor_m, sigma),
                depth_m=-float(z),
                first_seen_stamp=stamp,
                last_seen_stamp=stamp,
                sightings=1,
                associated_tracks={track_id},
                class_votes=[class_id],
                sum_inv_var=w,
                sum_wx=w * x,
                sum_wy=w * y,
                sum_wz=w * z,
            )
            self._contacts[c_id] = matched_contact
            return matched_contact

        # Fuse into existing contact
        matched_contact.associated_tracks.add(track_id)
        matched_contact.class_votes.append(class_id)
        matched_contact.class_id = Counter(matched_contact.class_votes).most_common(1)[
            0
        ][0]
        matched_contact.confidence = max(matched_contact.confidence, float(confidence))
        matched_contact.last_seen_stamp = stamp
        matched_contact.sightings += 1

        matched_contact.sum_inv_var += w
        matched_contact.sum_wx += w * x
        matched_contact.sum_wy += w * y
        matched_contact.sum_wz += w * z

        inv_sum = 1.0 / matched_contact.sum_inv_var
        matched_contact.x = matched_contact.sum_wx * inv_sum
        matched_contact.y = matched_contact.sum_wy * inv_sum
        matched_contact.z = matched_contact.sum_wz * inv_sum
        matched_contact.depth_m = -matched_contact.z

        raw_sigma = math.sqrt(inv_sum)
        matched_contact.sigma_xy_m = max(self.sigma_floor_m, raw_sigma)

        return matched_contact

    def clear(self) -> None:
        """Clear database."""
        self._contacts.clear()
        self._next_id = 0

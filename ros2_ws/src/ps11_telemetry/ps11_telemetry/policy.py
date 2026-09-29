"""Pure logic candidate scoring and telemetry policy (§11.7).

ROS-free module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ps11_telemetry.codec import ContactReport, Heartbeat


@dataclass
class CandidateContact:
    """Internal representation of a detected contact candidate."""

    id: int
    class_id: int
    confidence: float
    x_m: float
    y_m: float
    depth_m: float
    sigma_m: float
    last_seen_s: float


@dataclass
class SentContactState:
    """Historical state of a transmitted contact."""

    x_m: float
    y_m: float
    depth_m: float
    sigma_m: float
    class_id: int
    sent_time_s: float


@dataclass
class VehicleState:
    """Vehicle state for heartbeat generation."""

    x_m: float = 0.0
    y_m: float = 0.0
    depth_m: float = 0.0
    heading_deg: float = 0.0
    state: int = 0
    mission_start_s: float = 0.0


class SemanticPolicy:
    """Semantic telemetry scheduling policy (§11.7).

    Prioritizes contacts by class priority, confidence, novelty, and age boost.
    Enforces maximum and minimum heartbeat periods and energy-saving silence.
    """

    def __init__(
        self,
        class_priorities: dict[int, float] | None = None,
        hb_max_period_s: float = 15.0,
        hb_min_period_s: float = 5.0,
        update_min_move_m: float = 1.0,
        update_sigma_ratio: float = 0.5,
        update_novelty: float = 0.3,
        age_boost_tau_s: float = 30.0,
        age_boost_max_s: float = 60.0,
        battery_drain_per_hour: float = 0.1,
        mission_start_s: float = 0.0,
    ) -> None:
        # Default priorities from classes.yaml if none provided
        self.class_priorities = (
            class_priorities
            if class_priorities is not None
            else {0: 1.0, 1: 0.3, 2: 0.3, 3: 0.2}
        )
        self.hb_max_period_s = hb_max_period_s
        self.hb_min_period_s = hb_min_period_s
        self.update_min_move_m = update_min_move_m
        self.update_sigma_ratio = update_sigma_ratio
        self.update_novelty = update_novelty
        self.age_boost_tau_s = age_boost_tau_s
        self.age_boost_max_s = age_boost_max_s
        self.battery_drain_per_hour = battery_drain_per_hour
        self.mission_start_s = mission_start_s

        # Tracking state
        self.sent_contacts: dict[int, SentContactState] = {}
        # candidate_queue: contact_id -> (CandidateContact, become_candidate_time_s, is_update)
        self.candidate_queue: dict[int, tuple[CandidateContact, float, bool]] = {}
        self.last_heartbeat_s: float | None = None

    def update_contacts(self, contacts: list[CandidateContact], now_s: float) -> None:
        """Update candidate pool from fresh /vehicle/contacts."""
        for c in contacts:
            if c.id not in self.sent_contacts:
                # New contact
                if c.id in self.candidate_queue:
                    # Update contact data, keep original candidate arrival time
                    _, arr_time, _ = self.candidate_queue[c.id]
                    self.candidate_queue[c.id] = (c, arr_time, False)
                else:
                    self.candidate_queue[c.id] = (c, now_s, False)
            else:
                # Already sent; check update criteria
                sent = self.sent_contacts[c.id]
                dx = c.x_m - sent.x_m
                dy = c.y_m - sent.y_m
                dz = c.depth_m - sent.depth_m
                move_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                is_moved = move_dist >= self.update_min_move_m
                is_sigma_halved = c.sigma_m <= (
                    sent.sigma_m * self.update_sigma_ratio + 1e-6
                )
                is_class_changed = c.class_id != sent.class_id

                if is_moved or is_sigma_halved or is_class_changed:
                    if c.id in self.candidate_queue:
                        _, arr_time, _ = self.candidate_queue[c.id]
                        self.candidate_queue[c.id] = (c, arr_time, True)
                    else:
                        self.candidate_queue[c.id] = (c, now_s, True)
                else:
                    # Does not qualify for update
                    if c.id in self.candidate_queue and self.candidate_queue[c.id][2]:
                        del self.candidate_queue[c.id]

    def compute_candidate_score(
        self,
        contact: CandidateContact,
        become_candidate_time_s: float,
        is_update: bool,
        now_s: float,
    ) -> float:
        """Compute candidate priority score (§11.7)."""
        priority = self.class_priorities.get(contact.class_id, 0.1)
        age_s = max(0.0, now_s - become_candidate_time_s)
        age_boost = 1.0 + min(age_s, self.age_boost_max_s) / self.age_boost_tau_s
        novelty = self.update_novelty if is_update else 1.0
        return priority * contact.confidence * novelty * age_boost

    def compute_battery_frac(self, now_s: float) -> float:
        """Compute simulated linear battery drain (§11.2)."""
        elapsed_hours = max(0.0, now_s - self.mission_start_s) / 3600.0
        frac = 1.0 - (elapsed_hours * self.battery_drain_per_hour)
        return max(0.0, min(1.0, frac))

    def select_messages_for_frame(
        self,
        k_slots: int,
        now_s: float,
        vehicle: VehicleState,
    ) -> list[Heartbeat | ContactReport]:
        """Select up to k_slots messages for the next outgoing frame (§11.7)."""
        if self.last_heartbeat_s is None:
            # Initialize heartbeat baseline to now_s so contact candidates can go first
            self.last_heartbeat_s = now_s

        slots_left = k_slots
        selected: list[Heartbeat | ContactReport] = []

        # 1. Mandatory heartbeat if now - last_heartbeat >= hb_max_period_s (15 s)
        time_since_hb = now_s - self.last_heartbeat_s
        if time_since_hb >= self.hb_max_period_s and slots_left > 0:
            hb = self._create_heartbeat(now_s, vehicle)
            selected.append(hb)
            self.last_heartbeat_s = now_s
            slots_left -= 1

        # 2. Fill remaining slots with highest-scoring candidates
        if slots_left > 0 and self.candidate_queue:
            # Score all candidates
            scored: list[tuple[float, int, CandidateContact, bool]] = []
            for cid, (cand, arr_time, is_update) in self.candidate_queue.items():
                score = self.compute_candidate_score(cand, arr_time, is_update, now_s)
                scored.append((score, cid, cand, is_update))

            # Sort descending by score; secondary tie-breaker by candidate ID
            scored.sort(key=lambda item: (item[0], -item[1]), reverse=True)

            for score, cid, cand, is_update in scored[:slots_left]:
                report = ContactReport(
                    contact_id=cand.id,
                    class_id=cand.class_id,
                    confidence=cand.confidence,
                    x_m=cand.x_m,
                    y_m=cand.y_m,
                    depth_m=cand.depth_m,
                    sigma_m=cand.sigma_m,
                    t_s=round(cand.last_seen_s),
                    is_update=is_update,
                )
                selected.append(report)
                # Record sent state
                self.sent_contacts[cand.id] = SentContactState(
                    x_m=cand.x_m,
                    y_m=cand.y_m,
                    depth_m=cand.depth_m,
                    sigma_m=cand.sigma_m,
                    class_id=cand.class_id,
                    sent_time_s=now_s,
                )
                del self.candidate_queue[cid]
                slots_left -= 1

        # 3. If no candidate remains (or frame empty) and now - last_hb >= hb_min_period_s (5 s)
        if (
            len(self.candidate_queue) == 0
            and slots_left > 0
            and (now_s - self.last_heartbeat_s >= self.hb_min_period_s)
            and not any(isinstance(m, Heartbeat) for m in selected)
        ):
            hb = self._create_heartbeat(now_s, vehicle)
            selected.append(hb)
            self.last_heartbeat_s = now_s
            slots_left -= 1

        # If selected is empty, return empty list (send nothing to save energy)
        return selected

    def _create_heartbeat(self, now_s: float, vehicle: VehicleState) -> Heartbeat:
        pending = min(63, len(self.candidate_queue))
        battery_frac = self.compute_battery_frac(now_s)
        return Heartbeat(
            t_s=round(now_s),
            x_m=vehicle.x_m,
            y_m=vehicle.y_m,
            depth_m=vehicle.depth_m,
            heading_deg=vehicle.heading_deg,
            battery_frac=battery_frac,
            state=vehicle.state,
            pending=pending,
        )

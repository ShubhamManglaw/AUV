"""Semantic scheduling policy for T3.3 (plan §11.7) — ROS-free.

Decides, on each link tx_ready, whether the 8-byte frame carries a HEARTBEAT
or the highest-scoring CONTACT, using the frozen codec for encoding. Pure
logic: no rclpy imports; the ROS node feeds plain data in and publishes the
returned bytes.

Policy (plan §11.7, fill the frame's k slots in order):
1. If now - last_heartbeat >= hb_max_period_s: add a HEARTBEAT to the frame.
2. Fill remaining slots with the highest-scoring candidates:
   - new contact (never sent): score = priority[class] * confidence * 1.0 * age_boost
   - update (already sent AND moved >= update_min_move_m, or sigma <=
     update_sigma_ratio * sent sigma, or class changed): score = priority *
     confidence * update_novelty * age_boost
   - unchanged sent contacts are NOT eligible (no repeat spam)
   - age_boost = 1 + min(age_s, age_boost_max_s) / age_boost_tau_s
3. If no candidate remains and now - last_heartbeat >= hb_min_period_s: HEARTBEAT.
4. If the frame would still be empty: send nothing (saves energy).
Sent per-contact state (position, sigma, class) is remembered for update
judgement. Ties break on the lower contact id (deterministic).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ps11_telemetry.codec import ContactReport, Heartbeat, pack_frame


@dataclass(frozen=True)
class ContactInput:
    """Plain-data contact as received from /vehicle/contacts.

    t_last_seen_s: time of the last observation (CONTACT field t, §11.3);
    None falls back to the frame decision time.
    """

    contact_id: int
    class_id: int
    confidence: float
    x_m: float
    y_m: float
    depth_m: float
    sigma_xy_m: float
    t_last_seen_s: float | None = None


@dataclass(frozen=True)
class VehicleState:
    """Plain-data vehicle snapshot for HEARTBEAT fields."""

    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    yaw_rad: float = 0.0
    mission_state: int = 0


@dataclass(frozen=True)
class FrameDecision:
    """What the scheduler decided for one tx_ready."""

    payload: bytes | None
    kind: str | None  # "heartbeat" | "contact" | None
    skipped: tuple[int, ...]  # contact ids rejected by codec range rules


def compass_heading_deg(yaw_rad: float) -> float:
    """ENU yaw -> compass heading (§11.2): (90 deg - yaw) mod 360."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class SemanticPolicy:
    """M1 semantic scheduler state machine (plan §11.7)."""

    def __init__(
        self,
        cfg: dict,
        class_priorities: dict[int, float],
        frame_payload_bytes: int,
    ) -> None:
        hb = cfg["heartbeat"]
        sc = cfg["scoring"]
        self._hb_max = float(hb["max_period_s"])
        self._hb_min = float(hb["min_period_s"])
        self._update_min_move = float(sc["update_min_move_m"])
        self._update_sigma_ratio = float(sc["update_sigma_ratio"])
        self._update_novelty = float(sc["update_novelty"])
        self._age_tau = float(sc["age_boost_tau_s"])
        self._age_max = float(sc["age_boost_max_s"])
        self._drain_per_hour = float(cfg["battery"]["drain_per_hour"])
        self._mission_start = float(cfg.get("mission_start_s", 0.0))
        self._priorities = dict(class_priorities)
        self._frame_payload_bytes = int(frame_payload_bytes)

        self._candidates: dict[int, dict] = {}
        self._sent: dict[int, dict] = {}
        self._last_heartbeat_s: float | None = None
        self._seq = 0

    @property
    def seq(self) -> int:
        return self._seq

    def update_contacts(
        self, contacts: list[ContactInput], now_s: float
    ) -> tuple[int, ...]:
        """Upsert candidates from /vehicle/contacts. Returns skipped ids whose
        values violate the frozen codec's identifier ranges (0-255 / 0-7)."""
        skipped: list[int] = []
        for c in contacts:
            if not (0 <= c.contact_id <= 255) or not (0 <= c.class_id <= 7):
                skipped.append(c.contact_id)
                continue
            if c.contact_id not in self._candidates:
                self._candidates[c.contact_id] = {
                    "first_seen_s": now_s,
                    "contact": c,
                }
            else:
                self._candidates[c.contact_id]["contact"] = c
        return tuple(sorted(set(skipped)))

    def _age_boost(self, now_s: float, first_seen_s: float) -> float:
        age_s = max(now_s - first_seen_s, 0.0)
        return 1.0 + min(age_s, self._age_max) / self._age_tau

    def _eligible(self, now_s: float) -> list[tuple[float, int, float]]:
        """Eligible candidates as (score, contact_id, novelty) sorted for
        deterministic selection: highest score, then lowest contact id."""
        out: list[tuple[float, int, float]] = []
        for cid, state in self._candidates.items():
            contact: ContactInput = state["contact"]
            priority = self._priorities.get(contact.class_id, 0.0)
            sent = self._sent.get(cid)
            if sent is None:
                novelty = 1.0
            else:
                moved = math.hypot(contact.x_m - sent["x"], contact.y_m - sent["y"])
                sigma_improved = contact.sigma_xy_m <= (
                    self._update_sigma_ratio * sent["sigma"]
                )
                class_changed = contact.class_id != sent["class_id"]
                if (
                    moved < self._update_min_move
                    and not sigma_improved
                    and not class_changed
                ):
                    continue  # unchanged sent contact: suppressed
                novelty = self._update_novelty
            score = (
                priority
                * contact.confidence
                * novelty
                * self._age_boost(now_s, state["first_seen_s"])
            )
            out.append((score, cid, novelty))
        out.sort(key=lambda item: (-item[0], item[1]))
        return out

    def _t_s(self, now_s: float, when_s: float | None = None) -> int:
        """Whole seconds since mission start (§11.1); defaults to now."""
        when = now_s if when_s is None else when_s
        return max(int(when - self._mission_start), 0)

    def _heartbeat_msg(
        self, now_s: float, vehicle: VehicleState, pending: int
    ) -> Heartbeat:
        self._last_heartbeat_s = now_s
        return Heartbeat(
            t_s=self._t_s(now_s),
            x_m=vehicle.x_m,
            y_m=vehicle.y_m,
            depth_m=-vehicle.z_m,
            heading_deg=compass_heading_deg(vehicle.yaw_rad),
            battery_frac=max(
                0.0,
                1.0
                - self._drain_per_hour * max(now_s - self._mission_start, 0.0) / 3600.0,
            ),
            state=vehicle.mission_state,
            pending=min(pending, 63),
        )

    def on_frame(self, now_s: float, vehicle: VehicleState) -> FrameDecision:
        """Decide one frame on a link tx_ready (plan §11.7 steps 1-4)."""
        k = self._frame_payload_bytes // 8
        eligible = self._eligible(now_s)
        pending = len(eligible)
        msgs: list[Heartbeat | ContactReport] = []

        # Step 1: heartbeat when the max period elapsed (or none ever sent).
        hb_due_max = (
            self._last_heartbeat_s is None
            or (now_s - self._last_heartbeat_s) >= self._hb_max
        )
        if hb_due_max:
            msgs.append(self._heartbeat_msg(now_s, vehicle, pending))

        # Step 2: remaining slots go to the highest-scoring candidates.
        for score, cid, _novelty in eligible[: k - len(msgs)]:
            contact: ContactInput = self._candidates[cid]["contact"]
            msgs.append(
                ContactReport(
                    contact_id=cid,
                    class_id=contact.class_id,
                    confidence=contact.confidence,
                    x_m=contact.x_m,
                    y_m=contact.y_m,
                    depth_m=contact.depth_m,
                    sigma_m=contact.sigma_xy_m,
                    t_s=self._t_s(now_s, contact.t_last_seen_s),
                    is_update=cid in self._sent,
                )
            )
            self._sent[cid] = {
                "x": contact.x_m,
                "y": contact.y_m,
                "sigma": contact.sigma_xy_m,
                "class_id": contact.class_id,
            }

        # Step 3: no candidate remains and the min heartbeat period elapsed.
        if not eligible and not msgs:
            hb_due_min = (
                self._last_heartbeat_s is not None
                and (now_s - self._last_heartbeat_s) >= self._hb_min
            )
            if hb_due_min:
                msgs.append(self._heartbeat_msg(now_s, vehicle, pending))

        # Step 4: an empty frame sends nothing (saves energy).
        if not msgs:
            return FrameDecision(None, None, ())

        self._seq += 1
        kind = "heartbeat" if isinstance(msgs[0], Heartbeat) else "contact"
        return FrameDecision(pack_frame(msgs, self._frame_payload_bytes), kind, ())

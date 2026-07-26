"""Acoustic telemetry 8-byte bit-packed codec (§11.1–§11.5).

Pure-Python module with no ROS dependencies.
"""

import logging
import math
from dataclasses import dataclass
from typing import Any

logger = logging.getLogger(__name__)

# Constants
TYPE_PADDING = 0
TYPE_HEARTBEAT = 1
TYPE_CONTACT = 2
TYPE_EXTENDED = 3

# Sigma quantization thresholds and bucket upper bounds (§11.3)
SIGMA_THRESHOLDS = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0)
SIGMA_DECODE_BOUNDS = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0)


@dataclass(frozen=True)
class Heartbeat:
    """HEARTBEAT message (type 1)."""

    t_s: int
    x_m: float
    y_m: float
    depth_m: float
    heading_deg: float
    battery_frac: float
    state: int
    pending: int


@dataclass(frozen=True)
class ContactReport:
    """CONTACT message (type 2)."""

    contact_id: int
    class_id: int
    confidence: float
    x_m: float
    y_m: float
    depth_m: float
    sigma_m: float
    t_s: int
    is_update: bool


def round_half_away_from_zero(val: float) -> int:
    """Round float to nearest integer with ties rounded away from zero."""
    return math.floor(val + 0.5) if val >= 0 else math.ceil(val - 0.5)


def _clamp(val: float, min_val: float, max_val: float, field_name: str) -> float:
    if val < min_val:
        logger.warning(
            "Field '%s' value %s < min %s; clamping.", field_name, val, min_val
        )
        return min_val
    if val > max_val:
        logger.warning(
            "Field '%s' value %s > max %s; clamping.", field_name, val, max_val
        )
        return max_val
    return val


def _encode_sigma(sigma_m: float) -> int:
    for idx, thresh in enumerate(SIGMA_THRESHOLDS):
        if sigma_m <= thresh:
            return idx
    return 7


def _decode_sigma(bucket: int) -> float:
    if 0 <= bucket < len(SIGMA_DECODE_BOUNDS):
        return SIGMA_DECODE_BOUNDS[bucket]
    return 32.0


def encode(msg: Heartbeat | ContactReport) -> bytes:
    """Encode a Heartbeat or ContactReport into exactly 8 bytes (MSB-first, big-endian)."""
    val = 0

    if isinstance(msg, Heartbeat):
        # Validate non-clamped identifiers (§11.1)
        if not (0 <= msg.state <= 7):
            raise ValueError(f"Heartbeat.state must be in range 0–7, got {msg.state}")

        # type: 2 bits (1 = 01)
        val = (val << 2) | TYPE_HEARTBEAT

        # t: 11 bits (mod 2048)
        t_enc = msg.t_s % 2048
        val = (val << 11) | t_enc

        # x: 12 bits signed, 0.5 m resolution, clamp [-2048, 2047]
        x_steps = round_half_away_from_zero(msg.x_m / 0.5)
        x_clamped = int(_clamp(x_steps, -2048, 2047, "Heartbeat.x_m"))
        val = (val << 12) | (x_clamped & 0xFFF)

        # y: 12 bits signed, 0.5 m resolution, clamp [-2048, 2047]
        y_steps = round_half_away_from_zero(msg.y_m / 0.5)
        y_clamped = int(_clamp(y_steps, -2048, 2047, "Heartbeat.y_m"))
        val = (val << 12) | (y_clamped & 0xFFF)

        # depth: 8 bits unsigned, 0.5 m resolution, clamp [0, 255]
        depth_steps = round_half_away_from_zero(msg.depth_m / 0.5)
        depth_clamped = int(_clamp(depth_steps, 0, 255, "Heartbeat.depth_m"))
        val = (val << 8) | depth_clamped

        # heading: 6 bits unsigned, 5.625°/step, mod 64
        heading_steps = round_half_away_from_zero(msg.heading_deg / 5.625) % 64
        val = (val << 6) | heading_steps

        # battery: 4 bits unsigned, round(frac * 15), clamp [0, 15]
        bat_steps = round_half_away_from_zero(msg.battery_frac * 15.0)
        bat_clamped = int(_clamp(bat_steps, 0, 15, "Heartbeat.battery_frac"))
        val = (val << 4) | bat_clamped

        # state: 3 bits unsigned
        val = (val << 3) | msg.state

        # pending: 6 bits unsigned, saturates at 63 without warning
        pending_clamped = max(0, min(msg.pending, 63))
        val = (val << 6) | pending_clamped

    elif isinstance(msg, ContactReport):
        # Validate non-clamped identifiers (§11.1)
        if not (0 <= msg.contact_id <= 255):
            raise ValueError(
                f"ContactReport.contact_id must be in range 0–255, got {msg.contact_id}"
            )
        if not (0 <= msg.class_id <= 7):
            raise ValueError(
                f"ContactReport.class_id must be in range 0–7, got {msg.class_id}"
            )

        # type: 2 bits (2 = 10)
        val = (val << 2) | TYPE_CONTACT

        # contact_id: 8 bits unsigned
        val = (val << 8) | msg.contact_id

        # class_id: 3 bits unsigned
        val = (val << 3) | msg.class_id

        # confidence: 3 bits unsigned, round(conf * 7), clamp [0, 7]
        conf_steps = round_half_away_from_zero(msg.confidence * 7.0)
        conf_clamped = int(_clamp(conf_steps, 0, 7, "ContactReport.confidence"))
        val = (val << 3) | conf_clamped

        # x: 12 bits signed, 0.5 m resolution, clamp [-2048, 2047]
        x_steps = round_half_away_from_zero(msg.x_m / 0.5)
        x_clamped = int(_clamp(x_steps, -2048, 2047, "ContactReport.x_m"))
        val = (val << 12) | (x_clamped & 0xFFF)

        # y: 12 bits signed, 0.5 m resolution, clamp [-2048, 2047]
        y_steps = round_half_away_from_zero(msg.y_m / 0.5)
        y_clamped = int(_clamp(y_steps, -2048, 2047, "ContactReport.y_m"))
        val = (val << 12) | (y_clamped & 0xFFF)

        # depth: 8 bits unsigned, 0.5 m resolution, clamp [0, 255]
        depth_steps = round_half_away_from_zero(msg.depth_m / 0.5)
        depth_clamped = int(_clamp(depth_steps, 0, 255, "ContactReport.depth_m"))
        val = (val << 8) | depth_clamped

        # sigma: 3 bits unsigned bucket
        sigma_bucket = _encode_sigma(msg.sigma_m)
        val = (val << 3) | sigma_bucket

        # t: 11 bits unsigned, s mod 2048
        t_enc = msg.t_s % 2048
        val = (val << 11) | t_enc

        # flags: 2 bits (bit 1 = UPDATE, bit 0 = reserved 0)
        flags = (1 << 1) if msg.is_update else 0
        val = (val << 2) | flags

    else:
        raise TypeError(f"Unsupported message type: {type(msg)}")

    return val.to_bytes(8, byteorder="big")


def decode(data: bytes) -> Heartbeat | ContactReport | None:
    """Decode an 8-byte message. Returns None for padding or unknown EXTENDED."""
    if len(data) != 8:
        raise ValueError(f"Message length must be exactly 8 bytes, got {len(data)}")

    val = int.from_bytes(data, byteorder="big")
    msg_type = (val >> 62) & 0x3

    if msg_type == TYPE_PADDING:
        return None

    if msg_type == TYPE_EXTENDED:
        # Reserved for M3 thumbnails/uplink; decoders skip without error
        return None

    if msg_type == TYPE_HEARTBEAT:
        t_s = (val >> 51) & 0x7FF
        raw_x = (val >> 39) & 0xFFF
        x_steps = raw_x - 4096 if raw_x & 0x800 else raw_x
        x_m = x_steps * 0.5

        raw_y = (val >> 27) & 0xFFF
        y_steps = raw_y - 4096 if raw_y & 0x800 else raw_y
        y_m = y_steps * 0.5

        depth_steps = (val >> 19) & 0xFF
        depth_m = depth_steps * 0.5

        heading_steps = (val >> 13) & 0x3F
        heading_deg = heading_steps * 5.625

        bat_steps = (val >> 9) & 0xF
        battery_frac = bat_steps / 15.0

        state = (val >> 6) & 0x7
        pending = val & 0x3F

        return Heartbeat(
            t_s=t_s,
            x_m=x_m,
            y_m=y_m,
            depth_m=depth_m,
            heading_deg=heading_deg,
            battery_frac=battery_frac,
            state=state,
            pending=pending,
        )

    if msg_type == TYPE_CONTACT:
        contact_id = (val >> 54) & 0xFF
        class_id = (val >> 51) & 0x7
        conf_steps = (val >> 48) & 0x7
        confidence = conf_steps / 7.0

        raw_x = (val >> 36) & 0xFFF
        x_steps = raw_x - 4096 if raw_x & 0x800 else raw_x
        x_m = x_steps * 0.5

        raw_y = (val >> 24) & 0xFFF
        y_steps = raw_y - 4096 if raw_y & 0x800 else raw_y
        y_m = y_steps * 0.5

        depth_steps = (val >> 16) & 0xFF
        depth_m = depth_steps * 0.5

        sigma_bucket = (val >> 13) & 0x7
        sigma_m = _decode_sigma(sigma_bucket)

        t_s = (val >> 2) & 0x7FF
        flags = val & 0x3
        is_update = bool((flags >> 1) & 1)

        return ContactReport(
            contact_id=contact_id,
            class_id=class_id,
            confidence=confidence,
            x_m=x_m,
            y_m=y_m,
            depth_m=depth_m,
            sigma_m=sigma_m,
            t_s=t_s,
            is_update=is_update,
        )

    return None


def pack_frame(msgs: list[Any], frame_payload_bytes: int) -> bytes:
    """Pack a list of Heartbeat or ContactReport messages into a fixed-size frame payload."""
    k = frame_payload_bytes // 8
    chunks: list[bytes] = []

    for msg in msgs[:k]:
        chunks.append(encode(msg))

    # Pad remaining slots with all-zero bytes (TYPE_PADDING)
    while len(chunks) < k:
        chunks.append(b"\x00" * 8)

    return b"".join(chunks)


def unpack_frame(data: bytes) -> list[Heartbeat | ContactReport]:
    """Unpack a frame payload into a list of messages, skipping padding and unknown extended."""
    if len(data) % 8 != 0:
        raise ValueError(f"Frame payload length {len(data)} is not a multiple of 8")

    result: list[Heartbeat | ContactReport] = []
    for i in range(0, len(data), 8):
        chunk = data[i : i + 8]
        msg = decode(chunk)
        if msg is not None:
            result.append(msg)

    return result


def unwrap_time(t_mod: int, now_s: float, modulus: int = 2048) -> int:
    """Unwrap a modulo-timestamp.

    Chooses candidate k * modulus + t_mod closest to now_s,
    subject to candidate <= now_s + 5.0 (never more than 5s in future).
    """
    base_k = round(now_s / modulus)
    candidates: list[int] = []

    # Check neighborhood around base_k
    for k in (base_k - 2, base_k - 1, base_k, base_k + 1, base_k + 2):
        c = k * modulus + t_mod
        if c <= now_s + 5.0:
            candidates.append(c)

    if not candidates:
        # Fallback to closest candidate regardless of future constraint if none qualify
        return min(
            (k * modulus + t_mod for k in (base_k - 1, base_k, base_k + 1)),
            key=lambda c: abs(c - now_s),
        )

    return min(candidates, key=lambda c: abs(c - now_s))

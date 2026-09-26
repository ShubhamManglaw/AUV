"""Tests for telemetry codec (§11.1–§11.5)."""

import json
import logging
from pathlib import Path

import pytest
from ps11_telemetry.codec import (
    ContactReport,
    Heartbeat,
    decode,
    encode,
    pack_frame,
    round_half_away_from_zero,
    unpack_frame,
    unwrap_time,
)


def test_round_half_away_from_zero_half_steps() -> None:
    """Verify round-half-away-from-zero at exact half steps per §11.1."""
    # x=0.25 m / 0.5 -> 0.5 -> step 1
    assert round_half_away_from_zero(0.25 / 0.5) == 1
    # x=0.75 m / 0.5 -> 1.5 -> step 2
    assert round_half_away_from_zero(0.75 / 0.5) == 2
    # x=1.25 m / 0.5 -> 2.5 -> step 3
    assert round_half_away_from_zero(1.25 / 0.5) == 3
    # x=-0.25 m / 0.5 -> -0.5 -> step -1
    assert round_half_away_from_zero(-0.25 / 0.5) == -1
    # x=-0.75 m / 0.5 -> -1.5 -> step -2
    assert round_half_away_from_zero(-0.75 / 0.5) == -2


def test_identifier_validation() -> None:
    """Identifiers (contact_id, class_id, state) must not be clamped and must raise ValueError."""
    # Invalid contact_id < 0 or > 255
    with pytest.raises(ValueError, match="contact_id"):
        encode(
            ContactReport(
                contact_id=-1,
                class_id=0,
                confidence=0.5,
                x_m=0.0,
                y_m=0.0,
                depth_m=0.0,
                sigma_m=1.0,
                t_s=0,
                is_update=False,
            )
        )

    with pytest.raises(ValueError, match="contact_id"):
        encode(
            ContactReport(
                contact_id=256,
                class_id=0,
                confidence=0.5,
                x_m=0.0,
                y_m=0.0,
                depth_m=0.0,
                sigma_m=1.0,
                t_s=0,
                is_update=False,
            )
        )

    # Invalid class_id < 0 or > 7
    with pytest.raises(ValueError, match="class_id"):
        encode(
            ContactReport(
                contact_id=1,
                class_id=-1,
                confidence=0.5,
                x_m=0.0,
                y_m=0.0,
                depth_m=0.0,
                sigma_m=1.0,
                t_s=0,
                is_update=False,
            )
        )

    with pytest.raises(ValueError, match="class_id"):
        encode(
            ContactReport(
                contact_id=1,
                class_id=8,
                confidence=0.5,
                x_m=0.0,
                y_m=0.0,
                depth_m=0.0,
                sigma_m=1.0,
                t_s=0,
                is_update=False,
            )
        )

    # Invalid state < 0 or > 7
    with pytest.raises(ValueError, match="state"):
        encode(
            Heartbeat(
                t_s=0,
                x_m=0.0,
                y_m=0.0,
                depth_m=0.0,
                heading_deg=0.0,
                battery_frac=1.0,
                state=-1,
                pending=0,
            )
        )

    with pytest.raises(ValueError, match="state"):
        encode(
            Heartbeat(
                t_s=0,
                x_m=0.0,
                y_m=0.0,
                depth_m=0.0,
                heading_deg=0.0,
                battery_frac=1.0,
                state=8,
                pending=0,
            )
        )


def test_clamping_and_warnings() -> None:
    """Clamping with warning applies only to physical values: x, y, depth, confidence, battery."""
    log_msgs: list[str] = []

    class TestHandler(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            log_msgs.append(record.getMessage())

    codec_logger = logging.getLogger("ps11_telemetry.codec")
    handler = TestHandler()
    codec_logger.addHandler(handler)
    try:
        c = ContactReport(
            contact_id=255,
            class_id=0,
            confidence=1.5,
            x_m=1500.0,
            y_m=-2000.0,
            depth_m=200.0,
            sigma_m=50.0,
            t_s=10,
            is_update=False,
        )
        encoded = encode(c)
        assert len(encoded) == 8
        assert len(log_msgs) > 0
        assert any("clamping" in m for m in log_msgs)

        dec = decode(encoded)
        assert isinstance(dec, ContactReport)
        assert dec.contact_id == 255
        assert dec.class_id == 0
        assert dec.confidence == 1.0
        assert dec.x_m == 1023.5
        assert dec.y_m == -1024.0
        assert dec.depth_m == 127.5
        assert dec.sigma_m == 32.0
    finally:
        codec_logger.removeHandler(handler)


def test_pending_saturates_without_warning() -> None:
    """Pending field saturates at 63 without warning."""
    log_msgs: list[str] = []

    class TestHandler(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            log_msgs.append(record.getMessage())

    codec_logger = logging.getLogger("ps11_telemetry.codec")
    handler = TestHandler()
    codec_logger.addHandler(handler)
    try:
        hb = Heartbeat(
            t_s=0,
            x_m=0.0,
            y_m=0.0,
            depth_m=0.0,
            heading_deg=0.0,
            battery_frac=1.0,
            state=1,
            pending=100,
        )
        encoded = encode(hb)
        dec = decode(encoded)
        assert isinstance(dec, Heartbeat)
        assert dec.pending == 63
        assert not any("pending" in m for m in log_msgs)
    finally:
        codec_logger.removeHandler(handler)


def test_reference_contact_vector() -> None:
    c = ContactReport(
        contact_id=5,
        class_id=0,
        confidence=0.9,
        x_m=12.5,
        y_m=-3.0,
        depth_m=15.0,
        sigma_m=0.8,
        t_s=100,
        is_update=False,
    )
    encoded = encode(c)
    expected = bytes.fromhex("8146019ffa1e4190")
    assert encoded == expected, (
        f"Encoded: {encoded.hex()} != Expected: {expected.hex()}"
    )

    decoded = decode(encoded)
    assert isinstance(decoded, ContactReport)
    assert decoded.contact_id == 5
    assert decoded.class_id == 0
    assert abs(decoded.confidence - 6 / 7.0) < 1e-5
    assert decoded.x_m == 12.5
    assert decoded.y_m == -3.0
    assert decoded.depth_m == 15.0
    assert decoded.sigma_m == 1.0  # Bucket 2 upper bound
    assert decoded.t_s == 100
    assert not decoded.is_update


def test_reference_heartbeat_vector() -> None:
    hb = Heartbeat(
        t_s=100,
        x_m=12.5,
        y_m=-3.0,
        depth_m=12.5,
        heading_deg=90.0,
        battery_frac=0.8,
        state=2,
        pending=3,
    )
    encoded = encode(hb)
    expected = bytes.fromhex("43200cffd0ca1883")
    assert encoded == expected, (
        f"Encoded: {encoded.hex()} != Expected: {expected.hex()}"
    )

    decoded = decode(encoded)
    assert isinstance(decoded, Heartbeat)
    assert decoded.t_s == 100
    assert decoded.x_m == 12.5
    assert decoded.y_m == -3.0
    assert decoded.depth_m == 12.5
    assert decoded.heading_deg == 90.0
    assert abs(decoded.battery_frac - 12 / 15.0) < 1e-5
    assert decoded.state == 2
    assert decoded.pending == 3


def test_padding_and_extended() -> None:
    # All zeros is padding
    assert decode(bytes(8)) is None

    # Type 3 is EXTENDED -> decoders skip without error
    extended_bytes = (3 << 62).to_bytes(8, byteorder="big")
    assert decode(extended_bytes) is None

    # Invalid length raises ValueError
    with pytest.raises(ValueError, match="exactly 8 bytes"):
        decode(b"\x00" * 7)


def test_pack_and_unpack_frame() -> None:
    hb = Heartbeat(10, 0.0, 0.0, 5.0, 0.0, 1.0, 1, 0)
    c = ContactReport(1, 0, 0.8, 10.0, 20.0, 15.0, 0.5, 10, False)

    # 8-byte frame: 1 slot
    frame_8 = pack_frame([hb], frame_payload_bytes=8)
    assert len(frame_8) == 8
    unpacked_8 = unpack_frame(frame_8)
    assert len(unpacked_8) == 1
    assert isinstance(unpacked_8[0], Heartbeat)

    # 32-byte frame: 4 slots with 2 messages and 2 padding slots
    frame_32 = pack_frame([hb, c], frame_payload_bytes=32)
    assert len(frame_32) == 32
    unpacked_32 = unpack_frame(frame_32)
    assert len(unpacked_32) == 2
    assert isinstance(unpacked_32[0], Heartbeat)
    assert isinstance(unpacked_32[1], ContactReport)


def test_unwrap_time() -> None:
    # Basic unwrapping without wrap
    assert unwrap_time(t_mod=100, now_s=102.0) == 100

    # Unwrapping across the 2048 s boundary (now is 2050, received t_mod=4 -> 2052)
    assert unwrap_time(t_mod=4, now_s=2050.0) == 2052

    # Received t_mod=2047 when now is 2050 -> candidate 2047 (sent 3s ago)
    assert unwrap_time(t_mod=2047, now_s=2050.0) == 2047

    # Never more than 5s in future: now is 2040, t_mod=50 -> candidate 48s in future rejected, chooses 50 (from k=0)
    assert unwrap_time(t_mod=50, now_s=2040.0) == 50


def test_golden_vectors_file() -> None:
    """Load golden_vectors.json and verify encode(input) == hex AND decode(hex) == expected_decoded."""
    gv_path = Path(__file__).parent / "golden_vectors.json"
    assert gv_path.is_file(), f"golden_vectors.json not found at {gv_path}"

    with open(gv_path, "r", encoding="utf-8") as f:
        cases = json.load(f)

    assert len(cases) == 8

    for case in cases:
        cid = case["id"]
        name = case["name"]
        mtype = case["type"]
        expected_hex = case["hex"]
        inp = case["input"]
        exp_dec = case["expected_decoded"]

        expected_bytes = bytes.fromhex(expected_hex)
        assert len(expected_bytes) == 8, f"Case {cid} ({name}): length != 8"

        if mtype == "PADDING":
            assert inp is None
            assert exp_dec is None
            assert decode(expected_bytes) is None
        elif mtype == "HEARTBEAT":
            hb = Heartbeat(**inp)
            enc = encode(hb)
            assert enc.hex() == expected_hex, (
                f"Case {cid} ({name}): encode mismatch: {enc.hex()} != {expected_hex}"
            )
            dec = decode(expected_bytes)
            assert isinstance(dec, Heartbeat), (
                f"Case {cid} ({name}): decode type mismatch"
            )
            for field, expected_val in exp_dec.items():
                actual_val = getattr(dec, field)
                if isinstance(expected_val, float):
                    assert actual_val == pytest.approx(expected_val, rel=1e-5), (
                        f"Case {cid} ({name}): field {field} {actual_val} != {expected_val}"
                    )
                else:
                    assert actual_val == expected_val, (
                        f"Case {cid} ({name}): field {field} {actual_val} != {expected_val}"
                    )
        elif mtype == "CONTACT":
            c = ContactReport(**inp)
            enc = encode(c)
            assert enc.hex() == expected_hex, (
                f"Case {cid} ({name}): encode mismatch: {enc.hex()} != {expected_hex}"
            )
            dec = decode(expected_bytes)
            assert isinstance(dec, ContactReport), (
                f"Case {cid} ({name}): decode type mismatch"
            )
            for field, expected_val in exp_dec.items():
                actual_val = getattr(dec, field)
                if isinstance(expected_val, float):
                    assert actual_val == pytest.approx(expected_val, rel=1e-5), (
                        f"Case {cid} ({name}): field {field} {actual_val} != {expected_val}"
                    )
                else:
                    assert actual_val == expected_val, (
                        f"Case {cid} ({name}): field {field} {actual_val} != {expected_val}"
                    )

"""Tests for telemetry codec (§11.1–§11.5)."""

import json
from pathlib import Path

import pytest
from ps11_telemetry.codec import (
    ContactReport,
    Heartbeat,
    decode,
    encode,
    pack_frame,
    unpack_frame,
    unwrap_time,
)


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


def test_heartbeat_roundtrip_quantization() -> None:
    hb = Heartbeat(
        t_s=1234,
        x_m=100.2,  # round(100.2 / 0.5) * 0.5 = 100.0
        y_m=-45.8,  # round(-45.8 / 0.5) * 0.5 = -46.0
        depth_m=32.2,  # round(32.2 / 0.5) * 0.5 = 32.0
        heading_deg=182.0,  # 5.625 res: round(182 / 5.625) * 5.625 = 180.0
        battery_frac=0.55,  # 15 res: round(0.55 * 15) / 15 = 8/15
        state=4,
        pending=20,
    )
    encoded = encode(hb)
    assert len(encoded) == 8
    dec = decode(encoded)
    assert isinstance(dec, Heartbeat)

    assert dec.t_s == 1234
    assert abs(dec.x_m - hb.x_m) <= 0.25
    assert abs(dec.y_m - hb.y_m) <= 0.25
    assert abs(dec.depth_m - hb.depth_m) <= 0.25
    assert abs(dec.heading_deg - hb.heading_deg) <= 2.8125
    assert abs(dec.battery_frac - hb.battery_frac) <= (1.0 / 30.0)
    assert dec.state == 4
    assert dec.pending == 20


def test_contact_roundtrip_quantization() -> None:
    c = ContactReport(
        contact_id=128,
        class_id=3,
        confidence=0.45,  # round(0.45 * 7) = 3 -> 3/7
        x_m=-512.2,
        y_m=300.7,
        depth_m=50.1,
        sigma_m=3.5,  # bucket 4 (<= 4.0) -> decodes to 4.0
        t_s=2040,
        is_update=True,
    )
    encoded = encode(c)
    assert len(encoded) == 8
    dec = decode(encoded)
    assert isinstance(dec, ContactReport)

    assert dec.contact_id == 128
    assert dec.class_id == 3
    assert abs(dec.confidence - c.confidence) <= (1.0 / 14.0)
    assert abs(dec.x_m - c.x_m) <= 0.25
    assert abs(dec.y_m - c.y_m) <= 0.25
    assert abs(dec.depth_m - c.depth_m) <= 0.25
    assert dec.sigma_m == 4.0
    assert dec.t_s == 2040
    assert dec.is_update is True


def test_clamping_and_warnings() -> None:
    import logging

    log_msgs: list[str] = []

    class TestHandler(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            log_msgs.append(record.getMessage())

    codec_logger = logging.getLogger("ps11_telemetry.codec")
    handler = TestHandler()
    codec_logger.addHandler(handler)
    try:
        c = ContactReport(
            contact_id=300,  # clamp to 255
            class_id=10,  # clamp to 7
            confidence=1.5,  # clamp to 7 steps
            x_m=1500.0,  # clamp to 1023.5 m (2047 steps)
            y_m=-2000.0,  # clamp to -1024.0 m (-2048 steps)
            depth_m=200.0,  # clamp to 127.5 m (255 steps)
            sigma_m=50.0,  # bucket 7 (> 16m)
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
        assert dec.class_id == 7
        assert dec.confidence == 1.0
        assert dec.x_m == 1023.5
        assert dec.y_m == -1024.0
        assert dec.depth_m == 127.5
        assert dec.sigma_m == 32.0
    finally:
        codec_logger.removeHandler(handler)


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


def test_golden_vectors() -> None:
    # Find golden_vectors.json
    candidates = [
        Path("test/golden_vectors.json"),
        Path(__file__).parent / "golden_vectors.json",
        Path(__file__).parents[3] / "test" / "golden_vectors.json",
    ]
    gv_path = next((p for p in candidates if p.is_file()), None)
    assert gv_path is not None, "golden_vectors.json not found"

    with open(gv_path, "r", encoding="utf-8") as f:
        cases = json.load(f)

    assert len(cases) == 8

    for case in cases:
        name = case["name"]
        expected_hex = case["hex"]
        expected_bytes = bytes.fromhex(expected_hex)
        assert len(expected_bytes) == 8, f"{name}: length != 8"

        msg_data = case["msg"]
        if case["type"] == "PADDING":
            assert decode(expected_bytes) is None
        elif case["type"] == "HEARTBEAT":
            hb = Heartbeat(**msg_data)
            enc = encode(hb)
            assert enc.hex() == expected_hex, f"{name}: encode mismatch"
            dec = decode(expected_bytes)
            assert isinstance(dec, Heartbeat), f"{name}: decode type mismatch"
        elif case["type"] == "CONTACT":
            c = ContactReport(**msg_data)
            enc = encode(c)
            assert enc.hex() == expected_hex, f"{name}: encode mismatch"
            dec = decode(expected_bytes)
            assert isinstance(dec, ContactReport), f"{name}: decode type mismatch"

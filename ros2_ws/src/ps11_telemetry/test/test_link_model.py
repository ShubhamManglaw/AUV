"""Unit tests for LinkModel acoustic channel emulator (§11.6)."""

from ps11_telemetry.link_model import LinkModel


def test_60_frames_timing_m64() -> None:
    """60 frames at m64 take 60 s ± 1 s plus 0.5 s latency."""
    model = LinkModel(
        bitrate_bps=64,
        frame_payload_bytes=8,
        latency_s=0.5,
        loss_prob=0.0,
        mode="queue",
    )

    # Queue 60 frames at t = 0.0
    for seq in range(60):
        ok = model.send_frame(payload=b"\x01" * 8, seq=seq, now_s=0.0)
        assert ok

    assert model.queue_len == 59
    assert model.is_busy(0.0)

    # Step clock forward in 0.05 s increments
    t = 0.0
    dt = 0.05
    received: list[tuple[int, bytes, float]] = []
    while t <= 62.0:
        rx, _ = model.step(t)
        received.extend(rx)
        if len(received) == 60:
            break
        t += dt

    assert len(received) == 60
    # First frame arrives at t = 1.0 s airtime + 0.5 s latency = 1.5 s
    first_arrival = received[0][2]
    assert abs(first_arrival - 1.5) < 1e-4

    # 60th frame arrives at t = 59 * 1.0s + 1.0s airtime + 0.5s latency = 60.5 s
    last_arrival = received[-1][2]
    assert abs(last_arrival - 60.5) < 1e-4

    # Total duration is 60 s ± 1 s plus 0.5 s latency (60.5 s)
    total_duration = last_arrival - 0.0
    assert abs(total_duration - 60.5) <= 1.0


def test_loss_prob_over_1000_frames() -> None:
    """With fixed seed, loss is about 5% over 1000 frames."""
    model = LinkModel(
        bitrate_bps=64,
        frame_payload_bytes=8,
        latency_s=0.5,
        loss_prob=0.05,
        mode="queue",
        seed=42,
    )

    for seq in range(1000):
        model.send_frame(payload=b"\x01" * 8, seq=seq, now_s=0.0)

    # Advance time to deliver all frames (1000 * 1.0 s + 0.5 s = 1000.5 s)
    rx, _ = model.step(now_s=1001.0)
    stats = model.get_stats(now_s=1001.0)

    assert stats.frames_sent == 1000
    # About 5% of 1000 is 50; allow realistic statistical variance (3.5% - 6.5%)
    assert 35 <= stats.frames_lost <= 65
    assert len(rx) == 1000 - stats.frames_lost


def test_oversize_payload_rejected() -> None:
    """Oversize payloads are rejected and counted."""
    model = LinkModel(bitrate_bps=64, frame_payload_bytes=8)

    # 9 bytes > 8 bytes max
    ok = model.send_frame(payload=b"\x01" * 9, seq=1, now_s=0.0)
    assert not ok

    stats = model.get_stats(now_s=0.0)
    assert stats.frames_rejected == 1
    assert stats.frames_sent == 0
    assert stats.payload_bits_sent == 0


def test_queue_mode_busy_queuing() -> None:
    """Queue mode queues frames that arrive while busy."""
    model = LinkModel(
        bitrate_bps=64, frame_payload_bytes=8, latency_s=0.5, mode="queue"
    )

    # Send first frame at t = 0.0 (airtime 1.0 s -> busy until t = 1.0)
    model.send_frame(payload=b"\x01" * 8, seq=1, now_s=0.0)
    assert model.get_stats(0.0).queue_len == 0
    assert model.is_busy(0.0)

    # Second frame arrives while busy at t = 0.2
    model.send_frame(payload=b"\x02" * 8, seq=2, now_s=0.2)
    assert model.get_stats(0.2).queue_len == 1

    # Third frame arrives while busy at t = 0.5
    model.send_frame(payload=b"\x03" * 8, seq=3, now_s=0.5)
    assert model.get_stats(0.5).queue_len == 2

    # Advance clock to t = 1.0 s: first frame airtime completes, frame 2 starts
    model.step(now_s=1.0)
    assert model.get_stats(1.0).queue_len == 1

    # Advance clock to t = 2.0 s: second frame airtime completes, frame 3 starts
    model.step(now_s=2.0)
    assert model.get_stats(2.0).queue_len == 0


def test_pull_mode_tx_ready_repeats() -> None:
    """Pull mode publishes tx_ready when idle and repeats every 0.5 s."""
    model = LinkModel(bitrate_bps=64, frame_payload_bytes=8, latency_s=0.5, mode="pull")

    # At t = 0.0, channel is idle -> tx_ready triggers
    _rx, tx_ready = model.step(now_s=0.0)
    assert tx_ready is True

    # At t = 0.2, channel still idle, but < 0.5 s elapsed -> no tx_ready
    _rx, tx_ready = model.step(now_s=0.2)
    assert tx_ready is False

    # At t = 0.5, 0.5 s elapsed -> tx_ready repeats
    _rx, tx_ready = model.step(now_s=0.5)
    assert tx_ready is True

    # Scheduler sends a frame at t = 0.5 (airtime 1.0 s -> busy until t = 1.5)
    model.send_frame(payload=b"\x01" * 8, seq=1, now_s=0.5)

    # At t = 1.0, channel is busy -> no tx_ready
    _rx, tx_ready = model.step(now_s=1.0)
    assert tx_ready is False

    # At t = 1.5, airtime completes, channel becomes idle -> tx_ready fires immediately
    _rx, tx_ready = model.step(now_s=1.5)
    assert tx_ready is True

    # At t = 2.0, repeats while remaining idle
    _rx, tx_ready = model.step(now_s=2.0)
    assert tx_ready is True


def test_all_zero_payload_discarded_at_receiver() -> None:
    """All-zero payloads cost airtime and count as sent, but are discarded at receiver."""
    model = LinkModel(
        bitrate_bps=64,
        frame_payload_bytes=8,
        latency_s=0.5,
        loss_prob=0.0,
        mode="pull",
    )

    # Send 8 zero bytes
    model.send_frame(payload=b"\x00" * 8, seq=1, now_s=0.0)
    stats = model.get_stats(0.0)
    assert stats.frames_sent == 1
    assert stats.payload_bits_sent == 64
    assert model.is_busy(0.0)

    # Step to arrival time at t = 1.5 s
    rx, _ = model.step(now_s=1.5)
    # Discarded on receiving side (reserved for sync)
    assert len(rx) == 0

    stats = model.get_stats(1.5)
    assert stats.frames_sent == 1
    assert stats.frames_lost == 0
    assert stats.frames_rejected == 0

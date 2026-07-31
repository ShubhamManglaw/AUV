"""Acoustic link channel emulator model (§11.6).

Pure-Python module with no ROS dependencies.
"""

import logging
import random
from collections import deque
from dataclasses import dataclass
from typing import NamedTuple

logger = logging.getLogger(__name__)


class LinkStatsData(NamedTuple):
    """Link statistics snapshot."""

    profile_label: str
    payload_bits_sent: int
    frames_sent: int
    frames_lost: int
    frames_rejected: int
    queue_len: int
    utilisation: float


@dataclass
class _InFlightFrame:
    seq: int
    payload: bytes
    arrival_time_s: float
    is_lost: bool


class LinkModel:
    """Simulates an acoustic communication channel with latency, loss, and half-duplex airtime."""

    def __init__(
        self,
        bitrate_bps: int = 64,
        frame_payload_bytes: int = 8,
        latency_s: float = 0.5,
        loss_prob: float = 0.05,
        half_duplex: bool = True,
        label: str = "m64",
        mode: str = "pull",
        seed: int = 42,
    ) -> None:
        self.bitrate_bps = bitrate_bps
        self.frame_payload_bytes = frame_payload_bytes
        self.latency_s = latency_s
        self.loss_prob = loss_prob
        self.half_duplex = half_duplex
        self.label = label
        self.mode = mode
        self.rng = random.Random(seed)

        self.airtime_s = (frame_payload_bytes * 8) / float(bitrate_bps)

        # State
        self.queue: deque[tuple[bytes, int, float]] = deque()
        self.in_flight: list[_InFlightFrame] = []
        self.tx_busy_until: float | None = None
        self.last_tx_ready_time_s: float | None = None
        self.start_time_s: float | None = None

        # Statistics
        self.payload_bits_sent: int = 0
        self.frames_sent: int = 0
        self.frames_lost: int = 0
        self.frames_rejected: int = 0
        self.total_airtime_s: float = 0.0

    @property
    def queue_len(self) -> int:
        return len(self.queue)

    def is_busy(self, now_s: float) -> bool:
        """Return True if the transmitter is currently busy transmitting airtime."""
        return self.tx_busy_until is not None and now_s < self.tx_busy_until

    def _start_tx(self, payload: bytes, seq: int, start_time_s: float) -> None:
        self.tx_busy_until = start_time_s + self.airtime_s
        self.frames_sent += 1
        self.payload_bits_sent += self.frame_payload_bytes * 8
        self.total_airtime_s += self.airtime_s
        self.last_tx_ready_time_s = None

        arrival_time_s = start_time_s + self.airtime_s + self.latency_s
        is_lost = self.rng.random() < self.loss_prob
        self.in_flight.append(
            _InFlightFrame(
                seq=seq,
                payload=payload,
                arrival_time_s=arrival_time_s,
                is_lost=is_lost,
            )
        )

    def send_frame(self, payload: bytes, seq: int, now_s: float) -> bool:
        """Transmit or queue a frame."""
        if self.start_time_s is None:
            self.start_time_s = now_s

        if len(payload) > self.frame_payload_bytes:
            logger.error(
                "Payload size %d bytes exceeds max frame payload %d bytes; rejected.",
                len(payload),
                self.frame_payload_bytes,
            )
            self.frames_rejected += 1
            return False

        if self.is_busy(now_s):
            self.queue.append((payload, seq, now_s))
            return True

        self._start_tx(payload, seq, now_s)
        return True

    def step(self, now_s: float) -> tuple[list[tuple[int, bytes, float]], bool]:
        """Advance time to now_s.

        Returns:
            (rx_frames, tx_ready)
            where rx_frames is a list of (seq, payload, arrival_time_s),
            and tx_ready is a boolean indicating if tx_ready should be published.
        """
        if self.start_time_s is None:
            self.start_time_s = now_s

        # 1. Advance queue / transmissions
        while (self.tx_busy_until is not None and now_s >= self.tx_busy_until) or (
            self.tx_busy_until is None and len(self.queue) > 0
        ):
            busy_end = self.tx_busy_until if self.tx_busy_until is not None else now_s
            self.tx_busy_until = None
            if len(self.queue) > 0:
                next_payload, next_seq, _ = self.queue.popleft()
                self._start_tx(next_payload, next_seq, busy_end)
            else:
                break

        # 2. Deliver in-flight frames
        rx_frames: list[tuple[int, bytes, float]] = []
        remaining_in_flight: list[_InFlightFrame] = []
        for item in self.in_flight:
            if item.arrival_time_s <= now_s:
                if item.is_lost:
                    self.frames_lost += 1
                elif len(item.payload) > 0 and all(b == 0 for b in item.payload):
                    # Discard all-zero payloads on receiver side (reserved for sync)
                    pass
                else:
                    rx_frames.append((item.seq, item.payload, item.arrival_time_s))
            else:
                remaining_in_flight.append(item)
        self.in_flight = remaining_in_flight

        # 3. Pull mode tx_ready check
        tx_ready = False
        if self.mode == "pull":
            is_idle = (
                self.tx_busy_until is None or now_s >= self.tx_busy_until
            ) and len(self.queue) == 0
            if is_idle and (
                self.last_tx_ready_time_s is None
                or (now_s - self.last_tx_ready_time_s) >= 0.5 - 1e-6
            ):
                tx_ready = True
                self.last_tx_ready_time_s = now_s

        return rx_frames, tx_ready

    def get_stats(self, now_s: float) -> LinkStatsData:
        """Get current link statistics."""
        elapsed_s = (
            (now_s - self.start_time_s) if self.start_time_s is not None else 0.0
        )
        utilisation = 0.0
        if elapsed_s > 0.0:
            utilisation = min(1.0, max(0.0, self.total_airtime_s / elapsed_s))

        return LinkStatsData(
            profile_label=self.label,
            payload_bits_sent=self.payload_bits_sent,
            frames_sent=self.frames_sent,
            frames_lost=self.frames_lost,
            frames_rejected=self.frames_rejected,
            queue_len=len(self.queue),
            utilisation=utilisation,
        )

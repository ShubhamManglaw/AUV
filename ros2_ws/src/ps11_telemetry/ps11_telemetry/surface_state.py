"""Pure-Python state management for the surface receiver and decoder (§11.8).

No ROS imports. Manages contact list (latest report per contact ID),
unwraps modulo-2048 timestamps, and maintains vehicle track history.
"""

from dataclasses import dataclass

from ps11_telemetry import codec


@dataclass
class SurfaceContact:
    """Surface representation of a reported contact."""

    contact_id: int
    class_id: int
    confidence: float
    x_m: float
    y_m: float
    depth_m: float
    sigma_xy_m: float
    first_seen_s: float
    last_seen_s: float
    sightings: int


@dataclass(frozen=True)
class VehiclePoseRecord:
    """Historical record of vehicle pose from a decoded heartbeat."""

    t_s: float
    x_m: float
    y_m: float
    depth_m: float
    heading_deg: float
    battery_frac: float
    state: int
    pending: int


class SurfaceStateTracker:
    """Tracks surface contacts and vehicle path without ROS dependencies."""

    def __init__(self, mission_start_s: float = 0.0) -> None:
        self.mission_start_s = float(mission_start_s)
        self._contacts: dict[int, SurfaceContact] = {}
        self._track: list[VehiclePoseRecord] = []
        self._latest_heartbeat: VehiclePoseRecord | None = None

    @property
    def contacts(self) -> list[SurfaceContact]:
        """Return all tracked contacts sorted by contact ID."""
        return [self._contacts[cid] for cid in sorted(self._contacts.keys())]

    @property
    def track(self) -> list[VehiclePoseRecord]:
        """Return complete vehicle track history."""
        return list(self._track)

    @property
    def latest_heartbeat(self) -> VehiclePoseRecord | None:
        """Return the most recent heartbeat, if any."""
        return self._latest_heartbeat

    def get_contact(self, contact_id: int) -> SurfaceContact | None:
        """Return contact by ID, or None if unknown."""
        return self._contacts.get(contact_id)

    def get_contact_age(self, contact_id: int, now_s: float) -> float:
        """Return elapsed seconds since the last report for this contact."""
        c = self._contacts.get(contact_id)
        if c is None:
            return 0.0
        return max(0.0, now_s - c.last_seen_s)

    def handle_payload(self, payload: bytes, now_s: float) -> tuple[bool, str]:
        """Decode an 8-byte link frame payload and update state.

        Returns (updated, msg_type) where msg_type is 'heartbeat', 'contact',
        'padding', 'extended', or 'invalid'.
        """
        if len(payload) == 0 or len(payload) % 8 != 0:
            return False, "invalid"

        items = codec.unpack_frame(payload)
        if not items:
            type_code = (payload[0] >> 6) & 0x03
            if type_code == codec.TYPE_EXTENDED:
                return False, "extended"
            return False, "padding"

        now_mission_s = max(0.0, now_s - self.mission_start_s)
        any_updated = False
        last_type = "unhandled"

        for item in items:
            if isinstance(item, codec.Heartbeat):
                unwrapped_mission_t = codec.unwrap_time(item.t_s, now_mission_s)
                t_abs_s = self.mission_start_s + unwrapped_mission_t
                pose = VehiclePoseRecord(
                    t_s=t_abs_s,
                    x_m=item.x_m,
                    y_m=item.y_m,
                    depth_m=item.depth_m,
                    heading_deg=item.heading_deg,
                    battery_frac=item.battery_frac,
                    state=item.state,
                    pending=item.pending,
                )
                self._track.append(pose)
                self._latest_heartbeat = pose
                any_updated = True
                last_type = "heartbeat"

            elif isinstance(item, codec.ContactReport):
                unwrapped_mission_t = codec.unwrap_time(item.t_s, now_mission_s)
                t_abs_s = self.mission_start_s + unwrapped_mission_t

                if item.contact_id in self._contacts:
                    existing = self._contacts[item.contact_id]
                    existing.class_id = item.class_id
                    existing.confidence = item.confidence
                    existing.x_m = item.x_m
                    existing.y_m = item.y_m
                    existing.depth_m = item.depth_m
                    existing.sigma_xy_m = item.sigma_m
                    existing.last_seen_s = t_abs_s
                    existing.sightings += 1
                else:
                    self._contacts[item.contact_id] = SurfaceContact(
                        contact_id=item.contact_id,
                        class_id=item.class_id,
                        confidence=item.confidence,
                        x_m=item.x_m,
                        y_m=item.y_m,
                        depth_m=item.depth_m,
                        sigma_xy_m=item.sigma_m,
                        first_seen_s=t_abs_s,
                        last_seen_s=t_abs_s,
                        sightings=1,
                    )
                any_updated = True
                last_type = "contact"

        return any_updated, last_type

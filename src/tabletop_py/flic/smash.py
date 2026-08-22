"""Timestamp matching for the Teensy/Flic hardware smash test."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SmashLatencySample:
    """One paired Teensy and Flic button event."""

    teensy_ns: int
    flic_ns: int

    @property
    def delta_ns(self) -> int:
        """Return Flic timestamp minus Teensy timestamp in nanoseconds."""
        return self.flic_ns - self.teensy_ns

    @property
    def delta_ms(self) -> float:
        """Return Flic timestamp minus Teensy timestamp in milliseconds."""
        return self.delta_ns / 1_000_000.0


class SmashLatencyMatcher:
    """Pair asynchronous Flic and Teensy events by timestamp proximity.

    The first Teensy sensor snapshot establishes the pre-test baseline because
    ``button_last_time_pressed`` is repeated at 100 Hz until the next physical
    press. Only changes after that baseline become candidate smash events.
    """

    def __init__(self, target_addr: str, pairing_window_ms: float = 250.0):
        if pairing_window_ms <= 0:
            raise ValueError("pairing_window_ms must be positive")

        self.target_addr = target_addr.lower()
        self.pairing_window_ns = round(pairing_window_ms * 1_000_000)
        self.last_teensy_ns: int | None = None
        self.pending_teensy_ns: list[int] = []
        self.pending_flic_ns: list[int] = []
        self.latest_event_ns = 0
        self.expired_teensy_events = 0
        self.expired_flic_events = 0

    @property
    def baseline_ready(self) -> bool:
        """Return whether the initial repeated Teensy timestamp was observed."""
        return self.last_teensy_ns is not None

    def observe_teensy(self, timestamp_ns: int) -> list[SmashLatencySample]:
        """Observe the Teensy's interrupt-latched button-onset timestamp."""
        if timestamp_ns <= 0:
            return []
        if self.last_teensy_ns is None:
            self.last_teensy_ns = timestamp_ns
            return []
        if timestamp_ns == self.last_teensy_ns:
            return []

        self.last_teensy_ns = timestamp_ns
        self.pending_teensy_ns.append(timestamp_ns)
        return self._match_and_expire(timestamp_ns)

    def observe_flic(
        self, addr: str, timestamp_ns: int
    ) -> list[SmashLatencySample]:
        """Observe a Flic event, ignoring non-target or invalid events."""
        if addr.lower() != self.target_addr or timestamp_ns <= 0:
            return []

        self.pending_flic_ns.append(timestamp_ns)
        return self._match_and_expire(timestamp_ns)

    def _match_and_expire(self, latest_ns: int) -> list[SmashLatencySample]:
        self.latest_event_ns = max(self.latest_event_ns, latest_ns)
        samples: list[SmashLatencySample] = []

        while self.pending_teensy_ns and self.pending_flic_ns:
            best = min(
                (
                    (abs(flic_ns - teensy_ns), teensy_index, flic_index)
                    for teensy_index, teensy_ns in enumerate(
                        self.pending_teensy_ns
                    )
                    for flic_index, flic_ns in enumerate(self.pending_flic_ns)
                ),
                key=lambda candidate: candidate[0],
            )
            difference_ns, teensy_index, flic_index = best
            if difference_ns > self.pairing_window_ns:
                break

            teensy_ns = self.pending_teensy_ns.pop(teensy_index)
            flic_ns = self.pending_flic_ns.pop(flic_index)
            samples.append(SmashLatencySample(teensy_ns, flic_ns))

        cutoff_ns = self.latest_event_ns - self.pairing_window_ns
        old_teensy_count = len(self.pending_teensy_ns)
        old_flic_count = len(self.pending_flic_ns)
        self.pending_teensy_ns = [
            timestamp
            for timestamp in self.pending_teensy_ns
            if timestamp >= cutoff_ns
        ]
        self.pending_flic_ns = [
            timestamp
            for timestamp in self.pending_flic_ns
            if timestamp >= cutoff_ns
        ]
        self.expired_teensy_events += old_teensy_count - len(
            self.pending_teensy_ns
        )
        self.expired_flic_events += old_flic_count - len(self.pending_flic_ns)

        return samples

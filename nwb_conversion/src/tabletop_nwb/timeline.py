"""Shared session timeline: every stream's samples in seconds since session start.

``session_start_ns`` (host ROS time) is the earliest sample of any written
stream, floored to the microsecond so that the NWB ``session_start_time``
datetime represents it exactly. All four NWB files use the same value.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone

import numpy as np

from .extract import ARMS, CAMERAS, SERVICES, Extracted, service_calls
from .sync import SyncResult


@dataclass
class StreamTime:
    t_origin_s: np.ndarray  # seconds since sync origin (bag start)
    method: str  # host_clock | host_receive | pulse_fit
    source: str  # which clock column was used


@dataclass
class Timeline:
    origin_ns: int
    start_ns: int = 0
    streams: dict[str, StreamTime] = field(default_factory=dict)
    reordered: dict[str, int] = field(
        default_factory=dict
    )  # stream → samples moved to sort by time

    def add(
        self, name: str, t_origin_s: np.ndarray, method: str, source: str
    ) -> None:
        self.streams[name] = StreamTime(
            np.asarray(t_origin_s, dtype=np.float64), method, source
        )

    def host_ns_to_origin_s(self, ns) -> np.ndarray:
        return (np.asarray(ns, dtype=np.int64) - self.origin_ns).astype(
            np.float64
        ) / 1e9

    def rel(self, name: str) -> np.ndarray:
        return (
            self.streams[name].t_origin_s
            - (self.start_ns - self.origin_ns) / 1e9
        )

    def rel_host_ns(self, ns) -> np.ndarray:
        """Host ROS time (int ns) → seconds since session start."""
        return (np.asarray(ns, dtype=np.int64) - self.start_ns).astype(
            np.float64
        ) / 1e9

    @property
    def start_datetime(self) -> datetime:
        sec, ns = divmod(self.start_ns, 1_000_000_000)
        return datetime.fromtimestamp(sec, tz=timezone.utc).replace(
            microsecond=ns // 1000
        )

    def finalize(self) -> None:
        firsts = [
            np.nanmin(s.t_origin_s)
            for s in self.streams.values()
            if len(s.t_origin_s)
        ]
        earliest = min(firsts) if firsts else 0.0
        start = self.origin_ns + int(np.floor(earliest * 1e9))
        self.start_ns = start - (start % 1000)


def build_timeline(ex: Extracted, sync: SyncResult) -> Timeline:
    tl = Timeline(origin_ns=sync.origin_ns)
    h = tl.host_ns_to_origin_s

    def host(
        name: str, ns, source: str = "header stamp", method: str = "host_clock"
    ):
        if ns is not None and len(ns):
            tl.add(name, h(ns), method, source)

    for key in ["teensy_sensor", "task_event", "flic", "rosout", "markers"]:
        df = ex.tables.get(key)
        if df is not None and len(df):
            host(key, df["header_ns"].to_numpy())
    if ex.has("teensy_log"):
        host(
            "teensy_log",
            ex.tables["teensy_log"]["log_ns"].to_numpy(),
            "bag receive time",
            "host_receive",
        )
    if ex.has("joint_states"):
        host("joint_states", ex.arrays["joint_states"]["header_ns"])
    for a in ARMS:
        for key in (f"wrench_{a}", f"tcp_pose_{a}"):
            if ex.has(key):
                host(key, ex.tables[key]["header_ns"].to_numpy())
        if ex.has(f"io_{a}"):
            f = sync.fits.get(f"ur_{a}")
            log_s = h(ex.arrays[f"io_{a}"]["log_ns"])
            if f is not None and f.status == "ok":
                tl.add(
                    f"io_{a}",
                    f.to_rel_s(log_s, sync.origin_ns),
                    "pulse_fit",
                    "bag receive time → Teensy",
                )
            else:
                tl.add(f"io_{a}", log_s, "host_receive", "bag receive time")
    for s in SERVICES:
        calls = service_calls(ex.tables.get(f"srv_{s}"))
        if len(calls):
            host(
                f"srv_{s}",
                calls["time_ns"].to_numpy().astype(np.int64),
                "service introspection stamp",
            )
    if ex.has("eyelink"):
        e = ex.arrays["eyelink"]
        f = sync.fits.get("eyelink")
        if f is not None and f.status == "ok":
            tl.add(
                "eyelink",
                f.to_rel_s(e["eyelink_time_ms"] / 1e3, sync.origin_ns),
                "pulse_fit",
                "EyeLink tracker clock → Teensy",
            )
        else:
            host(
                "eyelink",
                e["header_ns"],
                "header stamp (host clock; no usable sync pulses)",
            )
    cam_fit = sync.fits.get("cameras")
    for c in CAMERAS:
        df = ex.tables.get(f"cam_meta_{c}")
        if df is None or not len(df):
            continue
        x = h(df["header_ns"].to_numpy())
        if cam_fit is not None and cam_fit.status == "ok":
            tl.add(
                f"cam_{c}",
                cam_fit.to_rel_s(x, sync.origin_ns),
                "pulse_fit",
                "FLIR header stamp → Teensy",
            )
        else:
            tl.add(f"cam_{c}", x, "host_clock", "FLIR header stamp")
    normalize_order(ex, tl)
    tl.finalize()
    return tl


def normalize_order(ex: Extracted, tl: Timeline) -> None:
    """Sort each stream by its own timestamps (NWB requires ascending times).

    Recorded order is bag-receive order; a few messages arrive with header
    stamps out of order (seen in real bags: one sample stamped ~1 ms ahead of
    the next, and /rosout mixing many nodes' clocks). Samples are reordered
    with a stable sort, data rows move with their timestamps, and the number
    of samples moved is recorded in ``tl.reordered`` for the provenance table
    and the validator (check D4).
    """
    for name, st in tl.streams.items():
        t = st.t_origin_s
        if len(t) < 2 or not np.any(np.diff(t) < 0):
            continue
        order = np.argsort(t, kind="stable")
        tl.reordered[name] = int(np.sum(order != np.arange(len(order))))
        st.t_origin_s = t[order]
        key = f"cam_meta_{name[4:]}" if name.startswith("cam_") else name
        if key in ex.tables and len(ex.tables[key]) == len(order):
            ex.tables[key] = ex.tables[key].iloc[order].reset_index(drop=True)
        elif key in ex.arrays and ex.arrays[key]:
            ex.arrays[key] = {k: v[order] for k, v in ex.arrays[key].items()}
        # srv_* timelines come from service_calls(), which the writer recomputes already sorted

"""Stage [2]: align every pulse-carrying stream to the Teensy pulse train.

Reference: Teensy pin-31 rising edges, timestamped in host ROS time by the
Teensy (``TeensySensor.sync_pulse_last_time_on``). Each other stream's own
pulse onsets (in that stream's clock) are matched to the reference by their
inter-pulse intervals (random 850–1150 ms, so a run of k intervals is a unique
code), then a robust linear map is fitted:

    host_s_since_origin = a * (stream_s - x0) + b_s

where ``origin_ns`` is an integer host-time origin (the bag start). Keeping
host times relative to an integer origin avoids float64 losing precision on
absolute epoch nanoseconds (~250 ns at 1.8e18).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from .extract import Extracted

EYELINK_SYNC_MASK = (
    0x20  # DB25 pin 12 → input bit 5; active-low (127 idle, 95 asserted)
)
FLIR_SYNC_BIT = 3  # Line3 in line_status
FLIR_SYNC_CAMERA = "left_back_top_cam"
UR_SYNC_DI = 3  # active-low

PULSE_WIDTH_RANGE_S = (0.05, 0.25)
IPI_RANGE_S = (0.7, 1.4)
MAX_RATE_ERROR = 0.01  # |a - 1|: real clocks drift by ≤ ~100s of ppm; anything near 1 % is a mismatch


@dataclass
class Pulses:
    onsets: np.ndarray  # seconds, stream clock
    widths: np.ndarray  # seconds
    n_raw_rises: int = 0


@dataclass
class SyncFit:
    stream: str
    method: str  # pulse_fit | host_clock | host_receive | reference
    status: str  # ok | reference | no_pulses | no_reference | fit_failed | not_attempted
    clock: str = ""
    a: float = 1.0  # host seconds per stream second
    b_s: float = 0.0
    x0_s: float = (
        0.0  # fit is centered: host_s_since_origin = a*(x - x0) + b_s
    )
    origin_ns: int = 0
    n_stream_pulses: int = 0
    n_raw_rises: int = 0
    n_reference_pulses: int = 0
    n_reference_in_span: int = 0
    n_matched: int = 0
    matched_fraction: float = float("nan")
    residual_rms_ms: float = float("nan")
    residual_max_ms: float = float("nan")
    drift_ppm: float = float("nan")
    coverage_s: float = 0.0
    note: str = ""
    matched_stream_s: list = field(default_factory=list, repr=False)
    matched_ref_s: list = field(default_factory=list, repr=False)

    def to_rel_s(self, x_s: np.ndarray, start_ns: int) -> np.ndarray:
        """Stream clock → seconds since ``start_ns`` (host ROS time)."""
        return (
            self.a * (np.asarray(x_s, dtype=np.float64) - self.x0_s)
            + self.b_s
            + (self.origin_ns - start_ns) / 1e9
        )

    def summary(self) -> dict:
        d = asdict(self)
        d.pop("matched_stream_s")
        d.pop("matched_ref_s")
        return d


def pulses_from_levels(
    t_s: np.ndarray, asserted: np.ndarray, width_range=PULSE_WIDTH_RANGE_S
) -> Pulses:
    """Onsets of asserted runs whose duration is a plausible sync pulse.

    An edge happens somewhere between the last sample before it and the first
    sample after it; using the first asserted sample would bias onsets late by
    half a sample interval (4.2 ms at 120 fps), so each edge is estimated as
    the midpoint of that bracket.
    """
    t_s = np.asarray(t_s, dtype=np.float64)
    a = np.asarray(asserted, dtype=bool)
    if len(a) < 2:
        return Pulses(np.array([]), np.array([]))
    d = np.diff(a.astype(np.int8))
    rises = np.flatnonzero(d == 1) + 1
    falls = np.flatnonzero(d == -1) + 1
    onsets, widths = [], []
    fi = 0
    for r in rises:
        while fi < len(falls) and falls[fi] <= r:
            fi += 1
        if fi >= len(falls):
            break
        on = 0.5 * (t_s[r - 1] + t_s[r])
        off = 0.5 * (t_s[falls[fi] - 1] + t_s[falls[fi]])
        w = off - on
        if width_range[0] <= w <= width_range[1]:
            onsets.append(on)
            widths.append(w)
    return Pulses(np.array(onsets), np.array(widths), n_raw_rises=len(rises))


def teensy_reference(
    ex: Extracted, bag_start_ns: int, bag_end_ns: int
) -> Pulses:
    """Teensy rising edges recorded during this bag, in seconds since ``bag_start_ns``.

    ``sync_pulse_last_time_on`` persists across sessions, so values before the
    bag start are stale and dropped.
    """
    ts = ex.tables.get("teensy_sensor")
    if ts is None or len(ts) == 0:
        return Pulses(np.array([]), np.array([]))
    on = np.unique(ts["sync_on_ns"].to_numpy())
    on = on[(on >= bag_start_ns) & (on <= bag_end_ns)]
    off = np.unique(ts["sync_off_ns"].to_numpy())
    off = off[(off >= bag_start_ns) & (off <= bag_end_ns)]
    widths = np.full(len(on), np.nan)
    j = np.searchsorted(off, on)
    ok = j < len(off)
    widths[ok] = (off[j[ok]] - on[ok]) / 1e9
    return Pulses(
        (on - bag_start_ns).astype(np.float64) / 1e9,
        widths,
        n_raw_rises=len(on),
    )


def _robust_line(
    x: np.ndarray, y: np.ndarray, x0: float, thresh: float
) -> tuple[float, float, np.ndarray]:
    """Fit y = a*(x-x0) + b with iterative outlier rejection; returns a, b, inlier mask."""
    xc = x - x0
    a, b = 1.0, float(np.median(y - xc))
    keep = np.ones(len(x), dtype=bool)
    for _ in range(10):
        r = y - (a * xc + b)
        new_keep = np.abs(r - np.median(r[keep])) < max(
            thresh,
            5 * 1.4826 * np.median(np.abs(r[keep] - np.median(r[keep]))),
        )
        if new_keep.sum() >= 2:
            if new_keep.sum() >= 3 and np.ptp(xc[new_keep]) > 0:
                a, b = np.polyfit(xc[new_keep], y[new_keep], 1)
            else:
                b = float(np.median((y - a * xc)[new_keep]))
        if np.array_equal(new_keep, keep):
            break
        keep = new_keep
    return float(a), float(b), keep


def match_and_fit(
    stream: str,
    clock: str,
    ref: Pulses,
    sp: Pulses,
    *,
    k: int = 5,
    interval_tol_s: float = 0.025,
    match_tol_s: float = 0.05,
    min_matched: int = 5,
) -> SyncFit:
    fit = SyncFit(
        stream=stream,
        method="pulse_fit",
        status="fit_failed",
        clock=clock,
        n_stream_pulses=len(sp.onsets),
        n_raw_rises=sp.n_raw_rises,
        n_reference_pulses=len(ref.onsets),
    )
    if len(sp.onsets) == 0:
        fit.status = "no_pulses"
        fit.note = (
            f"{sp.n_raw_rises} raw rising edges, none with a sync-pulse width"
        )
        return fit
    if len(ref.onsets) == 0:
        fit.status = "no_reference"
        return fit
    r, s = ref.onsets, sp.onsets
    dr, ds = np.diff(r), np.diff(s)
    kk = min(k, len(ds), len(dr))
    if kk < 2:
        fit.note = "too few pulses to match"
        return fit
    R = sliding_window_view(dr, kk)
    S = sliding_window_view(ds, kk)
    # Only windows made of plausible intervals can identify a position.
    good_s = np.flatnonzero(
        np.all((S > IPI_RANGE_S[0]) & (S < IPI_RANGE_S[1]), axis=1)
    )
    anchors = (
        good_s[
            np.linspace(0, len(good_s) - 1, min(300, len(good_s))).astype(int)
        ]
        if len(good_s)
        else []
    )
    pairs = []
    for i in anchors:
        j = np.flatnonzero(np.abs(R - S[i]).max(axis=1) < interval_tol_s)
        if len(j) == 1:
            pairs.append((i, j[0]))
    if len(pairs) < 2:
        fit.note = f"interval signatures matched at {len(pairs)} anchors"
        return fit
    x = s[[p[0] for p in pairs]]
    y = r[[p[1] for p in pairs]]
    x0 = float(np.median(s))
    a, b, keep = _robust_line(x, y, x0, thresh=match_tol_s)
    if keep.sum() < 3 or abs(a - 1) > MAX_RATE_ERROR:
        fit.note = f"anchor pairs inconsistent ({int(keep.sum())} agree, rate {a:.6f})"
        return fit
    # Assign every stream pulse to the nearest predicted reference pulse, refit.
    for _ in range(3):
        pred = a * (s - x0) + b
        j = np.clip(np.searchsorted(r, pred), 1, len(r) - 1)
        j = np.where(np.abs(r[j - 1] - pred) < np.abs(r[j] - pred), j - 1, j)
        ok = np.abs(r[j] - pred) < match_tol_s
        # one-to-one
        _, first = np.unique(j[ok], return_index=True)
        idx_s = np.flatnonzero(ok)[first]
        if len(idx_s) < 2:
            break
        a, b, keep = _robust_line(
            s[idx_s], r[j[idx_s]], x0, thresh=match_tol_s
        )
        idx_s = idx_s[keep]
    ms, mr = s[idx_s], r[j[idx_s]]
    resid = mr - (a * (ms - x0) + b)
    span = (a * (s.min() - x0) + b, a * (s.max() - x0) + b)
    in_span = int(
        np.sum((r >= span[0] - match_tol_s) & (r <= span[1] + match_tol_s))
    )
    fit.a, fit.b_s, fit.x0_s = a, b, x0
    fit.n_matched = len(idx_s)
    fit.n_reference_in_span = in_span
    fit.matched_fraction = len(idx_s) / max(1, min(len(s), in_span))
    fit.residual_rms_ms = float(np.sqrt(np.mean(resid**2)) * 1e3)
    fit.residual_max_ms = float(np.max(np.abs(resid)) * 1e3)
    fit.drift_ppm = (a - 1.0) * 1e6
    fit.coverage_s = float(ms.max() - ms.min()) if len(ms) else 0.0
    fit.matched_stream_s = ms.tolist()
    fit.matched_ref_s = mr.tolist()
    problems = []
    if len(idx_s) < min_matched:
        problems.append(f"only {len(idx_s)} matched pulses")
    if abs(a - 1) > MAX_RATE_ERROR:
        problems.append(f"implausible clock rate {a:.6f}")
    if fit.matched_fraction < 0.5:
        problems.append(f"matched fraction {fit.matched_fraction:.0%}")
    fit.status = "fit_failed" if problems else "ok"
    fit.note = "; ".join(problems)
    return fit


@dataclass
class SyncResult:
    reference: Pulses  # onsets in seconds since origin_ns
    fits: dict[str, SyncFit]
    stream_pulses: dict[str, Pulses]
    origin_ns: int

    def report(self) -> dict:
        on = self.reference.onsets
        return {
            "origin_ns": self.origin_ns,
            "reference": {
                "stream": "teensy",
                "n_pulses": int(len(on)),
                "first_ns": self.origin_ns + int(round(on[0] * 1e9))
                if len(on)
                else None,
                "last_ns": self.origin_ns + int(round(on[-1] * 1e9))
                if len(on)
                else None,
            },
            "fits": {k: v.summary() for k, v in self.fits.items()},
        }


def eyelink_pulses(ex: Extracted) -> Pulses:
    e = ex.arrays.get("eyelink") or {}
    if not e or len(e.get("input", [])) == 0:
        return Pulses(np.array([]), np.array([]))
    t = e["eyelink_time_ms"].astype(np.float64) / 1e3
    asserted = (e["input"].astype(np.int64) & EYELINK_SYNC_MASK) == 0
    return pulses_from_levels(t, asserted)


def camera_pulses(ex: Extracted, origin_ns: int) -> Pulses:
    df = ex.tables.get(f"cam_meta_{FLIR_SYNC_CAMERA}")
    if df is None or len(df) == 0:
        return Pulses(np.array([]), np.array([]))
    t = (df["header_ns"].to_numpy() - origin_ns).astype(np.float64) / 1e9
    asserted = (
        df["line_status"].to_numpy().astype(np.int64) >> FLIR_SYNC_BIT
    ) & 1
    return pulses_from_levels(t, asserted.astype(bool))


def ur_pulses(ex: Extracted, arm: str, origin_ns: int) -> Pulses:
    io = ex.arrays.get(f"io_{arm}") or {}
    if not io:
        return Pulses(np.array([]), np.array([]))
    t = (io["log_ns"] - origin_ns).astype(np.float64) / 1e9
    return pulses_from_levels(t, ~io["digital_in"][:, UR_SYNC_DI])


def run_sync(ex: Extracted, bag_start_ns: int, bag_end_ns: int) -> SyncResult:
    origin = int(bag_start_ns)
    ref = teensy_reference(ex, origin, bag_end_ns)
    fits: dict[str, SyncFit] = {}
    pulses: dict[str, Pulses] = {}
    fits["teensy"] = SyncFit(
        "teensy",
        "reference",
        "reference" if len(ref.onsets) else "no_pulses",
        clock="host ROS time (Teensy micro-ROS synchronized)",
        n_stream_pulses=len(ref.onsets),
        n_reference_pulses=len(ref.onsets),
        origin_ns=origin,
    )
    candidates = {
        "eyelink": (
            "EyeLink tracker clock (eyelink_time_ms / 1000)",
            eyelink_pulses,
        ),
        "cameras": (
            f"FLIR header stamp, s since origin ({FLIR_SYNC_CAMERA} Line3)",
            lambda e: camera_pulses(e, origin),
        ),
        "ur_left": (
            "bag receive time, s since origin (IOStates has no header)",
            lambda e: ur_pulses(e, "left", origin),
        ),
        "ur_right": (
            "bag receive time, s since origin (IOStates has no header)",
            lambda e: ur_pulses(e, "right", origin),
        ),
    }
    present = {
        "eyelink": ex.has("eyelink"),
        "cameras": ex.has(f"cam_meta_{FLIR_SYNC_CAMERA}"),
        "ur_left": ex.has("io_left"),
        "ur_right": ex.has("io_right"),
    }
    for name, (clock, fn) in candidates.items():
        if not present[name]:
            continue
        p = fn(ex)
        pulses[name] = p
        fits[name] = match_and_fit(
            name,
            clock,
            ref,
            p,
            match_tol_s=0.05 if name != "cameras" else 0.06,
        )
        fits[name].origin_ns = origin
    return SyncResult(ref, fits, pulses, origin)

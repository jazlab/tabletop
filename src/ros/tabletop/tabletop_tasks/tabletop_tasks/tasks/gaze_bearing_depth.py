"""Repeated viewing directions with independently varying LED depth."""

from collections.abc import Sequence

import numpy as np

DEFAULT_VIEWPOINT_OFFSET_M = (-0.006508, -0.483680, 0.334647)
DESIGN_DURATION_S = 150.0


def gaze_bearing_depth_offsets(
    *,
    num_segments: int = 1100,
    viewpoint_offset_from_center_led_m: Sequence[float] = (
        DEFAULT_VIEWPOINT_OFFSET_M
    ),
) -> np.ndarray:
    """Sample a closed pursuit curve at roughly equal arc length.

    The viewpoint is the midpoint of the eyes relative to the LED at the
    center pose, in world axes. With fixed tool orientation the LED and TCP
    translate by the same offsets, so the LED-to-TCP offset is not required.
    The 2-D pattern repeats six times; the 11 depth cycles give each pass a
    different range phase. The planner assigns execution timestamps.
    """
    eye = np.asarray(viewpoint_offset_from_center_led_m, dtype=float)
    if eye.shape != (3,) or not np.isfinite(eye).all():
        raise ValueError(
            "Viewpoint offset must contain three finite world coordinates"
        )
    if eye[1] >= -0.094:
        raise ValueError(
            "The viewpoint must be in front of the entire path (world -Y)"
        )
    if not isinstance(num_segments, int) or num_segments < 750:
        raise ValueError(
            "The repeated-bearing path needs at least 750 segments"
        )

    time = np.linspace(0.0, DESIGN_DURATION_S, 10 * num_segments + 1)
    path = _bearing_depth_path(time, eye)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    sample = np.linspace(0.0, arc[-1], num_segments + 1)
    return np.column_stack([np.interp(sample, arc, axis) for axis in path.T])


def _bearing_depth_path(time: np.ndarray, eye: np.ndarray) -> np.ndarray:
    """Evaluate the Cartesian curve along the nominal design clock."""
    ramp = 2.0
    entry = np.clip(time / ramp, 0.0, 1.0)
    exit_ = np.clip((DESIGN_DURATION_S - time) / ramp, 0.0, 1.0)

    def eased_clock(u: np.ndarray) -> np.ndarray:
        return ramp * u**3 * (6 - 8 * u + 3 * u**2)

    clock = np.where(time < ramp, eased_clock(entry), time)
    clock = np.where(
        time > DESIGN_DURATION_S - ramp,
        DESIGN_DURATION_S - eased_clock(exit_),
        clock,
    )
    phase = 2 * np.pi * clock / 25.0
    plane = np.column_stack(
        (
            0.070 * np.sin(3 * phase),
            np.zeros_like(phase),
            0.092 * np.sin(2 * phase),
        )
    )
    ray = (plane - eye) / -eye[1]
    # Find the allowable Y interval along each sightline within the
    # translation box. Smooth the intersections to avoid hard corners.
    bound = 0.094
    lo = np.full_like(time, -bound)
    hi = np.full_like(time, bound)

    def soft_min(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        # Blend the competing bounds over 12 mm instead of switching over
        # 4 mm: the tighter blend created brief acceleration spikes. The
        # soft minimum remains conservative, below both hard bounds.
        smoothing = 0.012
        return -smoothing * np.logaddexp(-a / smoothing, -b / smoothing)

    for axis in (0, 2):
        slope = ray[:, axis]
        active = np.abs(slope) >= 1e-9
        den = np.where(active, slope, 1.0)
        a = (-bound - plane[:, axis]) / den
        b = (bound - plane[:, axis]) / den
        lo = -soft_min(-lo, np.where(active, -np.minimum(a, b), np.inf))
        hi = soft_min(hi, np.where(active, np.maximum(a, b), np.inf))
    if np.any(lo >= hi):
        raise ValueError("The viewpoint would collapse the allowed depth span")

    def blend(u: np.ndarray) -> np.ndarray:
        u = np.clip(u, 0.0, 1.0)
        return u**3 * (10 - 15 * u + 6 * u**2)

    depth = (lo + hi) / 2 + 0.94 * (hi - lo) / 2 * np.sin(11 * phase / 6)
    depth *= blend(clock / ramp) * blend((DESIGN_DURATION_S - clock) / ramp)
    path = plane + depth[:, None] * ray
    path[(time == 0.0) | (time == DESIGN_DURATION_S)] = 0.0
    if not np.isfinite(path).all() or np.max(np.abs(path)) > bound + 1e-9:
        raise ValueError(
            "Repeated-bearing path exceeds the ±9.4 cm pursuit envelope"
        )

    return path

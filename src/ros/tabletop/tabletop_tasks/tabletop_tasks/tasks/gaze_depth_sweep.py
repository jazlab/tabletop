"""Geometry of the depth-balanced gaze-calibration sweep."""

import math

import numpy as np


def _blend(s: np.ndarray) -> np.ndarray:
    """Blend with zero first and second derivatives at both ends."""
    return s**3 * (10 + s * (-15 + 6 * s))


def _bezier(u: np.ndarray, control: np.ndarray) -> np.ndarray:
    u = u[:, None]
    v = 1 - u
    return (
        v**3 * control[0]
        + 3 * v**2 * u * control[1]
        + 3 * v * u**2 * control[2]
        + u**3 * control[3]
    )


def gaze_depth_sweep_offsets(
    *,
    num_segments: int = 1000,
    nominal_viewing_distance_m: float = 0.50,
    depth_half_span_m: float = 0.075,
    viewing_grid_half_span_m: float = 0.064,
    angular_dither_at_center_m: float = 0.012,
) -> np.ndarray:
    """Return a continuous near–far sweep about a nominal viewing origin.

    World Y is depth, with the viewer in front of the center on the -Y side.
    For a fixed-orientation fixture, the LED and TCP translate by the same
    offsets. The viewer's nominal distance to the central LED sets how far
    the transverse offsets scale during each depth excursion.

    The 150-second design clock specifies relative progress along the curve:
    nine 12-second neighborhoods, eight 3-second bridges, and a 9-second
    entrance and exit sweep. The robot planner retimes the sampled geometry;
    the task separately enforces the overall planned execution duration.
    """
    dimensions = (
        nominal_viewing_distance_m,
        depth_half_span_m,
        viewing_grid_half_span_m,
        angular_dither_at_center_m,
    )
    if not all(math.isfinite(x) and x > 0 for x in dimensions):
        raise ValueError("Depth sweep dimensions must be finite and positive")
    if depth_half_span_m >= nominal_viewing_distance_m:
        raise ValueError(
            "The near end of the sweep must remain in front of the viewer"
        )
    if num_segments < 750:
        raise ValueError("The depth sweep needs at least 750 segments")

    t = np.linspace(0, 150.0, 10 * num_segments + 1)
    anchors = viewing_grid_half_span_m * np.array(
        [
            (-1, -1),
            (0, -1),
            (1, -1),
            (1, 0),
            (0, 0),
            (-1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ]
    )
    projected = np.empty((len(t), 2))
    enter = t < 9.0
    leave = t >= 141.0
    projected[enter] = _bezier(
        _blend(t[enter] / 9.0),
        np.array([(0, 0), (0.078, 0.085), (-0.082, 0.084), anchors[0]]),
    )
    projected[leave] = _bezier(
        _blend((t[leave] - 141.0) / 9.0),
        np.array([anchors[-1], (-0.082, 0.078), (-0.078, -0.083), (0, 0)]),
    )
    for i, anchor in enumerate(anchors):
        start = 9.0 + i * 15.0
        projected[(t >= start) & (t < start + 12.0)] = anchor
        if i + 1 < len(anchors):
            bridge = (t >= start + 12.0) & (t < start + 15.0)
            s = _blend((t[bridge] - start - 12.0) / 3.0)
            projected[bridge] = anchor + s[:, None] * (anchors[i + 1] - anchor)

    # The small ellipse is continuous across depth reversals and bridges.
    # It repeats several times per depth cycle, so a given viewing direction
    # is sampled at multiple depths. Taper only at the overall start and end.
    envelope = _blend(np.clip(np.minimum(t, 150.0 - t) / 2.5, 0.0, 1.0))
    phase = 2 * np.pi * t / 2.0
    projected += (angular_dither_at_center_m * envelope)[
        :, None
    ] * np.column_stack([np.sin(phase), 0.8 * np.cos(phase)])
    depth = depth_half_span_m * envelope * np.sin(2 * np.pi * (t - 9.0) / 12.0)
    projected *= (1 + depth / nominal_viewing_distance_m)[:, None]
    offsets = np.column_stack((projected[:, 0], depth, projected[:, 1]))
    if float(np.max(np.abs(offsets))) > 0.100:
        raise ValueError(
            "Depth sweep exceeds the existing ±10 cm pursuit envelope"
        )
    # Sample by arc length so the gentle start/end tapers don't hand the
    # linear planner a run of sub-millimeter, nearly identical goals.
    arc = np.r_[
        0.0, np.cumsum(np.linalg.norm(np.diff(offsets, axis=0), axis=1))
    ]
    sample = np.linspace(0.0, arc[-1], num_segments + 1)
    return np.column_stack([np.interp(sample, arc, p) for p in offsets.T])

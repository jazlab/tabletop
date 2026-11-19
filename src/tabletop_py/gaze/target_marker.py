"""Identify a calibration target mounted on the right robot (offline only)."""

import logging
import re

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)

TCP_TOPIC = "/right_tcp_pose_broadcaster/pose"
TCP_CSV = "right_tcp_pose_broadcaster_pose.csv"

# Matches dual_rsp.launch.py and the UR base-link -> base rotation.
_WORLD_FROM_RIGHT_BASE = Rotation.from_euler("z", np.pi - 1.5707)
_RIGHT_BASE_IN_WORLD = np.array([1.3462, 0.7227, 0.3085])
# Matches the world -> optitrack static transform in optitrack.launch.py.
_WORLD_FROM_OPTITRACK = Rotation.from_euler("xyz", [np.pi / 2, 0, np.pi / 2])
_OPTITRACK_IN_WORLD = np.array([0.983, 0.623, 0.31])


def _stamp_ns(df: pd.DataFrame) -> np.ndarray:
    return df["header.stamp.sec"].to_numpy(
        dtype=np.int64
    ) * 1_000_000_000 + df["header.stamp.nanosec"].to_numpy(dtype=np.int64)


def _representative_frames(tcp_position: np.ndarray) -> np.ndarray:
    # The robot waits during planning. Do not let thousands of stationary
    # frames outvote the part of the recording where we can identify the target.
    _, cells = np.unique(
        np.floor(tcp_position / 0.02), axis=0, return_inverse=True
    )
    order = np.argsort(cells, kind="stable")
    groups = np.split(order, np.flatnonzero(np.diff(cells[order])) + 1)
    return np.concatenate(
        [
            g[np.linspace(0, len(g) - 1, min(3, len(g)), dtype=int)]
            for g in groups
        ]
    )


def _strongest_offset(points: np.ndarray, tolerance: float) -> np.ndarray:
    tree = cKDTree(points)
    votes = tree.query_ball_point(points, tolerance, return_length=True)
    neighborhood = tree.query_ball_point(points[votes.argmax()], tolerance)
    return np.median(points[neighborhood], axis=0)


def select_target_marker(
    markers: pd.DataFrame,
    tcp: pd.DataFrame,
    *,
    position_tolerance_m: float = 0.015,
    max_tool_offset_m: float = 0.30,
    max_tcp_skew_sec: float = 0.020,
) -> np.ndarray:
    """Return measured OptiTrack XYZ; leave unmatched/ambiguous rows as NaN.

    Learn the target's fixed offset in the tool frame from spatially spread
    samples. A lone point that dominates only while the tool is still is not
    sufficient evidence. Never substitute the robot pose for a measurement.
    """
    if min(position_tolerance_m, max_tool_offset_m, max_tcp_skew_sec) <= 0:
        raise ValueError("Target selection tolerances must be positive")
    for frame, expected in ((markers, "optitrack"), (tcp, "right_base")):
        if set(frame["header.frame_id"].dropna()) != {expected}:
            raise ValueError(
                f"Automatic target selection expects frame {expected!r}"
            )

    slots = sorted(
        {
            int(match[1])
            for col in markers.columns
            if (
                match := re.fullmatch(r"markers\[(\d+)\]\.translation\.x", col)
            )
        }
    )
    if not slots:
        raise ValueError("No OptiTrack marker XYZ columns found")
    xyz = np.stack(
        [
            markers[
                [f"markers[{i}].translation.{axis}" for axis in "xyz"]
            ].to_numpy(float)
            for i in slots
        ],
        axis=1,
    )

    t = _stamp_ns(markers)
    tt = _stamp_ns(tcp)
    p = tcp[[f"pose.position.{ax}" for ax in "xyz"]].to_numpy(float)
    quat = tcp[[f"pose.orientation.{ax}" for ax in "xyzw"]].to_numpy(float)
    good_tcp = (
        np.isfinite(p).all(axis=1)
        & np.isfinite(quat).all(axis=1)
        & (np.linalg.norm(quat, axis=1) > 0)
    )
    tt, p, quat = tt[good_tcp], p[good_tcp], quat[good_tcp]
    if len(tt) < 2 or np.any(np.diff(tt) < 0):
        raise ValueError(
            "Automatic target selection needs time-ordered TCP poses"
        )
    right = np.searchsorted(tt, t).clip(0, len(tt) - 1)
    left = (right - 1).clip(0)
    near = np.where(abs(t - tt[left]) <= abs(t - tt[right]), left, right)
    good_time = (
        (t >= tt[0])
        & (t <= tt[-1])
        & (abs(t - tt[near]) <= max_tcp_skew_sec * 1e9)
    )
    rows = np.flatnonzero(good_time)
    if not len(rows):
        raise ValueError(
            "OptiTrack and right TCP have no overlapping timestamps"
        )

    # Compare points in the rotating tool frame, not the camera frame.
    tcp_position = p[near[rows]]
    opti_from_base = _WORLD_FROM_OPTITRACK.inv() * _WORLD_FROM_RIGHT_BASE
    origin = opti_from_base.apply(
        tcp_position
    ) + _WORLD_FROM_OPTITRACK.inv().apply(
        _RIGHT_BASE_IN_WORLD - _OPTITRACK_IN_WORLD
    )
    tool_from_opti = (
        opti_from_base * Rotation.from_quat(quat[near[rows]])
    ).inv()
    relative = xyz[rows] - origin[:, None, :]
    offsets = np.stack(
        [tool_from_opti.apply(relative[:, i, :]) for i in range(len(slots))],
        axis=1,
    )
    offsets[np.linalg.norm(offsets, axis=2) > max_tool_offset_m] = np.nan

    fit_rows = _representative_frames(tcp_position)
    fit_offsets = offsets[fit_rows]
    points = fit_offsets[np.isfinite(fit_offsets).all(axis=2)]
    if len(points) < 12:
        raise ValueError(
            "Not enough nearby OptiTrack measurements to identify a target"
        )

    def has_motion_support(center: np.ndarray) -> bool:
        within = (
            np.linalg.norm(fit_offsets - center, axis=2)
            <= position_tolerance_m
        )
        hits = within.any(axis=1)
        return bool(
            hits.sum() >= max(12, 0.6 * len(fit_rows))
            and np.max(np.ptp(tcp_position[fit_rows[hits]], axis=0)) >= 0.05
        )

    center = _strongest_offset(points, position_tolerance_m)
    if not has_motion_support(center):
        raise ValueError(
            "No marker consistently follows the right TCP over at least 5 cm of motion; "
            "check target visibility or use --marker-idx after verifying its identity"
        )
    other = points[
        np.linalg.norm(points - center, axis=1) > position_tolerance_m
    ]
    fit_close = (
        np.linalg.norm(fit_offsets - center, axis=2) <= position_tolerance_m
    )
    often_ambiguous = (fit_close.sum(axis=1) > 1).mean() >= 0.25
    second_offset = len(other) >= 12 and has_motion_support(
        _strongest_offset(other, position_tolerance_m)
    )
    if often_ambiguous or second_offset:
        raise ValueError(
            "Multiple markers appear fixed to the right tool; use --marker-idx "
            "to explicitly select the point the subject looked at"
        )

    within = np.linalg.norm(offsets - center, axis=2) <= position_tolerance_m
    unique = within.sum(axis=1) == 1
    if not unique.any():
        raise ValueError("No frames have a unique target marker")
    chosen_slots = within[unique].argmax(axis=1)
    result = np.full((len(markers), 3), np.nan)
    result[rows[unique]] = xyz[rows[unique], chosen_slots]
    logger.info(
        "Automatic OptiTrack target: selected %d/%d frames; %d ambiguous; slots %s",
        unique.sum(),
        len(markers),
        (within.sum(axis=1) > 1).sum(),
        {
            slots[i]: int(n)
            for i, n in enumerate(np.bincount(chosen_slots))
            if n
        },
    )
    return result

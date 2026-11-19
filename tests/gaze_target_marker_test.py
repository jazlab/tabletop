"""Target selection regressions using ROS-independent CSV-shaped data."""

import numpy as np
import pandas as pd
import pytest
from scipy.spatial.transform import Rotation

from tabletop_py.gaze.preprocess import (
    MARKER_DATA_COLS,
    format_marker_columns,
    interpolate_selected_marker,
)
from tabletop_py.gaze.target_marker import select_target_marker


def _frames(*, target_separation=None, target_present=True):
    n = 401
    t = np.linspace(0, 1, n)
    stamps = {
        "header.stamp.sec": np.full(n, 1_700_000_000),
        "header.stamp.nanosec": np.arange(n) * 2_000_000,
    }
    p = np.column_stack([0.2 * t, -0.1 * t**2, 0.2 + 0.18 * t])
    rotation = Rotation.from_rotvec(np.column_stack([t / 4, t * 0, t * 0]))
    tcp = pd.DataFrame({**stamps, "header.frame_id": "right_base"})
    for ax, values in zip("xyz", p.T):
        tcp[f"pose.position.{ax}"] = values
    for ax, values in zip("xyzw", rotation.as_quat().T):
        tcp[f"pose.orientation.{ax}"] = values

    def optitrack(offset):
        in_base = p + rotation.apply(offset)
        world = Rotation.from_euler("z", np.pi - 1.5707).apply(in_base)
        world += [1.3462, 0.7227, 0.3085]
        return (world - [0.983, 0.623, 0.31])[:, [1, 2, 0]]

    target = optitrack([0.06, 0.10, 0.07])
    static = np.broadcast_to(target[0] + [0.08, 0.01, 0.01], (n, 3))
    chosen = target if target_present else np.full_like(target, np.nan)
    rows = np.stack([static, chosen], axis=1)
    rows[1::2] = rows[1::2, ::-1]
    rows[200, 1] = np.nan  # Missing target: only the unrelated marker is seen.
    extra = np.full_like(target, np.nan)
    if target_separation is not None:
        extra = optitrack([0.06 + target_separation, 0.10, 0.07])
    elif target_present:
        extra[240] = (
            target[240] + 0.002
        )  # Ambiguous even though the real point is visible.
    rows = np.concatenate([rows, extra[:, None, :]], axis=1)

    markers = pd.DataFrame({**stamps, "header.frame_id": "optitrack"})
    for i in range(rows.shape[1]):
        markers[f"markers[{i}].id_type"] = 1
        for ax, values in zip("xyz", rows[:, i, :].T):
            markers[f"markers[{i}].translation.{ax}"] = values
    return markers, tcp, target


def test_target_follows_tool_across_slots_without_filling_missing_frames():
    markers, tcp, target = _frames()
    result = select_target_marker(markers, tcp)
    expected = target.copy()
    expected[[200, 240]] = np.nan
    np.testing.assert_allclose(result, expected)


@pytest.mark.parametrize("separation", [0.006, 0.020, 0.060])
def test_refuses_to_pick_between_two_tool_mounted_markers(separation):
    markers, tcp, _ = _frames(target_separation=separation)
    with pytest.raises(ValueError, match="Multiple markers"):
        select_target_marker(markers, tcp)


def test_static_markers_do_not_count_as_following_the_tool():
    markers, tcp, _ = _frames(target_present=False)
    with pytest.raises(ValueError, match="No marker consistently follows"):
        select_target_marker(markers, tcp)


def test_no_tcp_timestamp_overlap_fails_instead_of_using_a_fixed_slot():
    markers, tcp, _ = _frames()
    tcp["header.stamp.sec"] += 10
    with pytest.raises(ValueError, match="no overlapping timestamps"):
        select_target_marker(markers, tcp)


def test_manual_slot_still_works_without_tcp_data():
    markers, _, _ = _frames()
    formatted = format_marker_columns(
        markers, marker_idx=0, freq=120, verify=False
    )
    np.testing.assert_allclose(
        formatted[MARKER_DATA_COLS],
        markers[[f"markers[0].translation.{axis}" for axis in "xyz"]],
    )


def test_interpolation_does_not_bridge_rejected_frames_or_timing_gaps():
    times = np.array([0.0, 0.01, 0.02, 0.03, 0.08])
    accepted = times[[0, 1, 3, 4]]
    track = pd.DataFrame(
        {"time": accepted, **{c: accepted for c in MARKER_DATA_COLS}}
    )
    points = np.array(
        [0, 0.005, 0.010, 0.011, 0.020, 0.029, 0.030, 0.031, 0.080, 0.081]
    )
    result = interpolate_selected_marker(track, times, points, tolerance=0.02)
    expected = np.array(
        [0, 0.005, 0.01, np.nan, np.nan, np.nan, 0.03, np.nan, 0.08, np.nan]
    )
    np.testing.assert_allclose(result[MARKER_DATA_COLS].iloc[:, 0], expected)

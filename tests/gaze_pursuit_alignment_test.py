"""Offline regressions for reproducible pursuit-to-hold calibration alignment."""

import numpy as np
import pandas as pd
import pytest

from tabletop_py.gaze.pursuit_alignment import (
    interpolate,
    main,
    motion_bounds,
    previous,
    timeline,
)


def _log():
    events = [(10, 0, "Waiting 5 seconds before moving to get baseline")]
    for i in range(1, 28):
        events.extend(
            [
                (200 + i, 0, f"STATIC_HOLD_START point={i}"),
                (200 + i, 500_000_000, f"STATIC_HOLD_END point={i}"),
            ]
        )
    return pd.DataFrame(events, columns=["stamp.sec", "stamp.nanosec", "msg"])


def test_timeline_requires_one_complete_nonoverlapping_session():
    log = _log()
    wait, holds = timeline(log.sample(frac=1, random_state=8))
    assert wait == 10_000_000_000 and holds[27]["END"] == 227_500_000_000
    for bad in (
        log.iloc[:-1],
        pd.concat([log, log.iloc[[1]]]),
        pd.concat([log, log.iloc[[0]]]),
    ):
        with pytest.raises(ValueError):
            timeline(bad)
    log.loc[2, "stamp.sec"] = log.loc[3, "stamp.sec"]
    with pytest.raises(ValueError, match="overlapping"):
        timeline(log)


def test_interpolation_and_past_observations_do_not_bridge_dropouts():
    t = np.array([0, 0.002, 0.004, 0.006, 0.080])
    eye = np.array([[1.0], [3.0], [np.nan], [5.0], [10.0]])
    grid = np.array(
        [-0.001, 0.001, 0.002, 0.003, 0.004, 0.005, 0.010, 0.080, 0.081]
    )
    np.testing.assert_allclose(
        previous(t, eye, grid).ravel(),
        [np.nan, 1, 3, 3, np.nan, np.nan, np.nan, 10, 10],
    )
    np.testing.assert_allclose(
        interpolate(t, eye, grid, 0.003).ravel(),
        [np.nan, 2, 3, np.nan, np.nan, np.nan, np.nan, 10, np.nan],
    )


def test_wrist_bounds_exclude_static_block_and_detect_missing_motion():
    ms = np.arange(0, 110_001, 10)
    moving = (ms >= 5000) & (ms < 105_000)
    x = np.where(moving, 0.4 * np.sin(2 * np.pi * (ms - 5000) / 100_000), 0)
    tcp = pd.DataFrame(
        {
            "header.stamp.sec": ms // 1000,
            "header.stamp.nanosec": ms % 1000 * 1_000_000,
            "pose.position.x": x,
            "pose.position.y": 0.0,
            "pose.position.z": 0.0,
        }
    )
    first_hold = 108_000_000_000
    start, end = motion_bounds(tcp, 0, first_hold)
    assert abs(start / 1e9 - 5) < 0.2
    assert abs(end / 1e9 - 105) < 0.2 and end < first_hold
    tcp["pose.position.x"] = 0.0
    with pytest.raises(ValueError, match="onset"):
        motion_bounds(tcp, 0, first_hold)


def test_cli_refuses_to_replace_a_previous_alignment(tmp_path):
    out = tmp_path / "alignment.npz"
    out.write_bytes(b"previous")
    with pytest.raises(SystemExit):
        main([str(tmp_path), str(out)])
    assert out.read_bytes() == b"previous"

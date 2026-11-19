"""Geometric regression checks for the repeated-bearing pursuit."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from gaze_depth_sweep_test import kwargs
from scipy.spatial import cKDTree
from tabletop_tasks.tasks.gaze_bearing_depth import (
    DEFAULT_VIEWPOINT_OFFSET_M,
    DESIGN_DURATION_S,
    _bearing_depth_path,
    gaze_bearing_depth_offsets,
)
from tabletop_tasks.tasks.smooth_pursuit import SmoothPursuitTask


def test_path_geometry():
    path = gaze_bearing_depth_offsets()
    assert path.shape == (1101, 3)
    np.testing.assert_array_equal(path[[0, -1]], np.zeros((2, 3)))
    assert np.abs(path).max() <= 0.094
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    assert steps.min() >= 0.006
    assert steps.max() <= 0.009
    assert steps.sum() == pytest.approx(8.09, abs=0.05)
    eye = np.array(DEFAULT_VIEWPOINT_OFFSET_M)
    ray = path - eye
    direction = (ray * (-eye[1] / ray[:, 1:2]))[:, (0, 2)]
    distance = np.linalg.norm(ray, axis=1)
    neighbors = cKDTree(direction).query_ball_point(direction, 0.005)
    span = [
        np.max(np.abs(distance[ii] - distance[i]))
        for i, ii in enumerate(neighbors)
    ]
    assert np.mean(np.asarray(span) >= 0.06) > 0.97
    assert np.mean(np.asarray(span) >= 0.10) > 0.72


def test_design_clock_has_no_sharp_interior_acceleration_spikes():
    t = np.linspace(0.0, DESIGN_DURATION_S, 7501)
    center = (t >= 2.0) & (t < DESIGN_DURATION_S - 2.0)
    path = _bearing_depth_path(t, np.array(DEFAULT_VIEWPOINT_OFFSET_M))
    velocity = np.gradient(path, t, axis=0)
    acceleration = np.gradient(velocity, t, axis=0)
    jerk = np.gradient(acceleration, t, axis=0)
    assert np.linalg.norm(acceleration[center], axis=1).max() < 0.18
    # The original 4 mm blend exceeded 2.6 m/s³ on this same design clock.
    assert np.linalg.norm(jerk[center], axis=1).max() < 0.5


@pytest.mark.parametrize(
    "eye", [(0, 0, 0), (0, -0.09, 0), (0, -0.5), (0, -0.5, np.nan)]
)
def test_invalid_viewpoint_is_rejected(eye):
    with pytest.raises(ValueError, match="[Vv]iewpoint"):
        gaze_bearing_depth_offsets(viewpoint_offset_from_center_led_m=eye)


def test_calibration_preset_keeps_closer_grid_and_orientation():
    baseline = kwargs("smooth_pursuit_sin_static_forward_five")
    config = kwargs("smooth_pursuit_bearing_depth_forward_five")
    for key in (
        "object_id",
        "robot_name",
        "static_hold_seconds",
        "static_half_span_m",
    ):
        assert config[key] == baseline[key]
    assert config["pursuit_duration_seconds"] == 150.0
    assert (
        config["motion_kwargs"]["center_pose"]
        == baseline["motion_kwargs"]["center_pose"]
    )

    commander = SimpleNamespace(get_logger=MagicMock())
    task = SmoothPursuitTask(commander, **config)
    task.log = MagicMock()
    goals = task.generate_bearing_depth(**config["motion_kwargs"])
    assert goals[0] == goals[-1]
    assert all(g.header.frame_id == "world" for g in goals)
    assert all(g.pose.orientation == goals[0].pose.orientation for g in goals)
    assert len(task._static_targets(goals[0])) == 27

    config["num_repetitions"] = 2
    with pytest.raises(ValueError, match="exactly once"):
        SmoothPursuitTask(commander, **config)

"""Geometry and sequencing checks without a robot or ROS graph."""

import asyncio
from contextlib import asynccontextmanager
from itertools import product
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, call

import numpy as np
import pytest
import yaml
from tabletop_tasks.tasks import smooth_pursuit
from tabletop_tasks.tasks.gaze_depth_sweep import gaze_depth_sweep_offsets

CONFIG = Path(__file__).resolve().parent.parent / "config"


def kwargs(name):
    return yaml.safe_load((CONFIG / f"{name}.yaml").read_text())["tasks"][0][
        "kwargs"
    ]


def test_sweep_revisits_each_sightline_at_both_depths():
    points = gaze_depth_sweep_offsets()
    assert points.shape == (1001, 3)
    np.testing.assert_array_equal(points[[0, -1]], np.zeros((2, 3)))
    assert np.abs(points).max() < 0.100
    projected = points[:, (0, 2)] / (1 + points[:, 1:2] / 0.50)
    for anchor in product((-0.064, 0, 0.064), repeat=2):
        near = np.linalg.norm(projected - anchor, axis=1) <= 0.014
        assert (points[near, 1] < -0.072).any()
        assert (points[near, 1] > 0.072).any()
    step = np.linalg.norm(np.diff(points, axis=0), axis=1)
    # Chords around the tighter bends are shorter than the equal arc steps.
    # Keep every chord well resolved but avoid nearly coincident LIN goals.
    assert step.min() > 0.004
    assert step.max() < 0.008


def test_sweep_validates_dimensions():
    with pytest.raises(ValueError, match="in front"):
        gaze_depth_sweep_offsets(nominal_viewing_distance_m=0.05)
    with pytest.raises(ValueError, match="pursuit envelope"):
        gaze_depth_sweep_offsets(viewing_grid_half_span_m=0.16)


def test_config_keeps_original_hold_grid_and_fixed_orientation():
    old_cfg = kwargs("smooth_pursuit_sin_static")
    new_cfg = kwargs("smooth_pursuit_depth_sweep")
    assert new_cfg["pursuit_duration_seconds"] == 150.0
    for key in (
        "static_hold_seconds",
        "static_half_span_m",
        "object_id",
        "robot_name",
    ):
        assert new_cfg[key] == old_cfg[key]
    assert (
        new_cfg["motion_kwargs"]["center_pose"]
        == old_cfg["motion_kwargs"]["center_pose"]
    )
    task = smooth_pursuit.SmoothPursuitTask(
        SimpleNamespace(get_logger=MagicMock()), **new_cfg
    )
    task.log = MagicMock()
    goals = task.generate_depth_sweep(**new_cfg["motion_kwargs"])
    assert goals[0] == goals[-1]
    assert all(g.header.frame_id == "world" for g in goals)
    assert all(g.pose.orientation == goals[0].pose.orientation for g in goals)
    with pytest.raises(ValueError, match="longer than"):
        task._retime_pursuit(SimpleNamespace(duration=151.0))


@pytest.mark.parametrize(
    ("preset", "event"),
    [
        ("smooth_pursuit_depth_sweep", "DEPTH_SWEEP"),
        ("smooth_pursuit_bearing_depth_forward_five", "BEARING_DEPTH"),
    ],
)
def test_one_sweep_to_same_static_block(monkeypatch, preset, event):
    approach, pursuit, retimed = [object()], [object()], [object()]
    planner = SimpleNamespace(
        manually_attach_object=AsyncMock(),
        plan_and_move=AsyncMock(),
        plan=AsyncMock(side_effect=[approach, pursuit]),
        move=AsyncMock(),
    )

    @asynccontextmanager
    async def presentation(_group):
        yield planner

    async def pursue():
        await asyncio.Event().wait()

    commander = SimpleNamespace(
        get_logger=MagicMock(),
        _moveit=MagicMock(),
        presentation_manipulation_context=presentation,
        validate_task_start_state=MagicMock(),
        occlude_smartglass=AsyncMock(),
        reveal_smartglass=AsyncMock(),
        smooth_pursuit_and_reward=pursue,
    )
    task = smooth_pursuit.SmoothPursuitTask(
        commander, **kwargs(preset)
    )
    task.log = MagicMock()
    start = smooth_pursuit.pose_stamped_msg(
        frame_id="world", position=[0.9525, 0.6, 0.53]
    )
    task._motion_fn = lambda **_geometry: [start, object(), start]
    task._retime_pursuit = MagicMock(return_value=retimed)
    task._plan_static_block = AsyncMock(return_value=[object()])
    task._execute_static_block = AsyncMock()
    monkeypatch.setattr(
        smooth_pursuit,
        "asyncio",
        SimpleNamespace(
            TaskGroup=asyncio.TaskGroup,
            wait=asyncio.wait,
            FIRST_COMPLETED=asyncio.FIRST_COMPLETED,
            sleep=AsyncMock(),
        ),
    )
    asyncio.run(task.run())
    assert planner.move.await_args_list == [call(approach), call(retimed)]
    assert planner.plan.await_args_list[1].kwargs["loop"] is False
    assert planner.plan.await_args_list[1].kwargs["apply_smoothing"] is True
    options = planner.plan.await_args_list[1].kwargs
    if event == "BEARING_DEPTH":
        assert options["planning_velocity_scaling_factor"] == 0.25
        assert options["planning_acceleration_scaling_factor"] == 0.20
    else:
        assert "planning_velocity_scaling_factor" not in options
    task._retime_pursuit.assert_called_once_with(pursuit)
    task._plan_static_block.assert_awaited_once_with(planner, start, retimed)
    task._execute_static_block.assert_awaited_once_with(
        planner, task._plan_static_block.return_value
    )
    assert call(f"{event}_START") in task.log.call_args_list
    assert call(f"{event}_END") in task.log.call_args_list

"""Validate static gaze grid sequencing without ROS nodes or hardware."""

import asyncio
from contextlib import asynccontextmanager
from itertools import islice
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest
import yaml
from tabletop_rig.exceptions import PlanningError
from tabletop_tasks.tasks import smooth_pursuit


def make_grid(commander):
    config = (
        Path(__file__).resolve().parents[1]
        / "config/smooth_pursuit_static_grid.yaml"
    )
    kwargs = yaml.safe_load(config.read_text())["tasks"][0]["kwargs"]
    return smooth_pursuit.SmoothPursuitTask(commander, **kwargs)


def test_grid_visits_full_volume_in_interleaved_sweeps():
    task = make_grid(SimpleNamespace(get_logger=MagicMock()))
    center = smooth_pursuit.pose_stamped_msg(
        **task._motion_kwargs["center_pose"]
    )
    targets = list(islice(task._timed_grid_targets(center), 217))
    xyz = np.array(
        [
            (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            for p in targets
        ]
    )
    assert len(np.unique(xyz[:216], axis=0)) == 216
    for values, middle in zip(xyz[:216].T, (0.9525, 0.6, 0.53), strict=True):
        levels, counts = np.unique(values, return_counts=True)
        assert levels == pytest.approx(middle + np.linspace(-0.07, 0.07, 6))
        assert list(counts) == [36] * 6
    for sweep in np.split(xyz[:216], 8):
        assert all(len(np.unique(values)) == 3 for values in sweep.T)
    assert max(np.linalg.norm(np.diff(xyz, axis=0), axis=1)) <= 0.056001
    assert all(
        p.header.frame_id == "world"
        and p.pose.orientation == center.pose.orientation
        for p in targets
    )


class Move(list):
    duration = 0.5


@pytest.mark.parametrize("fault", [None, "fallback", "planning", "cancelled"])
def test_timed_grid_preserves_hold_duration_and_safety_cleanup(
    monkeypatch, fault
):
    now = 0.0
    plans = []
    executed = []
    held = []
    fallback_requested = False
    approach_legs = 3 if fault == "fallback" else 2

    async def plan(**kwargs):
        nonlocal fallback_requested
        commander.reveal_smartglass.assert_not_awaited()
        if fault == "planning" and len(plans) == 4:
            raise PlanningError("blocked path", "right_manipulator")
        if fault == "fallback" and not fallback_requested:
            fallback_requested = True
            raise PlanningError(
                "direct center path blocked", "right_manipulator"
            )
        if fault == "fallback" and not plans:
            assert kwargs["goal"] == "present"
        if plans:
            assert kwargs["start_state"] is plans[-1][-1]
            if fault != "fallback" or len(plans) > 1:
                assert kwargs["planning_pipeline"] == "linear"
            assert kwargs["acceleration_scaling_factor"] == 0.10
        assert kwargs["use_cache"] is False
        move = Move([object(), object()])
        plans.append(move)
        return move

    async def move(trajectory):
        nonlocal now
        if len(executed) >= approach_legs:
            commander.reveal_smartglass.assert_awaited_once()
            now += trajectory.duration + 0.05  # real dispatch overhead
        else:
            commander.reveal_smartglass.assert_not_awaited()
            now += 4  # occluded approach legs are outside the budget
        executed.append(trajectory)

    async def sleep(duration):
        nonlocal now
        assert duration == 0.5
        if fault == "cancelled":
            raise asyncio.CancelledError()
        held.append(duration)
        now += duration

    manipulator = SimpleNamespace(
        manually_attach_object=AsyncMock(),
        plan_and_move=AsyncMock(),
        plan=plan,
        move=move,
    )

    @asynccontextmanager
    async def context(name):
        assert name == "right_manipulator"
        yield manipulator

    commander = SimpleNamespace(
        get_logger=MagicMock(),
        _moveit=MagicMock(),
        presentation_manipulation_context=context,
        validate_task_start_state=MagicMock(),
        occlude_smartglass=AsyncMock(),
        reveal_smartglass=AsyncMock(),
        smooth_pursuit_and_reward=AsyncMock(),
    )
    commander._moveit.allow_collision.return_value = [("a", "b")]

    async def park_or_stage(**_kwargs):
        if manipulator.plan_and_move.await_count == 2:
            assert commander.occlude_smartglass.await_count == 2
            assert held

    manipulator.plan_and_move.side_effect = park_or_stage
    task = make_grid(commander)
    monkeypatch.setattr(task, "log", MagicMock())
    monkeypatch.setattr(smooth_pursuit, "monotonic", lambda: now)
    monkeypatch.setattr(
        smooth_pursuit, "asyncio", SimpleNamespace(sleep=sleep)
    )

    if fault == "planning":
        with pytest.raises(RuntimeError, match="target 4; no grid shown"):
            asyncio.run(task.run())
        commander.reveal_smartglass.assert_not_awaited()
        assert not executed
    elif fault == "cancelled":
        with pytest.raises(asyncio.CancelledError):
            asyncio.run(task.run())
        assert commander.occlude_smartglass.await_count == 2
        assert not any(
            "STATIC_HOLD_END" in call.args[0]
            for call in task.log.call_args_list
        )
    else:
        asyncio.run(task.run())
        assert len(plans) == 299 + approach_legs
        assert len(held) == 286
        assert len(executed) == len(held) + approach_legs - 1
        assert 299 < now - 4 * approach_legs <= 300
        assert sum(
            "STATIC_HOLD_END" in call.args[0]
            for call in task.log.call_args_list
        ) == len(held)
        assert commander.occlude_smartglass.await_count == 2
    commander.validate_task_start_state.assert_called_once_with(
        allow_right_presentation_wall=True,
        attached_object_id="smooth_pursuit_object",
    )
    commander.smooth_pursuit_and_reward.assert_not_awaited()
    assert manipulator.plan_and_move.await_count == (1 if fault in ("planning", "cancelled") else 2)
    commander._moveit.disallow_collision.assert_called_once_with(
        ("a",), ("b",)
    )

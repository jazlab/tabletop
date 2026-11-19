"""Verify approach limits without a ROS graph or robot connection."""

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, call

import numpy as np
import pytest
import yaml
from moveit_msgs.msg import MoveItErrorCodes
from tabletop_rig.exceptions import PlanningError, PlanningPipelineError
from tabletop_tasks.tasks import smooth_pursuit


@pytest.mark.parametrize("fixture_collision", [False, True])
@pytest.mark.parametrize(
    "static_block, laps, tracking_ends_early",
    [(False, 1, False), (True, 1, False), (True, 2, False), (False, 1, True), (True, 1, True)],
)
def test_loaded_start_checked_before_planning_and_only_approach_is_slow(
    monkeypatch, fixture_collision, static_block, laps, tracking_ends_early
) -> None:
    first_waypoint = smooth_pursuit.pose_stamped_msg(
        frame_id="world",
        position=[0.9525, 0.6, 0.53],
        rpy=[1.5707, 1.5707, 0.0],
    )
    next_waypoint = object()
    approach = [object(), object()]
    pursuit = [object(), object()]
    holds = [[object(), object()] for _ in range(27)] if static_block else []
    manipulator = SimpleNamespace(
        manually_attach_object=AsyncMock(),
        plan_and_move=AsyncMock(),
        plan=AsyncMock(side_effect=[approach, pursuit, *holds]),
        move=AsyncMock(),
    )

    @asynccontextmanager
    async def manipulation_context(group_name):
        assert group_name == "right_manipulator"
        yield manipulator

    async def wait_for_pursuit():
        if not tracking_ends_early:
            await asyncio.Event().wait()

    async def execute(trajectory):
        if trajectory is pursuit and tracking_ends_early:
            await asyncio.Event().wait()

    async def park_or_stage(**_kwargs):
        if manipulator.plan_and_move.await_count == 2:
            # The new motion is outside acquisition and only starts opaque.
            assert commander.occlude_smartglass.await_count == 2
            assert manipulator.move.await_args_list[-1] == call(holds[-1] if holds else pursuit)

    def validate_start(**kwargs):
        manipulator.manually_attach_object.assert_awaited_once_with(
            "smooth_pursuit_object"
        )
        assert kwargs == {
            "allow_right_presentation_wall": True,
            "attached_object_id": "smooth_pursuit_object",
        }
        manipulator.plan_and_move.assert_not_awaited()
        if fixture_collision:
            raise RuntimeError("fixture in collision")

    commander = SimpleNamespace(
        get_logger=MagicMock(),
        _moveit=MagicMock(),
        presentation_manipulation_context=manipulation_context,
        validate_task_start_state=MagicMock(side_effect=validate_start),
        occlude_smartglass=AsyncMock(),
        reveal_smartglass=AsyncMock(),
        smooth_pursuit_and_reward=wait_for_pursuit,
    )
    manipulator.move.side_effect = execute
    manipulator.plan_and_move.side_effect = park_or_stage
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
    task = smooth_pursuit.SmoothPursuitTask(
        commander,
        motion_type="sin",
        motion_kwargs={},
        num_repetitions=laps,
        object_id="smooth_pursuit_object",
        robot_name="right_manipulator",
        velocity_scaling_factor=0.075,
        static_hold_seconds=1.0 if static_block else None,
    )
    monkeypatch.setattr(task, "log", MagicMock())
    monkeypatch.setattr(
        task, "_motion_fn", lambda: [first_waypoint, next_waypoint]
    )

    if fixture_collision:
        with pytest.raises(RuntimeError, match="fixture in collision"):
            asyncio.run(task.run())
        commander._moveit.allow_collision.assert_not_called()
        manipulator.plan_and_move.assert_not_awaited()
        return

    approach_options = dict(
        velocity_scaling_factor=0.05,
        acceleration_scaling_factor=0.10,
        use_cache=False,
    )
    expected_park = call(
        goal="fetched", cache_trajectories=False, **approach_options
    )
    if tracking_ends_early:
        if static_block:
            with pytest.raises(RuntimeError, match="static block skipped"):
                asyncio.run(task.run())
        else:
            asyncio.run(task.run())
        assert manipulator.plan_and_move.await_args_list == [expected_park]
        assert not any("GAZE_RETURN_" in c.args[0] for c in task.log.call_args_list)
        return

    asyncio.run(task.run())
    commander.validate_task_start_state.assert_called_once()
    assert manipulator.plan_and_move.await_args_list == [expected_park, expected_park]
    assert manipulator.plan.await_args_list[:2] == [
        call(
            goal=first_waypoint,
            group_name="right_manipulator",
            **approach_options,
        ),
        call(
            goals=[next_waypoint],
            group_name="right_manipulator",
            start_state=approach[-1],
            velocity_scaling_factor=0.075,
            post_process_after_concat=True,
            loop=True,
            planning_pipeline="linear",
            use_cache=False,
        ),
    ]
    assert manipulator.move.await_args_list == [
        call(approach),
        *([call(pursuit)] * laps),
        *(call(hold) for hold in holds),
    ]
    if not static_block:
        assert manipulator.plan.await_count == 2
        return

    target_calls = manipulator.plan.await_args_list[2:]
    assert len(target_calls) == 27
    targets = [entry.kwargs["goal"] for entry in target_calls]
    xyz = [
        (p.pose.position.x, p.pose.position.y, p.pose.position.z)
        for p in targets
    ]
    assert len(set(xyz)) == 27
    for col, center in zip(zip(*xyz), [0.9525, 0.6, 0.53]):
        assert sorted(set(col)) == pytest.approx(
            [center - 0.07, center, center + 0.07]
        )
    assert all(p.header.frame_id == "world" for p in targets)
    assert all(
        p.pose.orientation == first_waypoint.pose.orientation for p in targets
    )
    for source, entry in zip([pursuit, *holds], target_calls):
        assert entry.kwargs["start_state"] is source[-1]
        assert entry.kwargs["planning_pipeline"] == "linear"
        assert entry.kwargs["use_cache"] is False
    assert smooth_pursuit.asyncio.sleep.await_args_list == [
        call(5),
        *([call(1.0)] * 27),
    ]
    assert (
        sum("STATIC_HOLD_START" in c.args[0] for c in task.log.call_args_list)
        == 27
    )
    assert (
        sum("STATIC_HOLD_END" in c.args[0] for c in task.log.call_args_list)
        == 27
    )
    commander.reveal_smartglass.assert_awaited_once()
    assert commander.occlude_smartglass.await_count == 2


def test_static_planning_failure_names_target_and_does_not_execute() -> None:
    task = smooth_pursuit.SmoothPursuitTask(
        SimpleNamespace(get_logger=MagicMock()),
        motion_type="sin",
        motion_kwargs={},
        num_repetitions=1,
        object_id="smooth_pursuit_object",
        robot_name="right_manipulator",
        static_hold_seconds=1.0,
    )
    manipulator = SimpleNamespace(
        plan=AsyncMock(
            side_effect=PlanningError("invalid route", "right_manipulator")
        ),
        move=AsyncMock(),
    )
    center = smooth_pursuit.pose_stamped_msg(position=[0.9525, 0.6, 0.53])
    with pytest.raises(
        RuntimeError, match="Could not plan static fixation 1/27"
    ):
        asyncio.run(task._plan_static_block(manipulator, center, [object()]))
    manipulator.move.assert_not_awaited()


@pytest.mark.parametrize("start_collision", [False, True])
def test_return_plan_failure_does_not_command_another_move(start_collision):
    task = smooth_pursuit.SmoothPursuitTask(
        SimpleNamespace(get_logger=MagicMock()),
        motion_type="sin",
        motion_kwargs={},
        num_repetitions=1,
        object_id="smooth_pursuit_object",
        robot_name="right_manipulator",
    )
    task.log = MagicMock()
    error = (
        PlanningPipelineError(MoveItErrorCodes(val=MoveItErrorCodes.START_STATE_IN_COLLISION), group_name="right_manipulator")
        if start_collision
        else PlanningError("no route", "right_manipulator")
    )
    arm = SimpleNamespace(plan_and_move=AsyncMock(side_effect=error))
    if start_collision:
        with pytest.raises(PlanningPipelineError):
            asyncio.run(task._return_fixture_to_fetched(arm, {}))
    else:
        asyncio.run(task._return_fixture_to_fetched(arm, {}))
        assert task.log.call_args.args[0].startswith("GAZE_RETURN_SKIPPED")
    arm.plan_and_move.assert_awaited_once_with(goal="fetched", cache_trajectories=False)


def test_one_lap_configs_preserve_the_standard_sinusoid() -> None:
    directory = Path(__file__).resolve().parent.parent / "config"

    def load(name):
        conf = yaml.safe_load((directory / (name + ".yaml")).read_text())
        return conf["tasks"][0]["kwargs"]

    original = load("smooth_pursuit_sin")
    assert original["num_repetitions"] > 1
    one_lap = {**original, "num_repetitions": 1}
    assert load("smooth_pursuit_sin_one_lap") == one_lap
    assert load("smooth_pursuit_sin_static") == {
        **one_lap,
        "static_hold_seconds": 1.0,
        "static_half_span_m": 0.07,
    }
    standard = load("smooth_pursuit_sin_static")
    extended = load("smooth_pursuit_sin_two_laps_static")
    assert extended == {
        **standard,
        "motion_kwargs": {
            **standard["motion_kwargs"],
            "num_revolutions": 20,
            "num_segments": 1600,
        },
    }

    def positions(config):
        task = smooth_pursuit.SmoothPursuitTask(
            SimpleNamespace(get_logger=MagicMock()), **config
        )
        task.log = MagicMock()
        return np.array([
            (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            for p in task.generate_sin(**config["motion_kwargs"])
        ])

    short, long = positions(standard), positions(extended)
    mid = len(short) - 1
    np.testing.assert_allclose(long[:mid + 1], short)
    assert np.linalg.norm(long[mid:] - short, axis=1).mean() > 0.05
    # No jump or cusp where the old path would have returned to center.
    assert np.linalg.norm(long[mid + 1] - 2 * long[mid] + long[mid - 1]) < 0.004


@pytest.mark.parametrize(
    "base, suffix, offset_m, center_y",
    [
        ("sin_static", "forward", 0.1524, 0.4476),
        ("sin_static", "forward_five", 0.127, 0.473),
        ("sin_two_laps_static", "forward_five", 0.127, 0.473),
    ],
)
def test_forward_preset_translates_the_entire_path_and_grid(
    base, suffix, offset_m, center_y
) -> None:
    directory = Path(__file__).resolve().parent.parent / "config"

    def load(file):
        return yaml.safe_load((directory / file).read_text())["tasks"][0]["kwargs"]

    original = load(f"smooth_pursuit_{base}.yaml")
    forward = load(f"smooth_pursuit_{base}_{suffix}.yaml")
    expected_motion = {**original["motion_kwargs"]}
    expected_motion["center_pose"] = {
        **expected_motion["center_pose"],
        "position": [0.9525, center_y, 0.53],
    }
    assert forward == {
        **original,
        "motion_kwargs": expected_motion,
        "static_hold_seconds": 0.5,
    }

    def waypoints(conf):
        task = smooth_pursuit.SmoothPursuitTask(
            SimpleNamespace(get_logger=MagicMock()), **conf
        )
        task.log = MagicMock()
        path = task.generate_sin(**conf["motion_kwargs"])
        hold = task._static_targets(path[0])
        return path, hold

    original_path, original_holds = waypoints(original)
    forward_path, forward_holds = waypoints(forward)

    def positions(poses):
        return np.array(
            [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses]
        )

    shift = np.array([0.0, -offset_m, 0.0])
    assert len(forward_path) == original["motion_kwargs"]["num_segments"] + 1
    assert len(forward_holds) == 27
    np.testing.assert_allclose(
        positions(forward_path), positions(original_path) + shift, atol=1e-12
    )
    # Compare the grid set, not the order of numerically tied short steps.
    observed = {tuple(row) for row in positions(forward_holds).round(9)}
    expected = {tuple(row) for row in (positions(original_holds) + shift).round(9)}
    assert observed == expected
    assert all(
        p.pose.orientation == forward_path[0].pose.orientation
        for p in forward_path + forward_holds
    )
    if base == "sin_two_laps_static":
        short, _ = waypoints(load("smooth_pursuit_sin_static_forward_five.yaml"))
        first = positions(short)
        extended = positions(forward_path)
        mid = len(first) - 1
        np.testing.assert_allclose(extended[: mid + 1], first)
        assert np.linalg.norm(extended[mid:] - first, axis=1).mean() > 0.05
        assert np.linalg.norm(extended[mid + 1] - 2 * extended[mid] + extended[mid - 1]) < 0.004


def test_cancelling_a_hold_does_not_mark_it_complete_or_start_another(
    monkeypatch,
) -> None:
    task = smooth_pursuit.SmoothPursuitTask(
        SimpleNamespace(get_logger=MagicMock()),
        motion_type="sin",
        motion_kwargs={},
        num_repetitions=1,
        object_id="smooth_pursuit_object",
        robot_name="right_manipulator",
        static_hold_seconds=1.0,
    )
    logger = MagicMock()
    monkeypatch.setattr(task, "log", logger)
    monkeypatch.setattr(
        smooth_pursuit,
        "asyncio",
        SimpleNamespace(sleep=AsyncMock(side_effect=asyncio.CancelledError)),
    )
    manipulator = SimpleNamespace(move=AsyncMock())
    plan = [
        (smooth_pursuit.pose_stamped_msg(position=xyz), [object()])
        for xyz in ([0.95, 0.6, 0.53], [0.95, 0.6, 0.60])
    ]
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(task._execute_static_block(manipulator, plan))
    manipulator.move.assert_awaited_once_with(plan[0][1])
    logger.assert_called_once()
    assert logger.call_args.args[0].startswith("STATIC_HOLD_START point=1")

"""Regression tests for Commander rig-wide UR safety handling."""

import asyncio
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest
import tabletop_rig.interfaces.moveit.plan_and_execute as plan_module
import tabletop_rig.manipulation_preflight as preflight_module
import tabletop_rig.nodes  # noqa: F401
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from tabletop_interfaces.msg import TeensySensor
from tabletop_rig.exceptions import (
    ExecutionInterruptedError,
    ExecutionStoppedError,
    ManipulationContextExitedError,
    NotSafeToExecuteError,
    PlanningError,
    RigExecutionSafetyError,
    URSafetyViolationError,
)
from tabletop_rig.interfaces.moveit.moveit import MoveItInterface
from tabletop_rig.interfaces.moveit.object_manipulation import (
    ManipulationState,
    ObjectManipulationInterface,
)
from tabletop_rig.interfaces.moveit.plan_and_execute import (
    PlanAndExecuteInterface,
)
from tabletop_rig.interfaces.moveit.requests import PlanRequest
from tabletop_rig.nodes.commander import (
    Commander,
    ManipulationContextManager,
    _wait_for_task_or_safety_fault,
    handle_interruptions,
)
from tabletop_tasks.tasks.base import BaseObjectInteractionTask
from ur_dashboard_msgs.msg import SafetyMode


def _make_commander(*, simulate: bool) -> Commander:
    commander = object.__new__(Commander)
    commander._rig_safety_fault_lock = threading.Lock()
    commander._rig_safety_fault = None
    commander._rig_safety_monitor_armed = True
    commander._rig_safety_loop = None
    commander._rig_safety_event = None
    commander.log = MagicMock()
    commander.param = MagicMock(return_value=simulate)
    commander._manipulation_contexts = {
        name: SimpleNamespace(
            _manipulator=SimpleNamespace(stop_execution=MagicMock()),
            _ur=SimpleNamespace(stop_program=MagicMock()),
        )
        for name in ("left_manipulator", "right_manipulator")
    }
    return commander


class TestRigSafetyCallback:
    def test_normal_mode_does_nothing(self):
        commander = _make_commander(simulate=False)
        commander._ur_safety_mode_callback(
            "left_manipulator", SafetyMode(mode=SafetyMode.NORMAL)
        )
        assert not commander.rig_safety_faulted
        for context in commander._manipulation_contexts.values():
            context._manipulator.stop_execution.assert_not_called()
            context._ur.stop_program.assert_not_called()

    def test_violation_stops_both_real_arms_and_latches(self):
        commander = _make_commander(simulate=False)
        commander._ur_safety_mode_callback(
            "left_manipulator", SafetyMode(mode=SafetyMode.VIOLATION)
        )

        with pytest.raises(URSafetyViolationError) as exc_info:
            commander.raise_if_rig_safety_faulted()
        assert exc_info.value.robot_name == "left_manipulator"
        assert exc_info.value.safety_mode == "VIOLATION"
        for context in commander._manipulation_contexts.values():
            context._manipulator.stop_execution.assert_called_once_with()
            context._ur.stop_program.assert_called_once_with()

        # Repeated status messages must not flood stop requests or replace the
        # original arm/mode that caused the session abort.
        commander._ur_safety_mode_callback(
            "right_manipulator", SafetyMode(mode=SafetyMode.FAULT)
        )
        assert commander._get_rig_safety_fault() is exc_info.value
        for context in commander._manipulation_contexts.values():
            context._manipulator.stop_execution.assert_called_once_with()
            context._ur.stop_program.assert_called_once_with()

    def test_mock_violation_cancels_execution_without_dashboard_stop(self):
        commander = _make_commander(simulate=True)
        commander._ur_safety_mode_callback(
            "right_manipulator", SafetyMode(mode=SafetyMode.VIOLATION)
        )
        assert commander.rig_safety_faulted
        for context in commander._manipulation_contexts.values():
            context._manipulator.stop_execution.assert_called_once_with()
            context._ur.stop_program.assert_not_called()

    def test_monitor_is_not_armed_during_startup_recovery(self):
        commander = _make_commander(simulate=False)
        commander._rig_safety_monitor_armed = False
        commander._ur_safety_mode_callback(
            "left_manipulator", SafetyMode(mode=SafetyMode.VIOLATION)
        )
        assert not commander.rig_safety_faulted


def test_task_guard_cancels_experiment_and_raises_safety_fault():
    fault = URSafetyViolationError("left_manipulator", "VIOLATION")
    experiment_cancelled = asyncio.Event()

    async def experiment():
        try:
            await asyncio.Event().wait()
        finally:
            experiment_cancelled.set()

    async def safety_waiter():
        await asyncio.sleep(0)
        raise fault

    async def run():
        commander = SimpleNamespace(wait_for_rig_safety_fault=safety_waiter)
        task = asyncio.create_task(experiment())
        with pytest.raises(URSafetyViolationError) as exc_info:
            await _wait_for_task_or_safety_fault(commander, task)
        assert exc_info.value is fault
        assert task.cancelled()
        assert experiment_cancelled.is_set()

    asyncio.run(run())


def test_pre_fetch_preserves_cache_at_normal_scaling():
    manipulator = object.__new__(ObjectManipulationInterface)
    manipulator._simulate = True
    manipulator._manipulation_state = ManipulationState.IDLE
    manipulator.log = MagicMock()
    manipulator.param = MagicMock(return_value=False)
    manipulator._get_state_goal = MagicMock(
        return_value=MagicMock(spec=RobotState)
    )
    manipulator._transition_collisions_to_allow = MagicMock(return_value=[])
    manipulator.plan_and_execute = AsyncMock(return_value=None)
    manipulator._moveit = MagicMock()

    asyncio.run(
        manipulator._fetch_or_return_transition(
            "small_object_12", ManipulationState.PRE_FETCH
        )
    )

    request = manipulator.plan_and_execute.await_args.args[0]
    assert request.use_cache is True
    assert request.apply_totg is True
    assert request.apply_smoothing is False
    assert request.mitigate_overshoot is False
    assert request.velocity_scaling_factor == 1.0
    assert request.acceleration_scaling_factor == 1.0


def test_new_plan_is_validated_after_post_processing(monkeypatch):
    class FakePlanRequestParameters:
        planning_time = 0.25

    monkeypatch.setattr(
        plan_module, "PlanRequestParameters", FakePlanRequestParameters
    )

    interface = object.__new__(PlanAndExecuteInterface)
    interface.log = MagicMock()
    interface.param = MagicMock(return_value=1.0)
    interface._moveit = SimpleNamespace(moveit_py=object())

    raw_trajectory = object()
    processed_trajectory = object()
    response = MagicMock()
    response.__bool__.return_value = True
    response.trajectory = raw_trajectory
    planning_component = MagicMock()
    planning_component.plan.return_value = response
    params = FakePlanRequestParameters()

    interface._prepare_planning_component = MagicMock(
        return_value=(planning_component, params)
    )
    interface._post_process_trajectory = MagicMock(
        return_value=processed_trajectory
    )
    interface._validate_trajectory = MagicMock()

    request = SimpleNamespace(
        group_name="left_manipulator",
        max_attempts=1,
        exp_backoff_factor=2.0,
        planning_scene=None,
    )
    result = interface._plan_pipeline(request)
    assert result is processed_trajectory
    interface._post_process_trajectory.assert_called_once_with(
        raw_trajectory, request
    )
    interface._validate_trajectory.assert_called_once_with(
        processed_trajectory
    )


def test_postprocessed_validation_disables_native_verbose_output():
    interface = object.__new__(PlanAndExecuteInterface)
    interface.log = MagicMock()
    trajectory = MagicMock()
    interface._moveit = SimpleNamespace(
        is_path_valid=MagicMock(return_value=True)
    )

    interface._validate_trajectory(trajectory)
    interface._moveit.is_path_valid.assert_called_once_with(
        trajectory, verbose=False
    )


def test_native_planning_is_serialized_across_arm_interfaces():
    shared_lock = threading.Lock()
    first_entered = threading.Event()
    release_first = threading.Event()
    second_entered = threading.Event()

    first = object.__new__(PlanAndExecuteInterface)
    second = object.__new__(PlanAndExecuteInterface)
    for interface in (first, second):
        interface._moveit = SimpleNamespace(planning_lock=shared_lock)

    def first_plan(request, *, cancel_event):
        first_entered.set()
        assert release_first.wait(1.0)
        return object(), None

    def second_plan(request, *, cancel_event):
        second_entered.set()
        return object(), None

    first._plan_impl = MagicMock(side_effect=first_plan)
    second._plan_impl = MagicMock(side_effect=second_plan)

    async def run():
        first_task = asyncio.create_task(first.plan(goal=PoseStamped()))
        assert await asyncio.to_thread(first_entered.wait, 1.0)

        second_task = asyncio.create_task(second.plan(goal=PoseStamped()))
        await asyncio.sleep(0.05)
        assert not second_entered.is_set()

        release_first.set()
        await asyncio.gather(first_task, second_task)

    asyncio.run(run())


def test_protective_stop_stops_both_real_arms_and_latches():
    commander = _make_commander(simulate=False)

    commander._ur_safety_mode_callback(
        "right_manipulator",
        SafetyMode(mode=SafetyMode.PROTECTIVE_STOP),
    )

    with pytest.raises(URSafetyViolationError) as exc_info:
        commander.raise_if_rig_safety_faulted()
    assert exc_info.value.robot_name == "right_manipulator"
    assert exc_info.value.safety_mode == "PROTECTIVE_STOP"
    for context in commander._manipulation_contexts.values():
        context._manipulator.stop_execution.assert_called_once_with()
        context._ur.stop_program.assert_called_once_with()


def test_real_execution_failure_stops_rig_and_latches():
    commander = _make_commander(simulate=False)
    error = ExecutionInterruptedError(
        "Aborted due to path tolerance violation",
        group_name="right_manipulator",
    )

    fault = commander._handle_rig_execution_fault("right_manipulator", error)

    assert isinstance(fault, RigExecutionSafetyError)
    assert commander._get_rig_safety_fault() is fault
    for context in commander._manipulation_contexts.values():
        context._manipulator.stop_execution.assert_called_once_with()
        context._ur.stop_program.assert_called_once_with()


def test_real_execution_error_is_not_retried():
    error = ExecutionInterruptedError(
        "Aborted due to path tolerance violation",
        group_name="right_manipulator",
    )
    fault = RigExecutionSafetyError("right_manipulator", error)
    attempts = 0

    @handle_interruptions
    async def operation(context):
        nonlocal attempts
        attempts += 1
        raise error

    context = SimpleNamespace(
        _simulate=False,
        _raise_if_rig_safety_faulted=MagicMock(),
        _rig_execution_fault_handler=MagicMock(return_value=fault),
        _safe_to_execute_condition=MagicMock(return_value=True),
        _recover_after_laser_stop=AsyncMock(),
        _laser_stop_pending=MagicMock(return_value=False),
        _laser_break_generation_value=MagicMock(return_value=0),
        param=MagicMock(return_value=3),
    )

    async def run():
        with pytest.raises(RigExecutionSafetyError) as exc_info:
            await operation(context)
        assert exc_info.value is fault

    asyncio.run(run())
    assert attempts == 1
    context._rig_execution_fault_handler.assert_called_once_with(error)


def test_real_pre_fetch_preserves_cache_and_normal_scaling():
    manipulator = object.__new__(ObjectManipulationInterface)
    manipulator._simulate = False
    manipulator._manipulation_state = ManipulationState.IDLE
    manipulator.log = MagicMock()
    manipulator.param = MagicMock()
    manipulator._get_state_goal = MagicMock(
        return_value=MagicMock(spec=RobotState)
    )
    manipulator._transition_collisions_to_allow = MagicMock(return_value=[])
    manipulator.plan_and_execute = AsyncMock(return_value=None)
    manipulator._moveit = MagicMock()

    asyncio.run(
        manipulator._fetch_or_return_transition(
            "small_object_16", ManipulationState.PRE_FETCH
        )
    )

    request = manipulator.plan_and_execute.await_args.args[0]
    assert request.use_cache is True
    assert request.planning_pipeline is None
    assert request.apply_totg is True
    assert request.apply_smoothing is False
    assert request.velocity_scaling_factor == 1.0
    assert request.acceleration_scaling_factor == 1.0


def test_unspecified_pipeline_tries_ptp_before_ompl_fallback():
    interface = object.__new__(PlanAndExecuteInterface)
    interface.log = MagicMock()
    interface._moveit = SimpleNamespace(
        get_current_state=MagicMock(return_value=MagicMock(spec=RobotState)),
        planning_frame="world",
    )
    params = {
        "group_name": "left_manipulator",
        "planning.default_pose_link": "left_eef",
        "trajectory_cache.use_cached_trajectories": False,
        "planning.default_max_attempts": 3,
        "planning.default_exp_backoff_factor": 2.0,
        "planning.fast_pipeline": "ptp",
        "planning.fallback_pipeline": "aps_rrt_star",
    }
    interface.param = MagicMock(side_effect=params.__getitem__)
    trajectory = MagicMock()
    interface._plan_pipeline = MagicMock(
        side_effect=[
            PlanningError("Direct path rejected", "left_manipulator"),
            trajectory,
        ]
    )

    result, cache_kwargs = interface._plan_impl(
        PlanRequest(goal=PoseStamped(), use_cache=False)
    )

    assert result is trajectory
    assert [
        call.args[0].planning_pipeline
        for call in interface._plan_pipeline.call_args_list
    ] == ["ptp", "aps_rrt_star"]
    assert cache_kwargs[0]["request"].planning_pipeline == "aps_rrt_star"


def test_real_pre_fetch_uses_installed_fingerprint_validated_branch():
    manipulator = object.__new__(ObjectManipulationInterface)
    manipulator._simulate = False
    manipulator._saved_return_states = {}
    manipulator._preflight_pre_fetch_branches = {
        "small_object_16": {"right_shoulder_pan_joint": 3.4667}
    }
    manipulator._moveit = MagicMock()
    current_state = MagicMock(spec=RobotState)
    manipulator._moveit.get_current_state.return_value = current_state
    manipulator.log = MagicMock()

    def param(name):
        if name.endswith("object_overrides.small_object_16.pre_fetch"):
            return {
                "type": "joint_positions",
                "value": {"right_shoulder_pan_joint": 3.4667},
            }
        if name == "manipulation_state_goals.pre_fetch":
            return {"type": "offset", "value": [-0.05, 0.0, -0.05]}
        raise AssertionError(f"Unexpected parameter: {name}")

    manipulator.param = MagicMock(side_effect=param)

    goal = manipulator._get_state_goal(
        ManipulationState.PRE_FETCH, "small_object_16"
    )

    assert goal is current_state
    assert current_state.joint_positions == {
        "right_shoulder_pan_joint": 3.4667
    }
    current_state.update.assert_called_once_with()
    manipulator.param.assert_not_called()


@pytest.mark.parametrize("simulate", [False, True])
def test_preflight_branches_are_installed_when_authorized(
    monkeypatch, simulate
):
    object_id = "small_object_16"
    group_name = "right_manipulator"
    positions = {
        "right_shoulder_pan_joint": 3.4667,
        "right_shoulder_lift_joint": -1.2591,
    }
    manipulator = SimpleNamespace(
        group_name=group_name,
        reachable_object_ids={object_id},
        _moveit=SimpleNamespace(
            get_joint_names=MagicMock(return_value=list(positions))
        ),
    )
    commander = SimpleNamespace(
        param=MagicMock(return_value=simulate),
        log=MagicMock(),
        _manipulation_contexts={
            group_name: SimpleNamespace(_manipulator=manipulator)
        },
    )
    report = {
        "cache_version": preflight_module._CACHE_VERSION,
        "mode": "mock_planning_only",
        "objects": {
            object_id: {
                "fingerprint": "matching-fingerprint",
                "status": "PASS",
                "selected_branch": {"joint_positions": positions},
            }
        },
    }
    monkeypatch.setattr(preflight_module, "_load_cache", lambda path: report)
    monkeypatch.setattr(
        preflight_module, "_fingerprint_payload", lambda *args: {}
    )
    monkeypatch.setattr(
        preflight_module,
        "_fingerprint",
        lambda payload: "matching-fingerprint",
    )

    results = preflight_module.load_compatible_results(
        commander, allow_real_use=True
    )

    assert results["available_object_ids"] == {object_id}
    branches = results["branches_by_group"][group_name]
    assert branches == {object_id: positions}


def test_stop_all_execution_stops_both_arms():
    commander = _make_commander(simulate=True)
    commander.stop_all_execution()

    for context in commander._manipulation_contexts.values():
        context._manipulator.stop_execution.assert_called_once_with()


def test_cancelled_task_guard_stops_both_and_awaits_experiment_cleanup():
    experiment_cleaned = asyncio.Event()

    async def experiment():
        try:
            await asyncio.Event().wait()
        finally:
            await asyncio.sleep(0)
            experiment_cleaned.set()

    async def safety_waiter():
        await asyncio.Event().wait()

    async def run():
        commander = SimpleNamespace(
            wait_for_rig_safety_fault=safety_waiter,
            stop_all_execution=MagicMock(),
        )
        task = asyncio.create_task(experiment())
        guard = asyncio.create_task(
            _wait_for_task_or_safety_fault(commander, task)
        )
        await asyncio.sleep(0)
        guard.cancel()
        with pytest.raises(asyncio.CancelledError):
            await guard
        assert task.cancelled()
        assert experiment_cleaned.is_set()
        commander.stop_all_execution.assert_called_once_with()


def test_cancelled_object_task_does_not_start_cleanup_motion():
    class CancelledTask(BaseObjectInteractionTask):
        async def run_trial(self, *args, **kwargs):
            raise AssertionError("run_trial should not be called")

    task = object.__new__(CancelledTask)
    task._run_trials_asynchronously = AsyncMock(
        side_effect=asyncio.CancelledError
    )
    task._occlude_and_lock = AsyncMock()
    task._commander = SimpleNamespace()
    task.log = MagicMock()

    with pytest.raises(asyncio.CancelledError):
        asyncio.run(task.run())

    task._occlude_and_lock.assert_not_awaited()
    task.log.assert_called_once()


def test_skip_idle_return_advances_state_without_planning_motion():
    manipulator = object.__new__(ObjectManipulationInterface)
    manipulator._manipulation_state = ManipulationState.POST_RETURN
    manipulator.log = MagicMock()
    manipulator.param = MagicMock(
        side_effect=lambda name: name == "skip_idle_on_return"
    )
    manipulator._get_state_goal = MagicMock()
    manipulator.plan_and_execute = AsyncMock()

    result = asyncio.run(
        manipulator._fetch_or_return_transition(
            "small_object_7", ManipulationState.IDLE
        )
    )

    assert result is None
    manipulator._get_state_goal.assert_not_called()
    manipulator.plan_and_execute.assert_not_awaited()


def test_scene_hash_changes_when_robot_link_padding_changes():
    moveit = object.__new__(MoveItInterface)
    scene_config = {"rig": {}, "grid_objects": {"object_kwargs": {}}}
    padding = {"left_eef_link": 0.01}

    def param(name):
        if name == "planning_scene":
            return scene_config
        if name == "link_padding":
            return padding
        raise AssertionError(f"Unexpected parameter: {name}")

    moveit.param = MagicMock(side_effect=param)
    first_hash = moveit.scene_hash(include_robot=False)
    padding["left_eef_link"] = 0.02
    second_hash = moveit.scene_hash(include_robot=False)
    assert first_hash != second_hash


def test_skip_idle_return_also_skips_explicit_reset_cleanup_move():
    manipulator = object.__new__(ObjectManipulationInterface)
    manipulator._manipulation_state = ManipulationState.IDLE
    manipulator.log = MagicMock()
    manipulator.param = MagicMock(
        side_effect=lambda name: name == "skip_idle_on_return"
    )
    manipulator._plan_and_move_impl = AsyncMock()

    asyncio.run(
        manipulator._reset_manipulation_impl(
            reset_to_idle=True, cache_trajectories=False
        )
    )
    manipulator._plan_and_move_impl.assert_not_awaited()


def test_preflight_branch_requires_three_consecutive_full_chain_replans(
    monkeypatch,
):
    validation = {"score": {"total_joint_travel": 1.0}}
    validate = AsyncMock(
        side_effect=[validation, RuntimeError("unstable branch")]
    )
    monkeypatch.setattr(preflight_module, "_validate_branch", validate)

    async def run():
        with pytest.raises(RuntimeError, match="unstable branch"):
            await preflight_module._validate_branch_repeated(
                MagicMock(), "big_object_7", MagicMock(spec=RobotState)
            )

    asyncio.run(run())
    assert validate.await_count == 2


def test_preflight_branch_passes_only_after_all_three_replans(monkeypatch):
    validation = {"score": {"total_joint_travel": 1.0}}
    validate = AsyncMock(return_value=validation)
    monkeypatch.setattr(preflight_module, "_validate_branch", validate)

    async def run():
        result = await preflight_module._validate_branch_repeated(
            MagicMock(), "small_object_0", MagicMock(spec=RobotState)
        )
        assert result is validation

    asyncio.run(run())
    assert validate.await_count == 3


def _make_laser_context(*, ready_side_effect=False):
    context = object.__new__(ManipulationContextManager)
    context._simulate = False
    context._laser_stop_requested = threading.Event()
    context._laser_state_lock = threading.Lock()
    context._laser_is_broken = None
    context._laser_break_generation = 0
    context._manipulator = SimpleNamespace(
        stop_execution=MagicMock(),
        manipulation_state=ManipulationState.PRESENTED,
        group_name="right_manipulator",
    )
    is_ready = (
        AsyncMock(side_effect=ready_side_effect)
        if isinstance(ready_side_effect, list)
        else AsyncMock(return_value=ready_side_effect)
    )
    context._ur = SimpleNamespace(
        stop_program=MagicMock(),
        is_ready=is_ready,
        reset=AsyncMock(),
    )
    context._teensy = SimpleNamespace(lock_arms_and_wait=AsyncMock())
    context._safe_to_execute_condition = MagicMock(return_value=True)
    context._raise_if_rig_safety_faulted = MagicMock()
    context._rig_execution_fault_handler = MagicMock()
    context.param = MagicMock(return_value=2)
    context.log = MagicMock()
    return context


def test_laser_stop_is_idempotent_and_restarts_real_controller():
    context = _make_laser_context(ready_side_effect=[False, True])

    assert context.request_laser_safety_stop() is True
    assert context.request_laser_safety_stop() is False
    context._ur.stop_program.assert_called_once_with()

    asyncio.run(context._recover_after_laser_stop())

    context._ur.reset.assert_awaited_once_with()
    assert not context._laser_stop_pending()


def test_failed_laser_controller_restart_keeps_recovery_pending():
    context = _make_laser_context(ready_side_effect=False)
    context._ur.reset.side_effect = RuntimeError("dashboard reset failed")
    context.request_laser_safety_stop()

    with pytest.raises(RuntimeError, match="dashboard reset failed"):
        asyncio.run(context._recover_after_laser_stop())

    assert context._laser_stop_pending()


def test_real_laser_interruption_recovers_controller_and_retries_motion():
    context = _make_laser_context(ready_side_effect=[False, True, True])
    attempts = 0
    error = ExecutionInterruptedError(
        "controller stopped for safety laser",
        group_name="left_manipulator",
    )

    @handle_interruptions
    async def operation(active_context):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            assert active_context.request_laser_safety_stop()
            raise error
        return "resumed"

    assert asyncio.run(operation(context)) == "resumed"
    assert attempts == 2
    context._ur.stop_program.assert_called_once_with()
    context._ur.reset.assert_awaited_once_with()
    context._rig_execution_fault_handler.assert_not_called()
    assert not context._laser_stop_pending()


def test_laser_generation_records_only_new_break_edges():
    context = _make_laser_context(ready_side_effect=True)

    assert context.observe_laser_state(False) == 0
    assert context.observe_laser_state(True) == 1
    assert context.observe_laser_state(True) == 1
    assert context.observe_laser_state(False) == 1
    assert context.observe_laser_state(True) == 2


def test_idle_context_observes_laser_edges_without_stopping_program():
    commander = object.__new__(Commander)
    contexts = {
        name: _make_laser_context(ready_side_effect=True)
        for name in ("left_manipulator", "right_manipulator")
    }
    for context in contexts.values():
        context._manipulator.executing = False
    commander._manipulation_contexts = contexts
    commander._teensy = SimpleNamespace(safe_to_execute=True)
    commander.log = MagicMock()
    msg = TeensySensor()

    for broken in (False, True, True, False, True):
        msg.is_safety_laser_broken = broken
        commander._teensy_sensor_callback(msg)

    for context in contexts.values():
        assert context._laser_break_generation_value() == 2
        context._ur.stop_program.assert_not_called()


def test_laser_rebreak_during_controller_reset_restarts_recovery():
    context = _make_laser_context(ready_side_effect=[False, True, False, True])
    context.observe_laser_state(False)
    context.observe_laser_state(True)
    assert context.request_laser_safety_stop()
    context.observe_laser_state(False)

    async def reset_with_one_rebreak():
        if context._ur.reset.await_count == 1:
            context.observe_laser_state(True)
            context.observe_laser_state(False)

    context._ur.reset.side_effect = reset_with_one_rebreak

    asyncio.run(context._recover_after_laser_stop())

    assert context._ur.reset.await_count == 2
    assert context._ur.stop_program.call_count == 2
    assert not context._laser_stop_pending()


def test_brief_laser_break_makes_not_safe_error_recoverable():
    context = _make_laser_context(ready_side_effect=True)
    context.observe_laser_state(False)
    attempts = 0
    error = NotSafeToExecuteError(
        "beam broke before motion started",
        group_name="right_manipulator",
    )

    @handle_interruptions
    async def operation(active_context):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            active_context.observe_laser_state(True)
            active_context.observe_laser_state(False)
            raise error
        return "resumed"

    assert asyncio.run(operation(context)) == "resumed"
    assert attempts == 2
    context._rig_execution_fault_handler.assert_not_called()


def test_rebreak_between_recovery_and_retry_waits_until_safe():
    context = _make_laser_context(ready_side_effect=[False, True, True])
    context.observe_laser_state(False)
    safe = {"value": True}
    context._safe_to_execute_condition = MagicMock(
        side_effect=lambda: safe["value"]
    )

    async def clear_laser(*, condition):
        context.observe_laser_state(False)
        safe["value"] = True
        assert condition()

    context._teensy.lock_arms_and_wait.side_effect = clear_laser

    def break_after_recovery(message, *args, **kwargs):
        if message.startswith("Reset successful, retrying"):
            context.observe_laser_state(True)
            safe["value"] = False

    context.log.side_effect = break_after_recovery
    attempts = 0
    error = ExecutionInterruptedError(
        "controller stopped for safety laser",
        group_name="right_manipulator",
    )

    @handle_interruptions
    async def operation(active_context):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            active_context.observe_laser_state(True)
            assert active_context.request_laser_safety_stop()
            active_context.observe_laser_state(False)
            raise error
        assert safe["value"]
        return "resumed"

    assert asyncio.run(operation(context)) == "resumed"
    assert attempts == 2
    context._teensy.lock_arms_and_wait.assert_awaited_once()
    context._rig_execution_fault_handler.assert_not_called()


def test_controller_recovery_cycles_are_bounded():
    context = _make_laser_context(ready_side_effect=False)
    error = ExecutionStoppedError(
        "laser recovery exhausted",
        group_name="right_manipulator",
    )
    fault = RigExecutionSafetyError("right_manipulator", error)
    context._rig_execution_fault_handler.return_value = fault
    context.request_laser_safety_stop()

    with pytest.raises(RigExecutionSafetyError) as exc_info:
        asyncio.run(context._recover_after_laser_stop())

    assert exc_info.value is fault
    assert context._ur.reset.await_count == 2
    assert context._laser_stop_pending()


def _make_fetch_recovery_manipulator():
    manipulator = object.__new__(ObjectManipulationInterface)
    manipulator._simulate = True
    manipulator._manipulation_state = ManipulationState.POST_FETCH
    manipulator._current_manipulation_id = "big_object_4"
    manipulator.log = MagicMock()
    manipulator._validate_target_object = MagicMock()
    manipulator._validate_manipulation_state = MagicMock()
    manipulator.param = MagicMock(
        side_effect=lambda name: name == "fetch_recovery.enable"
    )
    manipulator._move_to_fetch_recovery_waypoint = AsyncMock()
    return manipulator


def test_fetched_planning_failure_uses_one_stage_recovery_then_succeeds():
    manipulator = _make_fetch_recovery_manipulator()
    error = PlanningError("no path", "left_manipulator")
    manipulator._fetch_or_return_transition = AsyncMock(
        side_effect=[error, None]
    )

    asyncio.run(manipulator._fetch_object_impl("big_object_4"))

    assert manipulator._fetch_or_return_transition.await_count == 2
    manipulator._move_to_fetch_recovery_waypoint.assert_awaited_once_with(
        "big_object_4", ManipulationState.FETCHED
    )
    assert manipulator._manipulation_state == ManipulationState.FETCHED


def test_fetch_stage_recovery_is_bounded_to_one_retry():
    manipulator = _make_fetch_recovery_manipulator()
    first = PlanningError("first failure", "left_manipulator")
    second = PlanningError("retry failure", "left_manipulator")
    manipulator._fetch_or_return_transition = AsyncMock(
        side_effect=[first, second]
    )

    with pytest.raises(PlanningError) as exc_info:
        asyncio.run(manipulator._fetch_object_impl("big_object_4"))

    assert exc_info.value is second
    assert manipulator._fetch_or_return_transition.await_count == 2
    manipulator._move_to_fetch_recovery_waypoint.assert_awaited_once_with(
        "big_object_4", ManipulationState.FETCHED
    )


@pytest.mark.parametrize(
    ("failed_state", "expected_goal"),
    [
        (ManipulationState.PRE_FETCH, "idle"),
        (ManipulationState.FETCHED, "clearance-pose"),
    ],
)
def test_fetch_recovery_waypoint_uses_five_attempts_and_is_cached(
    failed_state, expected_goal
):
    manipulator = object.__new__(ObjectManipulationInterface)
    manipulator.log = MagicMock()
    manipulator.param = MagicMock(
        return_value={
            "max_attempts": 5,
            "pre_fetch_transit_goal": "idle",
            "fetched_clearance_offset": [0.22, 0.0, -0.18],
        }
    )
    manipulator._grid_object_pose_stamped_with_offset = MagicMock(
        return_value="clearance-pose"
    )
    manipulator.plan_and_execute = AsyncMock()

    asyncio.run(
        manipulator._move_to_fetch_recovery_waypoint(
            "small_object_28", failed_state
        )
    )

    request = manipulator.plan_and_execute.await_args.args[0]
    assert request.goal == expected_goal
    assert request.max_attempts == 5
    assert (
        manipulator.plan_and_execute.await_args.kwargs["cache_trajectories"]
        is True
    )
    if failed_state == ManipulationState.FETCHED:
        manipulator._grid_object_pose_stamped_with_offset.assert_called_once_with(
            "small_object_28", [0.22, 0.0, -0.18]
        )


class _MinimalObjectTask(BaseObjectInteractionTask):
    async def run_trial(self, *args, **kwargs):
        raise AssertionError("run_trial should be replaced by the test")


def _planning_trial_exit(message):
    return ManipulationContextExitedError(
        "recovered planning failure",
        recovered_error=PlanningError(message, "left_manipulator"),
    )


def test_trial_planning_failure_restarts_same_trial_immediately_once():
    task = object.__new__(_MinimalObjectTask)
    feedback = object()
    task._run_one_trial_attempt = AsyncMock(
        side_effect=[_planning_trial_exit("first"), feedback]
    )
    task._trial_lock = asyncio.Lock()
    task.log = MagicMock()
    spec = SimpleNamespace(
        trial_number=17,
        object_id="small_object_11",
    )

    assert asyncio.run(task._run_one_trial(spec)) is feedback
    assert task._run_one_trial_attempt.await_count == 2
    assert all(
        call.args == (spec,)
        for call in task._run_one_trial_attempt.await_args_list
    )


def test_trial_restart_is_bounded_and_does_not_blacklist_object():
    task = object.__new__(_MinimalObjectTask)
    task._run_one_trial_attempt = AsyncMock(
        side_effect=[
            _planning_trial_exit("first"),
            _planning_trial_exit("second"),
        ]
    )
    task._trial_lock = asyncio.Lock()
    task.log = MagicMock()
    spec = SimpleNamespace(
        trial_number=18,
        object_id="small_object_11",
    )

    assert asyncio.run(task._run_one_trial(spec)) is None
    assert task._run_one_trial_attempt.await_count == 2
    assert any(
        "without blacklisting" in call.args[0]
        for call in task.log.call_args_list
    )


def test_nonplanning_trial_recovery_is_not_retried():
    task = object.__new__(_MinimalObjectTask)
    error = ExecutionInterruptedError(
        "controller error", group_name="left_manipulator"
    )
    task._run_one_trial_attempt = AsyncMock(
        side_effect=ManipulationContextExitedError(
            "recovered execution failure", recovered_error=error
        )
    )
    task._trial_lock = asyncio.Lock()
    task.log = MagicMock()
    spec = SimpleNamespace(trial_number=19, object_id="small_object_11")

    assert asyncio.run(task._run_one_trial(spec)) is None
    task._run_one_trial_attempt.assert_awaited_once_with(spec)

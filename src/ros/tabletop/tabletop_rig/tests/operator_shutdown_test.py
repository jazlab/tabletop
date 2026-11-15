"""Dashboard/asyncio cancellation races, without ROS nodes or hardware goals."""

import asyncio
import functools
import threading
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest
import tabletop_rig.interfaces.moveit.plan_and_execute as plan_module
import tabletop_rig.nodes  # noqa: F401
from std_srvs.srv import Trigger
from tabletop_rig.exceptions import (
    ExecutionInterruptedError,
    ExecutionPreventedError,
    ExecutionStoppedError,
    NotSafeToExecuteError,
    RigExecutionSafetyError,
    URSafetyViolationError,
)
from tabletop_rig.interfaces.moveit.plan_and_execute import (
    PlanAndExecuteInterface,
)
from tabletop_rig.nodes.commander import (
    Commander,
    asyncio_runner,
    handle_interruptions,
)
from ur_dashboard_msgs.msg import SafetyMode


def rig():
    commander = object.__new__(Commander)
    commander._rig_safety_fault_lock = threading.Lock()
    commander._rig_safety_fault = None
    commander._rig_safety_monitor_armed = True
    commander._rig_safety_loop = None
    commander._rig_safety_event = None
    commander._operator_stop_lock = threading.Lock()
    commander._operator_stop_future = Future()
    commander._execution_shutdown_requested = threading.Event()
    commander._entered_context = True
    commander._exited_context = False
    commander.log = MagicMock()
    commander.param = MagicMock(return_value=False)
    commander._manipulation_contexts = {}
    for name in ("left_manipulator", "right_manipulator"):
        commander._manipulation_contexts[name] = SimpleNamespace(
            _execution_shutdown_requested=commander._execution_shutdown_requested,
            _simulate=False,
            _raise_if_rig_safety_faulted=commander.raise_if_rig_safety_faulted,
            _rig_execution_fault_handler=functools.partial(
                commander._handle_rig_execution_fault, name
            ),
            _safe_to_execute_condition=MagicMock(return_value=True),
            _recover_after_laser_stop=AsyncMock(),
            _laser_stop_pending=MagicMock(return_value=False),
            _laser_break_generation_value=MagicMock(return_value=0),
            _manipulator=SimpleNamespace(stop_execution=MagicMock()),
            _ur=SimpleNamespace(stop_program=MagicMock(), reset=AsyncMock()),
            param=MagicMock(return_value=3),
            log=MagicMock(),
        )
    return commander


@pytest.mark.parametrize("arm", ["left_manipulator", "right_manipulator"])
@pytest.mark.parametrize(
    "error_type",
    [ExecutionStoppedError, ExecutionPreventedError, NotSafeToExecuteError],
)
def test_stop_error_arriving_before_asyncio_cancel_is_graceful(
    arm, error_type
):
    commander = rig()
    context = commander._manipulation_contexts[arm]
    attempts = []

    @handle_interruptions
    async def operation(active_context):
        attempts.append(arm)
        response = commander._operator_stop_callback(
            Trigger.Request(), Trigger.Response()
        )
        assert response.success
        raise error_type("stopped by operator", group_name=arm)

    with pytest.raises(asyncio.CancelledError):
        asyncio.run(operation(context))

    assert attempts == [arm]
    assert not commander.rig_safety_faulted
    assert commander._operator_stop_future.result() == "operator stop"
    for ctx in commander._manipulation_contexts.values():
        ctx._manipulator.stop_execution.assert_called_once_with()
        ctx._ur.stop_program.assert_not_called()
        ctx._ur.reset.assert_not_awaited()


def test_shutdown_refuses_new_operation_without_recovery_or_motion():
    commander = rig()
    commander.request_execution_shutdown()
    context = commander._manipulation_contexts["right_manipulator"]
    called = []

    @handle_interruptions
    async def operation(active_context):
        called.append(True)

    with pytest.raises(asyncio.CancelledError):
        asyncio.run(operation(context))
    assert not called
    context._recover_after_laser_stop.assert_not_awaited()


@pytest.mark.parametrize("intentional", [False, True])
def test_unrelated_controller_failure_still_latches_fatal_fault(intentional):
    commander = rig()
    context = commander._manipulation_contexts["right_manipulator"]

    @handle_interruptions
    async def operation(active_context):
        if intentional:
            commander.request_execution_shutdown()
        raise ExecutionInterruptedError(
            "path tolerance violated", group_name="right_manipulator"
        )

    with pytest.raises(RigExecutionSafetyError):
        asyncio.run(operation(context))
    assert commander.rig_safety_faulted


def test_execution_stop_without_shutdown_intent_is_still_fatal():
    commander = rig()

    @handle_interruptions
    async def operation(context):
        raise ExecutionStoppedError(
            "unexpected stop", group_name="left_manipulator"
        )

    with pytest.raises(RigExecutionSafetyError):
        asyncio.run(
            operation(commander._manipulation_contexts["left_manipulator"])
        )
    assert not commander._execution_shutdown_requested.is_set()


def test_real_ur_protective_stop_has_priority_over_intentional_cancellation():
    commander = rig()

    @handle_interruptions
    async def operation(context):
        commander.request_execution_shutdown()
        commander._ur_safety_mode_callback(
            "right_manipulator", SafetyMode(mode=SafetyMode.PROTECTIVE_STOP)
        )
        raise ExecutionStoppedError(
            "operator stop", group_name="right_manipulator"
        )

    with pytest.raises(URSafetyViolationError):
        asyncio.run(
            operation(commander._manipulation_contexts["right_manipulator"])
        )


def test_newly_planned_goal_cannot_be_submitted_after_stop():
    interface = object.__new__(PlanAndExecuteInterface)
    interface._execution_shutdown_requested = threading.Event()
    interface._execution_shutdown_requested.set()
    interface._execution_client = SimpleNamespace(send_goal_async=AsyncMock())
    trajectory = SimpleNamespace(joint_model_group_name="right_manipulator")
    with pytest.raises(ExecutionStoppedError, match="Intentional"):
        asyncio.run(interface._send_trajectory(trajectory))
    interface._execution_client.send_goal_async.assert_not_awaited()


@pytest.mark.parametrize("iteration", range(20))
@pytest.mark.parametrize("arm", ["left_manipulator", "right_manipulator"])
@pytest.mark.parametrize("fault_during_cleanup", [False, True])
def test_runner_exits_normally_for_dashboard_race_and_drains_task_cleanup(
    iteration, arm, fault_during_cleanup
):
    commander = rig()
    cleanup = []

    class Scope:
        async def __aenter__(self):
            return commander

        async def __aexit__(self, *args):
            cleanup.append("commander")
            if fault_during_cleanup:
                commander._ur_safety_mode_callback(
                    arm, SafetyMode(mode=SafetyMode.PROTECTIVE_STOP)
                )

        def raise_if_rig_safety_faulted(self):
            commander.raise_if_rig_safety_faulted()

        def request_execution_shutdown(self):
            commander.request_execution_shutdown()

        async def wait_for_rig_safety_fault(self):
            await asyncio.Event().wait()

    @handle_interruptions
    async def operation(context):
        commander._operator_stop_callback(
            Trigger.Request(), Trigger.Response()
        )
        raise ExecutionStoppedError("operator stop", group_name=arm)

    async def task(active_commander, config):
        try:
            await operation(commander._manipulation_contexts[arm])
        finally:
            cleanup.append("task")

    coroutine = asyncio_runner(
        Scope(), task, None, commander._operator_stop_future, 1
    )
    if fault_during_cleanup:
        with pytest.raises(URSafetyViolationError):
            asyncio.run(coroutine)
    else:
        asyncio.run(coroutine)
    assert cleanup == ["task", "commander"]
    assert commander.rig_safety_faulted == fault_during_cleanup


def test_stop_while_preparing_message_is_checked_again_before_submission(
    monkeypatch,
):
    interface = object.__new__(PlanAndExecuteInterface)
    flag = threading.Event()
    interface._execution_shutdown_requested = flag
    interface._execution_client = SimpleNamespace(send_goal_async=AsyncMock())
    interface.param = lambda name: 1.0
    interface._safe_to_execute_condition = lambda: True
    interface._moveit = SimpleNamespace(get_current_state=lambda: None)
    interface._check_right_base_velocity = MagicMock()
    monkeypatch.setattr(
        plan_module, "all_close_robot_states", lambda *a, **k: True
    )

    class Trajectory:
        joint_model_group_name = "right_manipulator"
        duration = 1.0

        def __getitem__(self, index):
            return None

        def get_robot_trajectory_msg(self):
            flag.set()
            return SimpleNamespace(joint_trajectory=object())

    with pytest.raises(ExecutionStoppedError, match="Intentional"):
        asyncio.run(interface._send_trajectory(Trajectory()))
    interface._execution_client.send_goal_async.assert_not_awaited()

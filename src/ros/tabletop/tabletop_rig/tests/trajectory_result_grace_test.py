"""Regression tests for delayed FollowJointTrajectory result delivery."""
import asyncio
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

import tabletop_rig.interfaces.moveit.plan_and_execute as plan_module
from tabletop_rig.exceptions import ExecutionInterruptedError
from tabletop_rig.interfaces.moveit.plan_and_execute import (
    PlanAndExecuteInterface,
)


class _Trajectory:
    duration = 0.01
    joint_model_group_name = "left_manipulator"

    def __len__(self):
        return 2

    def __getitem__(self, index):
        return f"state-{index}"

    def get_robot_trajectory_msg(self):
        return SimpleNamespace(joint_trajectory=JointTrajectory())


class _DelayedResultClient:
    def __init__(self, *, status: int, delay: float):
        self.goal_handle = SimpleNamespace(status=status)
        self.delay = delay
        self.result_cancelled = False

    async def send_goal_async(self, goal):
        return self.goal_handle

    async def get_result_async(self, goal_handle):
        assert goal_handle is self.goal_handle
        try:
            await asyncio.sleep(self.delay)
        except asyncio.CancelledError:
            self.result_cancelled = True
            raise
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def _make_interface(client, *, grace: float):
    interface = object.__new__(PlanAndExecuteInterface)
    params = {
        "execution.allowed_start_tolerance": 0.02,
        "execution.allowed_end_tolerance": 0.01,
        "execution.allowed_duration_scaling": 1.0,
        "execution.allowed_duration_margin": 0.0,
        "execution.result_delivery_grace": grace,
        "execution.result_recovery_velocity_tolerance": 0.01,
    }
    interface.param = params.__getitem__
    interface.log = MagicMock()
    interface._safe_to_execute_condition = lambda: True
    interface._moveit = SimpleNamespace(get_current_state=lambda: "current")
    interface._execution_client = client
    interface._goal_handle_lock = threading.Lock()
    interface._execution_goal_handle = None
    return interface


def test_success_status_gets_result_delivery_grace(monkeypatch):
    monkeypatch.setattr(plan_module, "all_close_robot_states", lambda *a, **k: True)
    client = _DelayedResultClient(
        status=GoalStatus.STATUS_SUCCEEDED,
        delay=0.04,
    )
    interface = _make_interface(client, grace=0.1)

    asyncio.run(interface._send_trajectory(_Trajectory()))

    assert not client.result_cancelled
    assert interface._execution_goal_handle is None
    assert any(
        "result delivery" in call.args[0]
        for call in interface.log.call_args_list
    )


def test_active_goal_still_fails_at_execution_deadline(monkeypatch):
    monkeypatch.setattr(plan_module, "all_close_robot_states", lambda *a, **k: True)
    client = _DelayedResultClient(
        status=GoalStatus.STATUS_EXECUTING,
        delay=1.0,
    )
    interface = _make_interface(client, grace=0.1)

    with pytest.raises(ExecutionInterruptedError, match="Allowed execution duration"):
        asyncio.run(interface._send_trajectory(_Trajectory()))

    assert client.result_cancelled
    interface.log.assert_not_called()


def test_success_status_recovers_if_result_grace_expires_at_stopped_endpoint(
    monkeypatch,
):
    close_states = MagicMock(return_value=True)
    monkeypatch.setattr(plan_module, "all_close_robot_states", close_states)
    client = _DelayedResultClient(
        status=GoalStatus.STATUS_SUCCEEDED,
        delay=1.0,
    )
    interface = _make_interface(client, grace=0.02)

    asyncio.run(interface._send_trajectory(_Trajectory()))

    assert client.result_cancelled
    assert close_states.call_args_list[1].kwargs["position_tolerance"] == 0.01
    assert close_states.call_args_list[1].kwargs["velocity_tolerance"] == 0.01
    assert any(
        "accepting the completed trajectory" in call.args[0]
        for call in interface.log.call_args_list
    )


def test_success_status_missing_result_fails_if_endpoint_is_not_stationary(
    monkeypatch,
):
    close_states = MagicMock(side_effect=[True, False])
    monkeypatch.setattr(plan_module, "all_close_robot_states", close_states)
    client = _DelayedResultClient(
        status=GoalStatus.STATUS_SUCCEEDED,
        delay=1.0,
    )
    interface = _make_interface(client, grace=0.02)

    with pytest.raises(
        ExecutionInterruptedError,
        match="endpoint/stationary verification failed",
    ):
        asyncio.run(interface._send_trajectory(_Trajectory()))

    assert client.result_cancelled

def test_success_status_missing_result_fails_if_rig_is_not_safe(monkeypatch):
    monkeypatch.setattr(
        plan_module, "all_close_robot_states", lambda *a, **k: True
    )
    client = _DelayedResultClient(
        status=GoalStatus.STATUS_SUCCEEDED,
        delay=1.0,
    )
    interface = _make_interface(client, grace=0.02)
    safety_states = iter([True, False])
    interface._safe_to_execute_condition = lambda: next(safety_states)

    with pytest.raises(
        ExecutionInterruptedError,
        match="safety condition no longer permits recovery",
    ):
        asyncio.run(interface._send_trajectory(_Trajectory()))

    assert client.result_cancelled

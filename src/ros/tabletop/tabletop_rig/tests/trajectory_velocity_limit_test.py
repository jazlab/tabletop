"""Checks for the right-base limit, including direct execution bypasses."""

import asyncio
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest
import yaml
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tabletop_rig.interfaces.moveit.plan_and_execute as module
from tabletop_rig.exceptions import ExecutionPreventedError, TrajectoryError
from tabletop_rig.interfaces.moveit.plan_and_execute import (
    PlanAndExecuteInterface,
    validate_joint_velocity_limit,
)

JOINT = "right_shoulder_pan_joint"


def test_default_big_object_6_has_no_stacked_request_scaling():
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package / "config/commander.yaml").read_text())
    params = params["/commander"]["ros__parameters"]
    for direction in ("present_overrides", "unpresent_overrides"):
        for section in ("common_manipulation_interface", "right_manipulation_interface"):
            assert "big_object_6" not in (params[section].get(direction) or {})
    limits = package.parent / "tabletop_moveit_config/config/joint_limits.yaml"
    bounds = yaml.safe_load(limits.read_text())["joint_limits"][JOINT]
    assert bounds["max_velocity"] == 2.4
    assert bounds["max_acceleration"] == 14.0


def message(speed=2.4, distance=1.0, joint=JOINT):
    return JointTrajectory(joint_names=[joint], points=[
        JointTrajectoryPoint(positions=[0.0], velocities=[0.0]),
        JointTrajectoryPoint(positions=[distance], velocities=[speed],
                             time_from_start=Duration(sec=1)),
    ])


@pytest.mark.parametrize("speed", [-2.4, 0.0, 2.4])
def test_accepts_both_velocity_directions(speed):
    validate_joint_velocity_limit(message(speed), JOINT, 2.4)


@pytest.mark.parametrize("speed", [-3.14, 2.42, float("nan"), float("inf")])
def test_rejects_excessive_or_nonfinite_velocity(speed):
    with pytest.raises(ValueError):
        validate_joint_velocity_limit(message(speed), JOINT, 2.4)


def test_cannot_hide_fast_segment_behind_zero_reported_velocity():
    with pytest.raises(ValueError, match="segment speed"):
        validate_joint_velocity_limit(message(0.0, distance=3.0), JOINT, 2.4)


def test_left_joint_speed_is_unchanged():
    validate_joint_velocity_limit(message(3.14, 3.0, "left_shoulder_pan_joint"), JOINT, 2.4)


def test_rejects_missing_derivatives_or_invalid_timing():
    msg = message()
    msg.points[-1].velocities = []
    with pytest.raises(ValueError, match="Missing"):
        validate_joint_velocity_limit(msg, JOINT, 2.4)
    msg = message()
    msg.points[-1].time_from_start = Duration()
    with pytest.raises(ValueError, match="Non-increasing"):
        validate_joint_velocity_limit(msg, JOINT, 2.4)


def interface():
    result = object.__new__(PlanAndExecuteInterface)
    group = SimpleNamespace(active_joint_model_names=[JOINT],
        active_joint_model_bounds=[[SimpleNamespace(velocity_bounded=True, max_velocity=2.4)]])
    result._moveit = SimpleNamespace(
        robot_model=SimpleNamespace(get_joint_model_group=lambda name: group),
        get_current_state=lambda: None,
        is_path_valid=MagicMock(return_value=True),
    )
    result.log = MagicMock()
    return result


def test_model_bound_used_by_planning_validation():
    obj = interface()
    trajectory = SimpleNamespace(joint_model_group_name="right_manipulator",
        get_robot_trajectory_msg=lambda: SimpleNamespace(joint_trajectory=message(3.14)))
    with pytest.raises(TrajectoryError):
        obj._validate_trajectory(trajectory)
    obj._moveit.is_path_valid.assert_not_called()


def test_direct_execute_refuses_fast_trajectory_before_sending(monkeypatch):
    obj = interface()
    obj.param = lambda name: 1.0
    obj._safe_to_execute_condition = lambda: True
    obj._execution_client = SimpleNamespace(send_goal_async=AsyncMock())
    monkeypatch.setattr(module, "all_close_robot_states", lambda *a, **k: True)

    class Trajectory:
        joint_model_group_name = "right_manipulator"
        duration = 1.0

        def __getitem__(self, index):
            return None

        def get_robot_trajectory_msg(self):
            return SimpleNamespace(joint_trajectory=message(3.14))

    with pytest.raises(ExecutionPreventedError, match="exceeds"):
        asyncio.run(obj._send_trajectory(Trajectory()))
    obj._execution_client.send_goal_async.assert_not_awaited()

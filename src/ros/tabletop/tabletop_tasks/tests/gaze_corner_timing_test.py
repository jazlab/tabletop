"""Regression checks for the opt-in sinusoid timing."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from gaze_depth_sweep_test import kwargs
from moveit_msgs.msg import RobotTrajectory
from scipy.integrate import cumulative_trapezoid
from tabletop_tasks.tasks import smooth_pursuit
from tabletop_tasks.tasks.gaze_corner_timing import pursuit_turn_clock
from trajectory_msgs.msg import JointTrajectoryPoint


def test_straight_motion_keeps_original_times():
    t = np.linspace(0.0, 8.0, 161)
    xyz = np.column_stack([t * 0.05, t * 0, t * 0])
    new, rate, derivative = pursuit_turn_clock(t, xyz, 0.80)
    np.testing.assert_allclose(new, t)
    np.testing.assert_allclose(rate, 1.0)
    np.testing.assert_allclose(derivative, 0.0)


def test_turn_eases_in_and_out_without_stopping_or_speeding_up():
    t = np.linspace(0.0, 12.0, 241)
    heading = (np.pi / 2.0) * (1.0 + np.tanh((t - 6.0) / 0.34))
    v = 0.07 * np.column_stack([np.cos(heading), np.sin(heading), t * 0])
    xyz = cumulative_trapezoid(v, t, axis=0, initial=0.0)
    new, rate, derivative = pursuit_turn_clock(t, xyz, 0.80)
    assert np.all(np.diff(new) >= np.diff(t) - 1e-12)
    assert np.all(np.diff(new) <= 1.25 * np.diff(t) + 1e-12)
    assert new[-1] > t[-1] + 0.10
    assert rate[0] == pytest.approx(1.0)
    assert rate[-1] == pytest.approx(1.0)
    assert rate.max() > 1.10
    assert np.max(np.abs(derivative)) < 0.5
    assert (
        np.linalg.norm(v / rate[:, None], axis=1).min() >= 0.07 * 0.80 - 1e-12
    )


def test_reject_duplicate_times():
    with pytest.raises(ValueError, match="Invalid trajectory"):
        pursuit_turn_clock(np.array([0.0, 0.0, 1.0]), np.zeros((3, 3)), 0.80)


def test_preset_only_changes_pursuit_timing_and_planning_budget():
    original = kwargs("smooth_pursuit_sin_static_forward_five")
    eased = kwargs("smooth_pursuit_sin_static_forward_five_gentle_turns")
    assert eased.pop("corner_speed_fraction") == 0.80
    assert eased.pop("max_motion_generation_attempts") == 40
    assert original.pop("max_motion_generation_attempts") == 20
    assert eased == original


def test_retimed_message_preserves_waypoints_and_checks_limits(monkeypatch):
    config = kwargs("smooth_pursuit_sin_static_forward_five_gentle_turns")
    moveit = MagicMock()
    task = smooth_pursuit.SmoothPursuitTask(
        SimpleNamespace(get_logger=MagicMock(), _moveit=moveit), **config
    )
    task.log = MagicMock()
    t = np.linspace(0.0, 8.0, 161)
    xyz = np.column_stack(
        [0.04 * np.sin(2.7 * t), 0.04 * np.cos(2.7 * t), 0.01 * t]
    )
    bound = SimpleNamespace(
        velocity_bounded=True,
        max_velocity=2.0,
        acceleration_bounded=True,
        max_acceleration=5.0,
    )
    group = SimpleNamespace(
        active_joint_model_names=["test_joint"],
        active_joint_model_bounds=[[bound]],
    )
    model = SimpleNamespace(get_joint_model_group=lambda _: group)
    source = RobotTrajectory()
    source.joint_trajectory.joint_names = ["test_joint"]
    for s in t:
        point = JointTrajectoryPoint(
            positions=[0.2 * s], velocities=[0.2], accelerations=[0.0]
        )
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(
            round(s * 1e9), 10**9
        )
        source.joint_trajectory.points.append(point)

    class FakeTrajectory:
        joint_model_group_name = "right_manipulator"

        def __init__(self, _model, message=None):
            self.message = message

        def __getitem__(self, i):
            p = SimpleNamespace(x=xyz[i, 0], y=xyz[i, 1], z=xyz[i, 2])
            return SimpleNamespace(
                robot_model=model,
                get_pose=lambda _: SimpleNamespace(position=p),
            )

        def __len__(self):
            return len(self.message.joint_trajectory.points)

        def get_robot_trajectory_msg(self):
            return deepcopy(self.message)

        def set_robot_trajectory_msg(self, _seed, message):
            self.message = message
            last = message.joint_trajectory.points[-1].time_from_start
            self.duration = last.sec + last.nanosec * 1e-9

    monkeypatch.setattr(smooth_pursuit, "RobotTrajectory", FakeTrajectory)
    result = task._retime_pursuit(FakeTrajectory(model, source))
    moveit.is_path_valid.assert_called_once_with(result, verbose=False)
    a = source.joint_trajectory.points
    b = result.message.joint_trajectory.points
    assert [p.positions for p in a] == [p.positions for p in b]
    assert result.duration > t[-1]
    assert max(p.velocities[0] for p in b) <= 0.2
    assert min(p.velocities[0] for p in b) >= 0.16 - 1e-12

    bound.max_velocity = 0.15
    with pytest.raises(ValueError, match="velocity limit"):
        task._retime_pursuit(FakeTrajectory(model, source))

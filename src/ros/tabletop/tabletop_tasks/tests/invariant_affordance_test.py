"""Tests for the invariant-affordance task timing and path helpers."""

import asyncio
from pathlib import Path

import numpy as np
import pytest
import yaml
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from moveit_msgs.msg import RobotTrajectory
from tabletop_interfaces.msg import TaskEvent
from tabletop_rig.utils.ros import arrays_from_pose_msg, pose_stamped_msg
from tabletop_tasks.tasks.invariant_affordance import (
    InvariantAffordanceTask,
    generate_bounded_bezier_poses,
    generate_distant_target_pose,
    generate_uniform_target_pose,
    stretch_trajectory_message,
    validate_trajectory_arm_half,
)
from tabletop_tasks.trial_generators.base import TrialSpec
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    MultiDOFJointTrajectoryPoint,
)


def test_generated_paths_stay_inside_pose_envelope() -> None:
    center = pose_stamped_msg(
        frame_id="world",
        position=[0.9525, 0.4, 0.6],
        rpy=[0.0, 0.0, -1.5707],
    )
    center_position, center_rpy = arrays_from_pose_msg(center.pose, euler=True)
    position_bounds = np.array([0.08, 0.04, 0.06])
    orientation_bounds = np.array([0.12, 0.12, 0.18])

    orientation_changes = []
    for seed in range(100):
        poses = generate_bounded_bezier_poses(
            center,
            rng=np.random.default_rng(seed),
            position_half_ranges=position_bounds,
            orientation_half_ranges=orientation_bounds,
            num_waypoints=8,
        )

        assert len(poses) == 8
        for pose in poses:
            position, rpy = arrays_from_pose_msg(pose.pose, euler=True)
            assert np.all(
                np.abs(position - center_position) <= position_bounds + 1e-12
            )
            assert np.all(
                np.abs(rpy - center_rpy) <= orientation_bounds + 1e-12
            )
            assert pose.header.frame_id == "world"
        orientation_changes.append(
            np.linalg.norm(
                arrays_from_pose_msg(poses[-1].pose, euler=True)[1]
                - center_rpy
            )
        )

    assert max(orientation_changes) > 0.05


def test_distant_target_is_bounded_and_substantially_changed() -> None:
    center = pose_stamped_msg(
        frame_id="world",
        position=[0.9525, 0.4, 0.6],
        rpy=[0.0, 0.0, -1.5707],
    )
    center_position, center_rpy = arrays_from_pose_msg(center.pose, euler=True)
    position_bounds = np.array([0.14, 0.08, 0.10])
    orientation_bounds = np.array([0.25, 0.22, 0.40])

    for seed in range(100):
        target = generate_distant_target_pose(
            center,
            rng=np.random.default_rng(seed),
            position_half_ranges=position_bounds,
            orientation_half_ranges=orientation_bounds,
            minimum_position_delta=0.12,
            minimum_orientation_delta=0.30,
        )
        position, rpy = arrays_from_pose_msg(target.pose, euler=True)
        position_delta = position - center_position
        orientation_delta = rpy - center_rpy

        assert np.all(np.abs(position_delta) <= position_bounds + 1e-12)
        assert np.all(np.abs(orientation_delta) <= orientation_bounds + 1e-12)
        assert np.linalg.norm(position_delta) >= 0.12 - 1e-12
        assert np.linalg.norm(orientation_delta) >= 0.30 - 1e-12
        assert target.header.frame_id == "world"


def test_uniform_target_samples_box_and_full_rotation_space() -> None:
    center = pose_stamped_msg(
        frame_id="world",
        position=[0.9525, 0.4, 0.6],
        rpy=[0.0, 0.0, -1.5707],
    )
    bounds = np.array([0.10, 0.06, 0.08])
    rng = np.random.default_rng(20260911)
    positions = []
    quaternions = []

    for _ in range(10_000):
        target = generate_uniform_target_pose(
            center,
            rng=rng,
            position_half_ranges=bounds,
        )
        positions.append(
            [
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
            ]
        )
        quaternions.append(
            [
                target.pose.orientation.w,
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
            ]
        )

    center_position, _ = arrays_from_pose_msg(center.pose, euler=True)
    position_offsets = np.asarray(positions) - center_position
    quaternion_array = np.asarray(quaternions)
    assert np.all(np.abs(position_offsets) <= bounds + 1e-12)
    assert np.allclose(position_offsets.mean(axis=0), 0.0, atol=0.002)
    assert np.allclose(
        np.linalg.norm(quaternion_array, axis=1), 1.0, atol=1e-12
    )
    # Every unit-quaternion component has E[q_i^2] = 1/4 under Haar measure.
    assert np.allclose(np.mean(quaternion_array**2, axis=0), 0.25, atol=0.01)


def test_uniform_target_respects_equal_area_tilt_stratum() -> None:
    center = pose_stamped_msg(position=[0.9525, 0.4, 0.6])
    rng = np.random.default_rng(99)
    for lower, upper in [(-1.0, -0.5), (-0.1, 0.2), (0.5, 1.0)]:
        for _ in range(200):
            target = generate_uniform_target_pose(
                center,
                rng=rng,
                position_half_ranges=[0.1, 0.1, 0.1],
                local_up_cosine_range=(lower, upper),
            )
            orientation = target.pose.orientation
            local_up_cosine = 1.0 - 2.0 * (orientation.x**2 + orientation.y**2)
            assert lower <= local_up_cosine <= upper


def test_orientation_strata_cover_full_space_and_wait_for_acceptance() -> None:
    task = object.__new__(InvariantAffordanceTask)
    task.orientation_strata = 3
    task._rng = np.random.default_rng(7)
    task._orientation_strata_queues = {}
    ranges = []

    for _ in range(3):
        current = task._next_orientation_cosine_range("right_manipulator")
        assert (
            task._next_orientation_cosine_range("right_manipulator") == current
        )
        ranges.append(current)
        task._orientation_strata_queues["right_manipulator"].pop()

    expected_edges = np.linspace(-1.0, 1.0, 4)
    assert sorted(ranges) == [
        (float(expected_edges[index]), float(expected_edges[index + 1]))
        for index in range(3)
    ]


def test_trajectory_arm_half_checks_every_waypoint() -> None:
    center = pose_stamped_msg(
        frame_id="world",
        position=[0.9525, 0.4, 0.6],
        rpy=[0.0, 0.0, -1.5707],
    )

    class State:
        def __init__(self, position) -> None:
            self.position = position

        def get_pose(self, link_name: str):
            assert link_name in {"left_eef", "right_eef"}
            return pose_stamped_msg(position=self.position).pose

    validate_trajectory_arm_half(
        [State([0.9525, 0.4, 0.6]), State([1.6219, 0.53, 0.73])],
        center,
        group_name="right_manipulator",
        centerline_overlap=0.12,
    )
    validate_trajectory_arm_half(
        [State([0.9525, 0.4, 0.6]), State([0.2744, 0.2, 1.1])],
        center,
        group_name="left_manipulator",
        centerline_overlap=0.12,
    )

    with pytest.raises(ValueError, match="waypoint 1 crosses"):
        validate_trajectory_arm_half(
            [State([0.9525, 0.4, 0.6]), State([0.82, 0.4, 0.6])],
            center,
            group_name="right_manipulator",
            centerline_overlap=0.12,
        )


def test_trajectory_scaling_only_slows_timing_and_derivatives() -> None:
    message = RobotTrajectory()
    point = JointTrajectoryPoint()
    point.positions = [0.1, 0.2]
    point.velocities = [2.0, -4.0]
    point.accelerations = [8.0, -12.0]
    point.time_from_start = Duration(sec=1, nanosec=500_000_000)
    message.joint_trajectory.points = [point]

    multi_dof_point = MultiDOFJointTrajectoryPoint()
    message.multi_dof_joint_trajectory.points = [multi_dof_point]
    multi_dof_point.time_from_start = Duration(sec=1)
    velocity = Twist()
    velocity.linear.x = 4.0
    velocity.angular.z = -2.0
    acceleration = Twist()
    acceleration.linear.y = 8.0
    multi_dof_point.velocities = [velocity]
    multi_dof_point.accelerations = [acceleration]

    result = stretch_trajectory_message(message, 2.0)

    assert result is not message
    assert list(result.joint_trajectory.points[0].positions) == [0.1, 0.2]
    assert list(result.joint_trajectory.points[0].velocities) == [1.0, -2.0]
    assert list(result.joint_trajectory.points[0].accelerations) == [2.0, -3.0]
    assert result.joint_trajectory.points[0].time_from_start == Duration(sec=3)
    assert result.multi_dof_joint_trajectory.points[
        0
    ].time_from_start == Duration(sec=2)
    assert (
        result.multi_dof_joint_trajectory.points[0].velocities[0].linear.x
        == 2.0
    )
    assert (
        result.multi_dof_joint_trajectory.points[0].velocities[0].angular.z
        == -1.0
    )
    assert (
        result.multi_dof_joint_trajectory.points[0].accelerations[0].linear.y
        == 2.0
    )

    # Input must remain intact.
    assert list(message.joint_trajectory.points[0].velocities) == [2.0, -4.0]
    with pytest.raises(ValueError, match="at least 1"):
        stretch_trajectory_message(message, 0.5)


def test_trial_phase_order_and_cues(monkeypatch) -> None:
    calls: list[object] = []
    clock = [100.0]

    async def fake_sleep(duration: float) -> None:
        calls.append(("sleep", duration))
        clock[0] += duration

    monkeypatch.setattr(
        "tabletop_tasks.tasks.invariant_affordance.asyncio.sleep", fake_sleep
    )
    monkeypatch.setattr(
        "tabletop_tasks.tasks.invariant_affordance.time.monotonic",
        lambda: clock[0],
    )

    class FakeCommander:
        async def reveal_smartglass(self) -> None:
            calls.append("reveal")

        async def occlude_smartglass(self) -> None:
            calls.append("occlude")

        async def play_sound(self, **kwargs) -> None:
            calls.append(("sound", kwargs))

        async def pulse_buzzer(self, arm: str) -> None:
            calls.append(("buzzer", arm))

        def publish_task_event(self, event_type: int, **kwargs) -> None:
            calls.append(("event", event_type, kwargs))

    class FakeManipulator:
        async def move(self, trajectory) -> None:
            calls.append(("move", trajectory))

    task = object.__new__(InvariantAffordanceTask)
    task._commander = FakeCommander()
    task.sound_kwargs = {"duration": 0.25}
    task.stimulus_duration = 1.0
    task.occlusion_settle_duration = 0.5
    task.motion_duration = 3.0
    task.post_motion_hold_duration = 0.25
    task.response_duration = 2.0
    task.log = lambda *_args, **_kwargs: None

    trajectory = object()
    return_trajectory = object()

    async def fake_plan(*_args, **_kwargs):
        calls.append("plan")
        return trajectory, return_trajectory

    task._plan_occluded_reposition = fake_plan
    spec = TrialSpec(
        trial_number=7,
        object_id="big_object_6",
        group_name="right_manipulator",
        object_pose=pose_stamped_msg(position=[0.9525, 0.4, 0.6]),
    )

    feedback = asyncio.run(task.run_trial(spec, FakeManipulator()))

    assert feedback.reaction_time is None
    assert feedback.timeout is None
    assert calls[:8] == [
        "plan",
        "reveal",
        (
            "event",
            TaskEvent.STIMULUS_REVEALED,
            {
                "trial_number": 7,
                "group_name": "right_manipulator",
                "object_id": "big_object_6",
            },
        ),
        ("sleep", 1.0),
        "occlude",
        ("sleep", 0.5),
        (
            "event",
            TaskEvent.MOTION_STARTED,
            {
                "trial_number": 7,
                "group_name": "right_manipulator",
                "object_id": "big_object_6",
            },
        ),
        ("move", trajectory),
    ]
    go_index = next(
        index
        for index, call in enumerate(calls)
        if isinstance(call, tuple)
        and len(call) > 1
        and call[:2] == ("event", TaskEvent.GO_CUE)
    )
    assert ("sound", {"duration": 0.25}) in calls[go_index + 1 :]
    assert ("buzzer", "both") in calls[go_index + 1 :]
    assert calls[go_index - 1] == ("sleep", 2.5)
    assert ("sleep", 2.0) in calls[go_index + 1 :]
    assert calls[-1] == ("move", return_trajectory)


def test_occluded_reposition_plans_bounded_complex_move() -> None:
    calls: list[dict] = []

    class FakeTrajectory:
        duration = 0.8

        def __init__(self, final_state: object, position) -> None:
            self.final_state = final_state
            self.position = position

        def __getitem__(self, index: int) -> object:
            assert index == 0
            return self

        def __len__(self) -> int:
            return 1

        def get_pose(self, link_name: str):
            assert link_name == "right_eef"
            return pose_stamped_msg(position=self.position).pose

    class FakeManipulator:
        async def plan(self, **kwargs):
            calls.append(kwargs)
            if len(calls) == 1:
                return outbound_trajectory
            return return_trajectory

    task = object.__new__(InvariantAffordanceTask)
    task.position_half_ranges = np.array([0.10, 0.06, 0.08])
    task.path_centerline_overlap = 0.12
    task.orientation_strata = 3
    task.motion_planning_pipeline = "aps_rrtc"
    task.motion_planning_time = 0.25
    task.motion_velocity_scaling_factor = 0.75
    task.motion_acceleration_scaling_factor = 1.0
    task.motion_duration = 3.0
    task.max_trajectory_duration = 2.25
    task.max_motion_generation_attempts = 5
    task._rng = np.random.default_rng(42)
    task._orientation_strata_queues = {}
    task.log = lambda *_args, **_kwargs: None
    center = pose_stamped_msg(
        frame_id="world",
        position=[0.9525, 0.4, 0.6],
        rpy=[0.0, 0.0, -1.5707],
    )
    spec = TrialSpec(
        trial_number=1,
        object_id="big_object_6",
        group_name="right_manipulator",
        object_pose=center,
    )

    outbound_end_state = object()
    outbound_trajectory = FakeTrajectory(
        outbound_end_state, [0.9525, 0.4, 0.6]
    )
    return_trajectory = FakeTrajectory(object(), [0.9525, 0.4, 0.6])

    result = asyncio.run(
        task._plan_occluded_reposition(spec, FakeManipulator())
    )

    assert result == (outbound_trajectory, return_trajectory)
    assert len(calls) == 2
    outbound_request, return_request = calls
    assert "goals" not in outbound_request
    assert outbound_request["planning_pipeline"] == "aps_rrtc"
    assert outbound_request["planning_time"] == 0.25
    assert outbound_request["max_attempts"] == 1
    assert "path_constraints" not in outbound_request
    assert outbound_request["use_cache"] is False
    assert outbound_request["apply_smoothing"] is True
    assert outbound_request["velocity_scaling_factor"] == 0.75
    assert outbound_request["acceleration_scaling_factor"] == 1.0
    assert return_request["goal"] is center
    assert return_request["start_state"] is outbound_trajectory
    assert return_request["planning_pipeline"] == "aps_rrtc"
    assert return_request["planning_time"] == 0.25
    assert return_request["max_attempts"] == 1
    assert "path_constraints" not in return_request
    assert return_request["use_cache"] is False
    assert return_request["apply_smoothing"] is True
    assert return_request["velocity_scaling_factor"] == 0.75
    assert return_request["acceleration_scaling_factor"] == 1.0

    center_position, _ = arrays_from_pose_msg(center.pose, euler=True)
    target_position, _ = arrays_from_pose_msg(
        outbound_request["goal"].pose, euler=True
    )
    assert np.all(
        np.abs(target_position - center_position)
        <= task.position_half_ranges + 1e-12
    )


def test_production_config_uses_only_the_two_selected_objects() -> None:
    config_path = (
        Path(__file__).parents[1] / "config" / "invariant_affordance.yaml"
    )
    with config_path.open(encoding="utf-8") as stream:
        task_kwargs = yaml.safe_load(stream)["tasks"][0]["kwargs"]

    grouped = task_kwargs["trial_generator"]["kwargs"]["grouped_object_ids"]
    assert list(grouped) == ["right_manipulator", "left_manipulator"]
    assert grouped["right_manipulator"] == ["small_object_28"]
    assert grouped["left_manipulator"] == ["small_object_0"]
    assert task_kwargs["stimulus_duration"] == 1.0
    assert task_kwargs["occlusion_settle_duration"] == 0.5
    assert task_kwargs["motion_duration"] == 3.0
    assert task_kwargs["post_motion_hold_duration"] == 0.25
    assert task_kwargs["response_duration"] == 2.0
    assert task_kwargs["position_half_ranges"] == [0.10, 0.06, 0.08]
    assert task_kwargs["path_centerline_overlap"] == 0.12
    assert task_kwargs["orientation_strata"] == 3
    assert task_kwargs["motion_planning_pipeline"] == "aps_rrtc"
    assert task_kwargs["motion_planning_time"] == 0.25
    assert task_kwargs["motion_velocity_scaling_factor"] == 0.75
    assert task_kwargs["motion_acceleration_scaling_factor"] == 1.0
    assert task_kwargs["max_motion_generation_attempts"] == 30

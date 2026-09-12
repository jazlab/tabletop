"""Invariant-affordance task with a bounded occluded pose change.

Each trial starts with one object at the established center of the
presentation region. The glass reveals it for one second and returns opaque;
the robot then waits briefly for the glass to settle before moving directly
and briskly to a substantially different, random orientation inside the
presentation region. The trial holds that pose briefly after motion ends.
An audible go cue and both one-second hand buzzers then occur, followed by a
two-second hold at the final pose.
"""

import asyncio
import math
import time
from collections.abc import Mapping, Sequence
from copy import deepcopy
from typing import Any

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from tabletop_interfaces.msg import TaskEvent
from tabletop_rig.exceptions import PlanningError
from tabletop_rig.nodes import Commander
from tabletop_rig.nodes.commander import ManipulationContextManager
from tabletop_rig.utils.ros import (
    arrays_from_pose_msg,
    pose_stamped_msg,
)

from tabletop_tasks.tasks.base import BaseObjectInteractionTask
from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


def _vector3(name: str, values: Sequence[float]) -> np.ndarray:
    """Return a finite, non-negative three-vector from configuration."""
    result = np.asarray(values, dtype=float)
    if result.shape != (3,):
        raise ValueError(f"{name} must contain exactly three values")
    if not np.all(np.isfinite(result)) or np.any(result < 0):
        raise ValueError(f"{name} values must be finite and non-negative")
    return result


def generate_bounded_bezier_poses(
    center_pose: PoseStamped,
    *,
    rng: np.random.Generator,
    position_half_ranges: Sequence[float],
    orientation_half_ranges: Sequence[float],
    num_waypoints: int,
) -> list[PoseStamped]:
    """Generate a cubic Bezier path wholly inside a pose-space box.

    The first Bezier control is the centered presentation pose. The other
    three controls are sampled independently inside the configured position
    and roll/pitch/yaw bounds. A Bezier curve is a convex combination of its
    controls, so every generated position and Euler-angle offset remains
    inside the same bounds. Small configured angle ranges avoid wraparound.
    """
    if num_waypoints < 2:
        raise ValueError("num_waypoints must be at least 2")

    position_bounds = _vector3("position_half_ranges", position_half_ranges)
    orientation_bounds = _vector3(
        "orientation_half_ranges", orientation_half_ranges
    )
    center_position, center_rpy = arrays_from_pose_msg(
        center_pose.pose, euler=True
    )

    position_controls = np.vstack(
        (
            np.zeros(3),
            rng.uniform(-position_bounds, position_bounds, size=(3, 3)),
        )
    )
    orientation_controls = np.vstack(
        (
            np.zeros(3),
            rng.uniform(-orientation_bounds, orientation_bounds, size=(3, 3)),
        )
    )

    poses: list[PoseStamped] = []
    for t in np.linspace(0.0, 1.0, num_waypoints + 1)[1:]:
        weights = np.array(
            [
                (1.0 - t) ** 3,
                3.0 * (1.0 - t) ** 2 * t,
                3.0 * (1.0 - t) * t**2,
                t**3,
            ]
        )
        position = center_position + weights @ position_controls
        rpy = center_rpy + weights @ orientation_controls
        poses.append(
            pose_stamped_msg(
                header=deepcopy(center_pose.header),
                position=position,
                rpy=rpy,
            )
        )
    return poses


def generate_distant_target_pose(
    center_pose: PoseStamped,
    *,
    rng: np.random.Generator,
    position_half_ranges: Sequence[float],
    orientation_half_ranges: Sequence[float],
    minimum_position_delta: float,
    minimum_orientation_delta: float,
    minimum_tilt_delta: float = 0.0,
) -> PoseStamped:
    """Sample a target inside the envelope but away from its center.

    Rejection sampling avoids the barely perceptible center-biased endpoints
    produced by unconstrained uniform sampling. Position and Euler-angle
    offsets must independently meet their configured Euclidean minimums.
    The optional tilt minimum additionally requires a large roll or pitch
    change, so a yaw-only turn cannot satisfy a requested reorientation.
    """
    position_bounds = _vector3("position_half_ranges", position_half_ranges)
    orientation_bounds = _vector3(
        "orientation_half_ranges", orientation_half_ranges
    )
    for name, value, bounds in (
        ("minimum_position_delta", minimum_position_delta, position_bounds),
        (
            "minimum_orientation_delta",
            minimum_orientation_delta,
            orientation_bounds,
        ),
    ):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and non-negative")
        maximum_delta = float(np.linalg.norm(bounds))
        if value > 0 and value >= maximum_delta:
            raise ValueError(
                f"{name} must be smaller than the envelope's maximum delta"
            )
    if not math.isfinite(minimum_tilt_delta) or minimum_tilt_delta < 0:
        raise ValueError("minimum_tilt_delta must be finite and non-negative")
    if minimum_tilt_delta > float(np.max(orientation_bounds[:2])):
        raise ValueError(
            "minimum_tilt_delta cannot exceed both roll and pitch bounds"
        )

    center_position, center_rpy = arrays_from_pose_msg(
        center_pose.pose, euler=True
    )
    for _ in range(10_000):
        position_offset = rng.uniform(
            -position_bounds, position_bounds, size=3
        )
        orientation_offset = rng.uniform(
            -orientation_bounds, orientation_bounds, size=3
        )
        if (
            np.linalg.norm(position_offset) >= minimum_position_delta
            and np.linalg.norm(orientation_offset) >= minimum_orientation_delta
            and np.max(np.abs(orientation_offset[:2])) >= minimum_tilt_delta
        ):
            return pose_stamped_msg(
                header=deepcopy(center_pose.header),
                position=center_position + position_offset,
                rpy=center_rpy + orientation_offset,
            )
    raise ValueError("could not sample a distant target inside the envelope")


def generate_uniform_target_pose(
    center_pose: PoseStamped,
    *,
    rng: np.random.Generator,
    position_half_ranges: Sequence[float],
    local_up_cosine_range: Sequence[float] | None = None,
) -> PoseStamped:
    """Sample position uniformly in a box and orientation uniformly on SO(3).

    The Shoemake quaternion construction is Haar-uniform: equal-volume
    subsets of 3D rotation space have equal probability. This avoids the
    pole bias introduced by independently sampling Euler angles.
    """
    position_bounds = _vector3("position_half_ranges", position_half_ranges)
    center_position, _ = arrays_from_pose_msg(center_pose.pose, euler=True)
    position = center_position + rng.uniform(
        -position_bounds, position_bounds, size=3
    )

    cosine_range: tuple[float, float] | None = None
    if local_up_cosine_range is not None:
        values = np.asarray(local_up_cosine_range, dtype=float)
        if (
            values.shape != (2,)
            or not np.all(np.isfinite(values))
            or values[0] < -1.0
            or values[1] > 1.0
            or values[0] >= values[1]
        ):
            raise ValueError(
                "local_up_cosine_range must be two increasing finite values "
                "within [-1, 1]"
            )
        cosine_range = (float(values[0]), float(values[1]))

    while True:
        u1, u2, u3 = rng.random(3)
        x = math.sqrt(1.0 - u1) * math.sin(2.0 * math.pi * u2)
        y = math.sqrt(1.0 - u1) * math.cos(2.0 * math.pi * u2)
        z = math.sqrt(u1) * math.sin(2.0 * math.pi * u3)
        w = math.sqrt(u1) * math.cos(2.0 * math.pi * u3)
        local_up_cosine = 1.0 - 2.0 * (x * x + y * y)
        if cosine_range is None or (
            cosine_range[0] <= local_up_cosine <= cosine_range[1]
        ):
            break
    return pose_stamped_msg(
        header=deepcopy(center_pose.header),
        position=position,
        orientation=(w, x, y, z),
    )


def validate_trajectory_arm_half(
    trajectory: RobotTrajectory,
    center_pose: PoseStamped,
    *,
    group_name: str,
    centerline_overlap: float,
) -> None:
    """Keep a trajectory on its arm's half, with a center overlap allowance.

    MoveIt collision-checks the path between trajectory states. Checking each
    resulting state here prevents an arm from crossing deeply into the other
    arm's workspace while allowing complex routes back toward its own storage
    area. The overlap lets both arms reach the center presentation pose.
    """
    if not math.isfinite(centerline_overlap) or centerline_overlap < 0:
        raise ValueError("centerline_overlap must be finite and non-negative")
    center_position, _ = arrays_from_pose_msg(center_pose.pose, euler=True)
    side = group_name.removesuffix("_manipulator")
    if side not in {"left", "right"}:
        raise ValueError(f"unsupported manipulator group: {group_name}")
    link_name = f"{side}_eef"
    for waypoint_index in range(len(trajectory)):
        position = trajectory[waypoint_index].get_pose(link_name).position
        crosses_centerline = (
            side == "left"
            and position.x > center_position[0] + centerline_overlap + 1e-9
        ) or (
            side == "right"
            and position.x < center_position[0] - centerline_overlap - 1e-9
        )
        if crosses_centerline:
            raise ValueError(
                f"trajectory waypoint {waypoint_index} crosses the "
                f"{side}-arm workspace boundary: x={position.x:.4f}"
            )


def _scale_twist(twist: Any, factor: float) -> None:
    """Scale every component of a geometry_msgs/Twist in place."""
    for vector in (twist.linear, twist.angular):
        vector.x *= factor
        vector.y *= factor
        vector.z *= factor


def stretch_trajectory_message(
    message: RobotTrajectoryMsg, scale: float
) -> RobotTrajectoryMsg:
    """Return a copy of a trajectory message slowed by scale.

    Time is multiplied by scale; velocities and accelerations are divided by
    scale and scale squared respectively. Positions are untouched, so the
    collision-checked geometric path is unchanged. Only scale values at least
    one are accepted: this helper can never speed a trajectory up.
    """
    if not math.isfinite(scale) or scale < 1.0:
        raise ValueError("trajectory scale must be finite and at least 1")

    result = deepcopy(message)
    for point in result.joint_trajectory.points:
        seconds = point.time_from_start.sec + (
            point.time_from_start.nanosec / 1_000_000_000
        )
        nanoseconds = round(seconds * scale * 1_000_000_000)
        point.time_from_start.sec = nanoseconds // 1_000_000_000
        point.time_from_start.nanosec = nanoseconds % 1_000_000_000
        point.velocities = [value / scale for value in point.velocities]
        point.accelerations = [
            value / (scale * scale) for value in point.accelerations
        ]

    for point in result.multi_dof_joint_trajectory.points:
        seconds = point.time_from_start.sec + (
            point.time_from_start.nanosec / 1_000_000_000
        )
        nanoseconds = round(seconds * scale * 1_000_000_000)
        point.time_from_start.sec = nanoseconds // 1_000_000_000
        point.time_from_start.nanosec = nanoseconds % 1_000_000_000
        for velocity in point.velocities:
            _scale_twist(velocity, 1.0 / scale)
        for acceleration in point.accelerations:
            _scale_twist(acceleration, 1.0 / (scale * scale))

    return result


def stretch_trajectory_to_duration(
    trajectory: RobotTrajectory, target_duration: float
) -> RobotTrajectory:
    """Stretch a planned trajectory to exactly target_duration seconds."""
    original_duration = float(trajectory.duration)
    if original_duration <= 0:
        raise ValueError("planned trajectory duration must be positive")
    if original_duration > target_duration + 1e-6:
        raise ValueError(
            f"planned trajectory is {original_duration:.3f}s, longer than "
            f"the {target_duration:.3f}s target"
        )

    scale = max(1.0, target_duration / original_duration)
    start_state = trajectory[0]
    group_name = trajectory.joint_model_group_name
    message = stretch_trajectory_message(
        trajectory.get_robot_trajectory_msg(), scale
    )
    trajectory.set_robot_trajectory_msg(start_state, message)
    trajectory.joint_model_group_name = group_name
    return trajectory


class InvariantAffordanceTask(BaseObjectInteractionTask):
    """Alternate two front objects through a bounded moving-object trial."""

    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
        *,
        stimulus_duration: float = 1.0,
        occlusion_settle_duration: float = 0.5,
        motion_duration: float = 3.0,
        post_motion_hold_duration: float = 0.25,
        response_duration: float = 2.0,
        sound_kwargs: Mapping[str, Any] | None = None,
        position_half_ranges: Sequence[float] = (0.14, 0.08, 0.10),
        path_centerline_overlap: float = 0.12,
        orientation_strata: int = 3,
        motion_planning_pipeline: str = "aps_rrtc",
        motion_planning_time: float = 0.25,
        motion_velocity_scaling_factor: float = 0.75,
        motion_acceleration_scaling_factor: float = 1.0,
        max_motion_generation_attempts: int = 10,
        random_seed: int | None = None,
    ) -> None:
        """Configure phase timing and the bounded reposition envelope."""
        super().__init__(
            "invariant_affordance_task", commander, trial_generator
        )
        for name, value in (
            ("stimulus_duration", stimulus_duration),
            ("motion_duration", motion_duration),
            ("response_duration", response_duration),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and greater than 0")
        for name, value in (
            ("occlusion_settle_duration", occlusion_settle_duration),
            ("post_motion_hold_duration", post_motion_hold_duration),
        ):
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if max_motion_generation_attempts < 1:
            raise ValueError(
                "max_motion_generation_attempts must be at least 1"
            )
        if not motion_planning_pipeline:
            raise ValueError("motion_planning_pipeline cannot be empty")
        if (
            not math.isfinite(motion_planning_time)
            or motion_planning_time <= 0
        ):
            raise ValueError(
                "motion_planning_time must be finite and greater than 0"
            )
        for name, value in (
            (
                "motion_velocity_scaling_factor",
                motion_velocity_scaling_factor,
            ),
            (
                "motion_acceleration_scaling_factor",
                motion_acceleration_scaling_factor,
            ),
        ):
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f"{name} must be finite and in (0, 1]")

        self.stimulus_duration = float(stimulus_duration)
        self.occlusion_settle_duration = float(occlusion_settle_duration)
        self.motion_duration = float(motion_duration)
        self.post_motion_hold_duration = float(post_motion_hold_duration)
        self.response_duration = float(response_duration)
        self.sound_kwargs = dict(sound_kwargs or {})
        self.position_half_ranges = _vector3(
            "position_half_ranges", position_half_ranges
        )
        if (
            not math.isfinite(path_centerline_overlap)
            or path_centerline_overlap < 0
        ):
            raise ValueError(
                "path_centerline_overlap must be finite and non-negative"
            )
        if not isinstance(orientation_strata, int) or orientation_strata < 2:
            raise ValueError(
                "orientation_strata must be an integer at least 2"
            )
        self.path_centerline_overlap = float(path_centerline_overlap)
        self.orientation_strata = orientation_strata
        self.motion_planning_pipeline = motion_planning_pipeline
        self.motion_planning_time = float(motion_planning_time)
        self.motion_velocity_scaling_factor = float(
            motion_velocity_scaling_factor
        )
        self.motion_acceleration_scaling_factor = float(
            motion_acceleration_scaling_factor
        )
        self.max_motion_generation_attempts = max_motion_generation_attempts
        self.max_trajectory_duration = (
            self.motion_duration
            - self.occlusion_settle_duration
            - self.post_motion_hold_duration
        )
        if self.max_trajectory_duration <= 0:
            raise ValueError(
                "motion_duration must exceed occlusion_settle_duration plus "
                "post_motion_hold_duration"
            )
        self._rng = np.random.default_rng(random_seed)
        self._orientation_strata_queues: dict[
            str, list[tuple[float, float]]
        ] = {}

    def _next_orientation_cosine_range(
        self, group_name: str
    ) -> tuple[float, float]:
        """Return a shuffled equal-area tilt stratum for one arm."""
        queue = self._orientation_strata_queues.setdefault(group_name, [])
        if not queue:
            edges = np.linspace(-1.0, 1.0, self.orientation_strata + 1)
            for index in self._rng.permutation(self.orientation_strata):
                queue.append((float(edges[index]), float(edges[index + 1])))
        # Keep offering this stratum until a plan is accepted, so a planning
        # miss cannot silently remove a tilt category from the cycle.
        return queue[-1]

    def _publish_phase(self, event_type: int, trial_spec: TrialSpec) -> None:
        self.commander.publish_task_event(
            event_type,
            trial_number=trial_spec.trial_number,
            group_name=trial_spec.group_name,
            object_id=trial_spec.object_id,
        )

    async def _plan_occluded_reposition(
        self,
        trial_spec: TrialSpec,
        manipulator: ManipulationContextManager,
    ) -> tuple[RobotTrajectory, RobotTrajectory]:
        """Plan validated outbound and center-return moves before reveal."""
        errors: list[PlanningError | ValueError] = []
        orientation_range = self._next_orientation_cosine_range(
            trial_spec.group_name
        )
        for attempt in range(1, self.max_motion_generation_attempts + 1):
            goal = generate_uniform_target_pose(
                trial_spec.object_pose,
                rng=self._rng,
                position_half_ranges=self.position_half_ranges,
                local_up_cosine_range=orientation_range,
            )
            try:
                outbound_trajectory = await manipulator.plan(
                    goal=goal,
                    group_name=trial_spec.group_name,
                    planning_pipeline=self.motion_planning_pipeline,
                    planning_time=self.motion_planning_time,
                    max_attempts=1,
                    use_cache=False,
                    apply_smoothing=True,
                    velocity_scaling_factor=(
                        self.motion_velocity_scaling_factor
                    ),
                    acceleration_scaling_factor=(
                        self.motion_acceleration_scaling_factor
                    ),
                )
                if (
                    float(outbound_trajectory.duration)
                    > self.max_trajectory_duration + 1e-6
                ):
                    raise ValueError(
                        "reposition trajectory takes "
                        f"{outbound_trajectory.duration:.3f}s, "
                        "longer than the "
                        f"{self.max_trajectory_duration:.3f}s execution "
                        "budget"
                    )
                validate_trajectory_arm_half(
                    outbound_trajectory,
                    trial_spec.object_pose,
                    group_name=trial_spec.group_name,
                    centerline_overlap=self.path_centerline_overlap,
                )
                return_trajectory = await manipulator.plan(
                    goal=trial_spec.object_pose,
                    start_state=outbound_trajectory[
                        len(outbound_trajectory) - 1
                    ],
                    group_name=trial_spec.group_name,
                    planning_pipeline=self.motion_planning_pipeline,
                    planning_time=self.motion_planning_time,
                    max_attempts=1,
                    use_cache=False,
                    apply_smoothing=True,
                    velocity_scaling_factor=(
                        self.motion_velocity_scaling_factor
                    ),
                    acceleration_scaling_factor=(
                        self.motion_acceleration_scaling_factor
                    ),
                )
                validate_trajectory_arm_half(
                    return_trajectory,
                    trial_spec.object_pose,
                    group_name=trial_spec.group_name,
                    centerline_overlap=self.path_centerline_overlap,
                )
                position = goal.pose.position
                orientation = goal.pose.orientation
                local_up_cosine = 1.0 - 2.0 * (
                    orientation.x**2 + orientation.y**2
                )
                self._orientation_strata_queues[trial_spec.group_name].pop()
                self.log(
                    "Accepted uniform occluded target "
                    f"{attempt}/{self.max_motion_generation_attempts} for "
                    f"{trial_spec.group_name}: position="
                    f"({position.x:.4f}, {position.y:.4f}, {position.z:.4f}), "
                    "quaternion="
                    f"({orientation.w:.4f}, {orientation.x:.4f}, "
                    f"{orientation.y:.4f}, {orientation.z:.4f}), "
                    f"local_up_cosine={local_up_cosine:.4f}, "
                    f"tilt_stratum=({orientation_range[0]:.4f}, "
                    f"{orientation_range[1]:.4f}), "
                    f"duration={outbound_trajectory.duration:.3f}s"
                )
                return outbound_trajectory, return_trajectory
            except (PlanningError, ValueError) as error:
                errors.append(error)
                self.log(
                    f"Occluded reposition attempt {attempt}/"
                    f"{self.max_motion_generation_attempts} failed for "
                    f"{trial_spec.group_name}: {error}",
                    severity="WARN",
                )

        raise PlanningError(
            "Could not produce a collision-free uniformly sampled "
            "presentation pose within the fixed "
            f"{self.motion_duration:.3f}s interval after "
            f"{self.max_motion_generation_attempts} attempts; last error: "
            f"{errors[-1]}",
            trial_spec.group_name,
        )

    async def run_trial(
        self, trial_spec: TrialSpec, manipulator: ManipulationContextManager
    ) -> TrialFeedback:
        """Run reveal, bounded motion, simultaneous cues, and final hold."""
        self.log(f"Invariant-affordance trial spec: {trial_spec}")

        # Plan before the stimulus becomes visible. Retiming is allowed only to
        # slow a valid plan, never to accelerate one to fit the phase duration.
        trajectory, return_trajectory = await self._plan_occluded_reposition(
            trial_spec, manipulator
        )

        await self.commander.reveal_smartglass()
        self._publish_phase(TaskEvent.STIMULUS_REVEALED, trial_spec)
        await asyncio.sleep(self.stimulus_duration)
        await self.commander.occlude_smartglass()
        occluded_at = time.monotonic()

        # Give the switched glass time to become fully opaque before any
        # visible object motion begins.
        if self.occlusion_settle_duration > 0:
            await asyncio.sleep(self.occlusion_settle_duration)

        self._publish_phase(TaskEvent.MOTION_STARTED, trial_spec)
        await manipulator.move(trajectory)

        # Hold until the fixed occlusion-to-cue deadline. Planning reserves
        # post_motion_hold_duration inside this interval so normal execution
        # finishes before the deadline rather than causing a late cue.
        remaining_occluded_interval = self.motion_duration - (
            time.monotonic() - occluded_at
        )
        if remaining_occluded_interval > 0:
            await asyncio.sleep(remaining_occluded_interval)
        else:
            self.log(
                "Occluded motion exceeded its fixed cue deadline by "
                f"{-remaining_occluded_interval:.3f}s; cueing immediately",
                severity="WARN",
            )

        cue_started = time.monotonic()
        self._publish_phase(TaskEvent.GO_CUE, trial_spec)
        async with asyncio.TaskGroup() as task_group:
            task_group.create_task(
                self.commander.play_sound(**self.sound_kwargs)
            )
            task_group.create_task(self.commander.pulse_buzzer("both"))

        # Hold the random final pose until exactly response_duration after cue
        # onset. No response device is consulted in this version of the task.
        remaining = self.response_duration - (time.monotonic() - cue_started)
        if remaining > 0:
            await asyncio.sleep(remaining)

        # The behavioral trial is complete. While the glass remains opaque,
        # follow the separately planned and validated trajectory back to the
        # standard center pose. This gives the generic unpresentation state
        # machine its known, reliable starting geometry without exposing
        # cleanup motion.
        await manipulator.move(return_trajectory)
        return TrialFeedback()

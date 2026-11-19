"""Smooth pursuit eye tracking task.

This module provides a task for smooth pursuit eye tracking experiments.
The robot moves an object along predefined trajectories (spiral or random)
while tracking eye movements and providing rewards for successful tracking.

The task supports several motion types:
- Spiral: Helical trajectory around a center point
- Sin: Sinusoidal motion on three axes
- Random: Random waypoints within a bounding box
- Depth sweep: Smooth near–far loops through a grid of viewing directions
- Bearing depth: Repeat one angular figure at different viewing distances
- Static grid: Standalone fixation block

This task does not use the standard trial-based structure. Instead, it
continuously executes the trajectory and monitors eye tracking.

Example:
    task = SmoothPursuitTask(
        commander=commander,
        motion_type="spiral",
        motion_kwargs={"center_pose_kwargs": {...}, "radius": 0.1, ...},
        num_repetitions=5,
        object_id="target_object",
    )
    await task.run()
"""

import asyncio
import math
from collections.abc import Iterator, Mapping
from itertools import product
from time import monotonic
from typing import Any, Literal

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from tabletop_rig.exceptions import PlanningError
from tabletop_rig.nodes import Commander
from tabletop_rig.nodes.commander import (
    ManipulationContextManager,
    _contains_start_state_collision,
)
from tabletop_rig.utils.ros import pose_stamped_msg

from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.tasks.gaze_bearing_depth import gaze_bearing_depth_offsets
from tabletop_tasks.tasks.gaze_corner_timing import pursuit_turn_clock
from tabletop_tasks.tasks.gaze_depth_sweep import gaze_depth_sweep_offsets
from tabletop_tasks.tasks.invariant_affordance import (
    stretch_trajectory_to_duration,
)

_TIMED_PURSUIT_EVENTS = {
    "depth_sweep": "DEPTH_SWEEP",
    "bearing_depth": "BEARING_DEPTH",
}

_BEARING_DEPTH_LIN_OPTIONS = {
    "planning_velocity_scaling_factor": 0.25,
    "planning_acceleration_scaling_factor": 0.20,
}


class SmoothPursuitTask(BaseTask):
    """Task for smooth pursuit eye tracking experiments.

    Moves an attached object along a predefined trajectory while
    monitoring eye tracking.
    Rewards are delivered based on successful gaze tracking.

    Unlike other tasks, SmoothPursuitTask does not use trial generators
    or the standard trial-based structure. It runs continuously until
    either the trajectory repetitions complete or smooth pursuit tracking
    ends.

    Attributes:
        _goals: List of PoseStamped waypoints for the trajectory.
        _num_repetitions: Number of times to repeat the trajectory.
        _velocity_scaling_factor: Scaling factor for trajectory velocity.
        _post_process_after_concat: Whether to smooth the trajectory
            after concatenating segments.
    """

    def __init__(
        self,
        commander: Commander,
        *,
        motion_type: Literal[
            "spiral", "sin", "random", "depth_sweep", "bearing_depth", "static_grid"
        ],
        motion_kwargs: Mapping[str, Any],
        num_repetitions: int,
        object_id: str,
        robot_name: str,
        velocity_scaling_factor: float = 1.0,
        max_motion_generation_attempts: int = 5,
        static_hold_seconds: float | None = None,
        static_half_span_m: float = 0.07,
        pursuit_duration_seconds: float | None = None,
        static_block_seconds: float | None = None,
        corner_speed_fraction: float | None = None,
    ):
        """Initialize the smooth pursuit task.

        Args:
            commander: Commander instance for robot interaction.
            motion_type: "spiral", "sin", "random", "depth_sweep",
                "bearing_depth", or "static_grid" for a standalone fixation grid.
            motion_kwargs: Keyword arguments for the trajectory generator.
                For spiral: center_pose, radius, length, num_revolutions,
                    num_segments.
                For sin: center_pose, amplitudes, periods, num_revolutions,
                    num_segments.
                For random: start_pose, min_x, max_x, min_y, max_y, min_z,
                    max_z, num_waypoints.
            num_repetitions: Number of times to execute the trajectory.
            object_id: ID of the object to attach to the end effector.
            robot_name: Name of the robot group to execute trajectory
                (e.g., "left_manipulator", "right_manipulator").
            velocity_scaling_factor: Scaling factor for trajectory velocity
                (default 1.0, range 0.0-1.0).
            max_motion_generation_attempts: Maximum number of trajectory
                generation attempts before giving up (default 5).
            static_hold_seconds: Optional seconds per fixation after the last
                sinusoidal lap or single depth-calibration path.
            static_half_span_m: Distance from grid center to outer levels.
            pursuit_duration_seconds: Target planned duration of a depth
                calibration, attained only by slowing an already validated path.
            static_block_seconds: Budget including moves and holds, for a
                standalone static grid, starting after approach and reveal.
            corner_speed_fraction: Optional lowest speed fraction for tight
                turns in a single sinusoidal lap. Never speeds up the plan.

        Raises:
            ValueError: If a motion type or duration is unsupported.
        """
        super().__init__("smooth_pursuit_task", commander)
        if motion_type == "static_grid":
            if (
                num_repetitions != 1
                or static_block_seconds is None
                or static_hold_seconds is None
                or not math.isfinite(static_block_seconds)
                or static_block_seconds <= static_hold_seconds
                or set(motion_kwargs) != {"center_pose"}
            ):
                raise ValueError(
                    "A static grid needs one block, a center pose, and a positive time and hold budget"
                )
        elif static_block_seconds is not None:
            raise ValueError("A static block duration applies only to a static grid")
        self._pursuit_event = _TIMED_PURSUIT_EVENTS.get(motion_type)
        if self._pursuit_event is not None and num_repetitions != 1:
            raise ValueError("A depth calibration path must run exactly once")
        if pursuit_duration_seconds is not None and (
            self._pursuit_event is None
            or not math.isfinite(pursuit_duration_seconds)
            or pursuit_duration_seconds <= 0
        ):
            raise ValueError(
                "A positive pursuit duration applies only to a depth calibration"
            )
        if static_hold_seconds is not None:
            if (
                motion_type
                not in ("sin", "depth_sweep", "bearing_depth", "static_grid")
                or num_repetitions < 1
            ):
                raise ValueError(
                    "Static holds require a sinusoidal lap, depth calibration, or a static grid"
                )
            if (
                not math.isfinite(static_hold_seconds)
                or static_hold_seconds <= 0
            ):
                raise ValueError(
                    "static_hold_seconds must be finite and positive"
                )
            if (
                not math.isfinite(static_half_span_m)
                or static_half_span_m <= 0
            ):
                raise ValueError(
                    "static_half_span_m must be finite and positive"
                )
        self._static_hold_seconds = static_hold_seconds
        self._static_half_span_m = static_half_span_m
        self._static_block_seconds = static_block_seconds
        self._pursuit_duration_seconds = pursuit_duration_seconds
        if corner_speed_fraction is not None and (
            motion_type != "sin"
            or num_repetitions != 1
            or not 0.70 <= corner_speed_fraction < 1.0
        ):
            raise ValueError(
                "Corner easing requires one sinusoidal lap and a speed fraction in [0.70, 1)"
            )
        self._corner_speed_fraction = corner_speed_fraction
        match motion_type:
            case "spiral":
                self._post_process_after_concat = True
                self._motion_fn = self.generate_spiral
            case "sin":
                self._post_process_after_concat = True
                self._motion_fn = self.generate_sin
            case "depth_sweep":
                self._post_process_after_concat = True
                self._motion_fn = self.generate_depth_sweep
            case "bearing_depth":
                self._post_process_after_concat = True
                self._motion_fn = self.generate_bearing_depth
            case "random":
                self._post_process_after_concat = False
                self._motion_fn = self.generate_random
            case "static_grid":
                self._post_process_after_concat = False
                self._motion_fn = None
            case _:
                raise ValueError(f"Unsupported motion type: {motion_type}")

        self._object_id = object_id
        self._robot_name = robot_name
        self._motion_type = motion_type
        self._motion_kwargs = motion_kwargs
        self._num_repetitions = num_repetitions
        self._velocity_scaling_factor = velocity_scaling_factor
        self._max_motion_generation_attempts = max_motion_generation_attempts

    def generate_spiral(
        self,
        center_pose: PoseStamped | Mapping[str, Any],
        radius: float,
        length: float,
        num_revolutions: int,
        num_segments: int,
    ) -> list[PoseStamped]:
        """Generate a helical spiral trajectory.

        Creates waypoints for a 3D spiral (helix) centered on a given pose.
        The spiral moves in the XZ plane while oscillating along Y, creating
        a spring-like trajectory.

        The parametric equations are:
        - x = center_x + radius * cos(theta_xz)
        - y = center_y - (length/2) * cos(theta_y)
        - z = center_z + radius * sin(theta_xz)

        Where theta_xz controls the XZ rotation (num_revolutions times
        around) and theta_y controls the Y oscillation (one full cycle).

        Args:
            center_pose: PoseStamped or dict of pose parameters for the
                spiral center (passed to pose_stamped_msg if dict).
            radius: Radius of the spiral in the XZ plane (meters).
            length: Total length of Y-axis oscillation (meters).
            num_revolutions: Number of complete rotations in XZ plane.
            num_segments: Number of waypoints to generate.

        Returns:
            List of PoseStamped waypoints forming the spiral trajectory.
        """
        self.log("Generating spiral trajectory")

        if not isinstance(center_pose, PoseStamped):
            center_pose = pose_stamped_msg(**center_pose)

        goals: list[PoseStamped] = []

        for i in range(num_segments + 1):
            # XZ plane rotation (multiple revolutions)
            theta_xz = (2 * np.pi * i * num_revolutions) / num_segments
            # Y axis oscillation (single cycle)
            theta_y = (2 * np.pi * i) / num_segments

            x = center_pose.pose.position.x + radius * np.cos(theta_xz)
            y = center_pose.pose.position.y - (length / 2) * np.cos(theta_y)
            z = center_pose.pose.position.z + radius * np.sin(theta_xz)

            goal = pose_stamped_msg(
                position=[x, y, z],
                orientation=center_pose.pose.orientation,
            )
            goals.append(goal)

        return goals

    def generate_sin(
        self,
        center_pose: PoseStamped | Mapping[str, Any],
        amplitudes: list[float],
        periods: list[float],
        num_revolutions: int,
        num_segments: int,
    ) -> list[PoseStamped]:
        """Generate a sinusoidal oscillation trajectory in 3D.

        Creates waypoints with independent sinusoidal oscillation along
        X, Y, and Z axes. Each axis oscillates at its own frequency and
        amplitude. num_revolutions controls how many cycles occur along
        the axis with the largest period.

        Args:
            center_pose: PoseStamped or dict of pose parameters for the
                oscillation center (passed to pose_stamped_msg if dict).
            amplitudes: List of three amplitudes for [X, Y, Z] oscillation
                (meters).
            periods: List of three periods for [X, Y, Z] oscillation (seconds
                or normalized units).
            num_revolutions: Number of sinusoidal cycles along the axis
                with the largest period.
            num_segments: Number of waypoints to generate.

        Returns:
            List of PoseStamped waypoints forming the oscillation trajectory.

        Raises:
            ValueError: If amplitudes or periods are not length 3.
        """
        self.log("Generating sinusoidal trajectory")

        if not isinstance(center_pose, PoseStamped):
            center_pose = pose_stamped_msg(**center_pose)

        goals: list[PoseStamped] = []

        if len(amplitudes) != 3 or len(periods) != 3:
            raise ValueError("amplitudes and periods must both be of length 3")

        max_period = max(periods)

        for i in range(num_segments + 1):
            t = (2 * np.pi * i * num_revolutions * max_period) / num_segments
            offset = [a * np.sin(t / p) for a, p in zip(amplitudes, periods)]

            x = center_pose.pose.position.x + offset[0]
            y = center_pose.pose.position.y + offset[1]
            z = center_pose.pose.position.z + offset[2]

            goal = pose_stamped_msg(
                position=[x, y, z],
                orientation=center_pose.pose.orientation,
            )
            goals.append(goal)

        return goals

    def generate_random(
        self,
        start_pose: PoseStamped | Mapping[str, Any],
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        min_z: float,
        max_z: float,
        num_waypoints: int,
    ) -> list[PoseStamped]:
        """Generate a trajectory through random waypoints.

        Creates waypoints by uniformly sampling positions within a
        3D bounding box. The start pose is included as the first
        waypoint, followed by randomly sampled positions.

        All waypoints maintain the same orientation as the start pose.

        Args:
            start_pose: PoseStamped or dict of pose parameters for the
                initial position (passed to pose_stamped_msg if dict).
            min_x: Minimum X coordinate for random sampling (meters).
            max_x: Maximum X coordinate for random sampling (meters).
            min_y: Minimum Y coordinate for random sampling (meters).
            max_y: Maximum Y coordinate for random sampling (meters).
            min_z: Minimum Z coordinate for random sampling (meters).
            max_z: Maximum Z coordinate for random sampling (meters).
            num_waypoints: Number of random waypoints to generate
                (excluding the start pose).

        Returns:
            List of PoseStamped waypoints starting with the start pose
            followed by randomly sampled positions.
        """
        self.log("Generating random trajectory")

        if not isinstance(start_pose, PoseStamped):
            start_pose = pose_stamped_msg(**start_pose)

        goals: list[PoseStamped] = []
        goals.append(start_pose)

        for i in range(num_waypoints):
            # Uniform random sampling within bounding box
            x = (max_x - min_x) * np.random.random_sample() + min_x
            y = (max_y - min_y) * np.random.random_sample() + min_y
            z = (max_z - min_z) * np.random.random_sample() + min_z

            print(f"Goal {i}: {x}, {y}, {z}")

            goal = pose_stamped_msg(
                position=[x, y, z],
                orientation=start_pose.pose.orientation,
            )
            goals.append(goal)

        return goals

    def generate_depth_sweep(
        self, center_pose: PoseStamped | Mapping[str, Any], **geometry: Any
    ) -> list[PoseStamped]:
        """Generate a closed, fixed-orientation calibration path in world."""
        if not isinstance(center_pose, PoseStamped):
            center_pose = pose_stamped_msg(**center_pose)
        if center_pose.header.frame_id not in ("", "world"):
            raise ValueError("The depth sweep uses the world coordinate frame")
        self.log("Generating depth-balanced gaze calibration path")
        p = center_pose.pose.position
        points = gaze_depth_sweep_offsets(**geometry) + [p.x, p.y, p.z]
        return [
            pose_stamped_msg(
                frame_id="world",
                position=position,
                orientation=center_pose.pose.orientation,
            )
            for position in points
        ]

    def generate_bearing_depth(
        self, center_pose: PoseStamped | Mapping[str, Any], **geometry: Any
    ) -> list[PoseStamped]:
        """Generate the fixed-orientation repeated-bearing calibration path."""
        if not isinstance(center_pose, PoseStamped):
            center_pose = pose_stamped_msg(**center_pose)
        if center_pose.header.frame_id not in ("", "world"):
            raise ValueError("The repeated-bearing path uses the world coordinate frame")
        self.log("Generating repeated-bearing depth calibration path")
        p = center_pose.pose.position
        points = gaze_bearing_depth_offsets(**geometry) + [p.x, p.y, p.z]
        return [
            pose_stamped_msg(
                frame_id="world",
                position=position,
                orientation=center_pose.pose.orientation,
            )
            for position in points
        ]

    def _retime_pursuit(self, trajectory: RobotTrajectory) -> RobotTrajectory:
        if self._corner_speed_fraction is not None:
            return self._ease_sinusoid_corners(trajectory)
        if self._pursuit_duration_seconds is None:
            return trajectory
        duration = float(trajectory.duration)
        stretched = stretch_trajectory_to_duration(
            trajectory, self._pursuit_duration_seconds
        )
        label = (
            "Depth sweep"
            if self._motion_type == "depth_sweep"
            else "Repeated-bearing depth"
        )
        self.log(
            f"{label} planned duration: {duration:.2f}s → {stretched.duration:.2f}s"
        )
        return stretched

    def _ease_sinusoid_corners(self, trajectory: RobotTrajectory) -> RobotTrajectory:
        """Keep the checked waypoint positions and smoothly stretch tight turns."""
        assert self._corner_speed_fraction is not None
        seed = trajectory[0]
        group_name = trajectory.joint_model_group_name
        message = trajectory.get_robot_trajectory_msg()
        joints = message.joint_trajectory
        points = joints.points
        names = joints.joint_names
        if (
            len(points) != len(trajectory)
            or not names
            or message.multi_dof_joint_trajectory.points
            or any(
                not (
                    len(p.positions)
                    == len(p.velocities)
                    == len(p.accelerations)
                    == len(names)
                )
                for p in points
            )
        ):
            raise ValueError("Corner easing requires a fully timed joint trajectory")
        times = np.array(
            [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        )
        tcp_link = self._robot_name.removesuffix("_manipulator") + "_eef"
        poses = (trajectory[i].get_pose(tcp_link) for i in range(len(points)))
        tcp_xyz = np.array(
            [(p.x, p.y, p.z) for p in (pose.position for pose in poses)]
        )
        times_new, scale, slope = pursuit_turn_clock(
            times, tcp_xyz, self._corner_speed_fraction
        )
        nanos = np.rint(times_new * 1e9).astype(np.int64)
        if np.any(np.diff(nanos) <= 0):
            raise ValueError("Corner easing produced non-increasing waypoint times")
        q = np.array([p.positions for p in points])
        v = np.array([p.velocities for p in points])
        a = np.array([p.accelerations for p in points])
        v_new = v / scale[:, None]
        a_new = a / scale[:, None] ** 2 - v * slope[:, None] / scale[:, None] ** 3
        group = seed.robot_model.get_joint_model_group(group_name)
        limits = dict(
            zip(group.active_joint_model_names, group.active_joint_model_bounds, strict=True)
        )
        for i, name in enumerate(names):
            bound, = limits[name]
            if not np.isfinite(np.c_[q[:, i], v_new[:, i], a_new[:, i]]).all():
                raise ValueError(f"Invalid corner-eased values for {name}")
            if bound.velocity_bounded and (
                np.abs(v_new[:, i]).max() > bound.max_velocity + .001
                or (np.abs(np.diff(q[:, i])) / (np.diff(nanos) * 1e-9)).max()
                > bound.max_velocity + .001
            ):
                raise ValueError(f"Corner-eased trajectory exceeds {name} velocity limit")
            if (
                bound.acceleration_bounded
                and np.abs(a_new[:, i]).max() > bound.max_acceleration + .001
            ):
                raise ValueError(f"Corner-eased trajectory exceeds {name} acceleration limit")
        for p, ns, velocity, acceleration in zip(
            points, nanos, v_new, a_new, strict=True
        ):
            p.time_from_start.sec, p.time_from_start.nanosec = divmod(int(ns), 10**9)
            p.velocities = velocity.tolist()
            p.accelerations = acceleration.tolist()
        retimed = RobotTrajectory(seed.robot_model)
        retimed.set_robot_trajectory_msg(seed, message)
        retimed.joint_model_group_name = group_name
        if not self.commander._moveit.is_path_valid(retimed, verbose=False):
            raise ValueError("Corner-eased trajectory failed collision or joint checking")
        self.log(
            f"Sinusoid corner easing: {times[-1]:.2f}s → {retimed.duration:.2f}s; "
            f"tightest turn at {100 / scale.max():.0f}% of original speed"
        )
        return retimed

    async def execute_loop(
        self,
        manipulator: ManipulationContextManager,
        trajectory: RobotTrajectory,
    ):
        """Execute the trajectory repeatedly.

        Executes the planned trajectory for the specified number of
        repetitions (num_repetitions).

        Args:
            manipulator: ManipulationContextManager for the active robot.
            trajectory: The planned robot trajectory to execute.
        """
        event = self._pursuit_event
        if self._corner_speed_fraction is not None:
            event = "CORNER_EASE"
        for _ in range(self._num_repetitions):
            if event is not None:
                self.log(f"{event}_START")
            await manipulator.move(trajectory)
            if event is not None:
                self.log(f"{event}_END")

    def _static_targets(self, center: PoseStamped) -> list[PoseStamped]:
        """Cover three levels on each axis, preferring short transitions."""
        half_span = self._static_half_span_m
        offsets = list(product((-half_span, 0.0, half_span), repeat=3))
        # Shuffle once for reproducible tie-breaking between short moves.
        np.random.default_rng(27).shuffle(offsets)
        location = (0.0, 0.0, 0.0)
        origin = center.pose.position
        targets = []
        while offsets:
            index = min(
                range(len(offsets)),
                key=lambda i: (
                    math.inf
                    if not targets and offsets[i] == location
                    else math.dist(location, offsets[i])
                ),
            )
            location = offsets.pop(index)
            x, y, z = location
            targets.append(
                pose_stamped_msg(
                    frame_id=center.header.frame_id,
                    position=[origin.x + x, origin.y + y, origin.z + z],
                    orientation=center.pose.orientation,
                )
            )
        return targets

    async def _plan_static_block(
        self,
        manipulator: ManipulationContextManager,
        center: PoseStamped,
        pursuit: RobotTrajectory,
    ) -> list[tuple[PoseStamped, RobotTrajectory]]:
        """Plan every hold transition in the loaded scene before reveal."""
        start = pursuit[len(pursuit) - 1]
        plan = []
        for number, goal in enumerate(self._static_targets(center), start=1):
            try:
                move = await manipulator.plan(
                    goal=goal,
                    group_name=self._robot_name,
                    start_state=start,
                    planning_pipeline="linear",
                    velocity_scaling_factor=self._velocity_scaling_factor,
                    acceleration_scaling_factor=0.10,
                    use_cache=False,
                )
            except PlanningError as exc:
                p = goal.pose.position
                raise RuntimeError(
                    f"Could not plan static fixation {number}/27 at "
                    f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m; "
                    "not starting the calibration block"
                ) from exc
            plan.append((goal, move))
            start = move[len(move) - 1]
        self.log(f"Planned all {len(plan)} static fixation targets")
        return plan

    async def _execute_static_block(
        self,
        manipulator: ManipulationContextManager,
        plan: list[tuple[PoseStamped, RobotTrajectory]],
    ) -> None:
        for number, (goal, move) in enumerate(plan, start=1):
            await manipulator.move(move)
            await self._hold_static_target(number, goal)

    async def _hold_static_target(self, number: int, goal: PoseStamped) -> None:
        assert self._static_hold_seconds is not None
        p = goal.pose.position
        self.log(
            f"STATIC_HOLD_START point={number} frame={goal.header.frame_id} "
            f"tcp_xyz_m=({p.x:.4f},{p.y:.4f},{p.z:.4f})"
        )
        await asyncio.sleep(self._static_hold_seconds)
        self.log(f"STATIC_HOLD_END point={number}")

    def _timed_grid_targets(self, center: PoseStamped) -> Iterator[PoseStamped]:
        """Interleave eight 3x3x3 sweeps of a uniform 6x6x6 lattice."""
        levels = np.linspace(-self._static_half_span_m, self._static_half_span_m, 6)
        # Visit complementary subgrids together to balance coverage early.
        parity = tuple(product(range(2), repeat=3))
        origin = center.pose.position
        last = (0.0, 0.0, 0.0)
        while True:
            for i in (0, 7, 1, 6, 2, 5, 3, 4):
                axes = [levels[p::2] for p in parity[i]]
                for axis, initial in enumerate(last):
                    if abs(axes[axis][-1] - initial) < abs(axes[axis][0] - initial):
                        axes[axis] = axes[axis][::-1]
                xs, ys, zs = axes
                for k, z in enumerate(zs):
                    for j, y in enumerate(ys if k % 2 == 0 else ys[::-1]):
                        for x in xs if (k * len(ys) + j) % 2 == 0 else xs[::-1]:
                            last = (x, y, z)
                            yield pose_stamped_msg(
                                frame_id=center.header.frame_id,
                                position=[origin.x + x, origin.y + y, origin.z + z],
                                orientation=center.pose.orientation,
                            )

    async def _run_timed_static_grid(
        self, manipulator: ManipulationContextManager, approach_options: Mapping[str, Any]
    ) -> None:
        budget = self._static_block_seconds
        hold = self._static_hold_seconds
        assert budget is not None and hold is not None
        center = self._motion_kwargs["center_pose"]
        if not isinstance(center, PoseStamped):
            center = pose_stamped_msg(**center)
        goals = self._timed_grid_targets(center)
        first = next(goals)
        # Enter the known pursuit center before moving to the outer grid.
        # This seeds the Cartesian grid on the same wrist/IK branch as the
        # existing pursuit task instead of asking for IK directly at a corner.
        approaches: list[RobotTrajectory] = []
        try:
            approach = await manipulator.plan(
                goal=center, group_name=self._robot_name, **approach_options
            )
        except PlanningError:
            # A direct goal can select an IK branch on the wrong side of the
            # virtual region. Seed from the existing named presentation pose
            # if needed, still with the fixture and the same collision checks.
            self.log(
                "Direct grid-center approach failed; planning via presentation waypoint"
            )
            waypoint = await manipulator.plan(
                goal="present", group_name=self._robot_name, **approach_options
            )
            approaches.append(waypoint)
            approach = await manipulator.plan(
                goal=center,
                group_name=self._robot_name,
                start_state=waypoint[len(waypoint) - 1],
                **approach_options,
            )
        approaches.append(approach)
        entry = await manipulator.plan(
            goal=first,
            group_name=self._robot_name,
            start_state=approach[len(approach) - 1],
            planning_pipeline="linear",
            **approach_options,
        )
        start = entry[len(entry) - 1]
        plan: list[tuple[PoseStamped, RobotTrajectory, float]] = []
        planned_seconds = hold
        for number, goal in enumerate(goals, start=2):
            if planned_seconds + hold >= budget:
                break
            try:
                move = await manipulator.plan(
                    goal=goal,
                    group_name=self._robot_name,
                    start_state=start,
                    planning_pipeline="linear",
                    velocity_scaling_factor=self._velocity_scaling_factor,
                    acceleration_scaling_factor=0.10,
                    use_cache=False,
                )
            except PlanningError as exc:
                raise RuntimeError(
                    f"Could not preplan static grid target {number}; no grid shown"
                ) from exc
            duration = float(move.duration)
            if not math.isfinite(duration) or duration <= 0:
                raise RuntimeError(f"Invalid move duration for static grid target {number}")
            if planned_seconds + duration + hold > budget:
                break
            plan.append((goal, move, duration))
            planned_seconds += duration + hold
            start = move[len(move) - 1]
        if len(plan) + 1 < 27:
            raise RuntimeError("Static grid budget cannot fit one 27-point coverage sweep")
        self.log(f"Preplanned {len(plan) + 1} static grid targets ({planned_seconds:.2f}s)")

        for trajectory in approaches:
            await manipulator.move(trajectory)
        await manipulator.move(entry)
        try:
            await self.commander.reveal_smartglass()
            started = monotonic()
            self.log(
                f"STATIC_GRID_BLOCK_START budget_s={budget:.3f} "
                f"hold_s={hold:.3f} planned_points={len(plan) + 1}"
            )
            await self._hold_static_target(1, first)
            completed = 1
            for number, (goal, move, duration) in enumerate(plan, start=2):
                if monotonic() - started + duration + hold > budget:
                    break
                # Use the existing presentation safety gate. A safety pause
                # can extend this move; don't cut it off or shorten the hold.
                await manipulator.move(move)
                await self._hold_static_target(number, goal)
                completed = number
            self.log(
                f"STATIC_GRID_BLOCK_END points={completed} "
                f"elapsed_s={monotonic() - started:.3f}"
            )
        finally:
            await self.commander.occlude_smartglass()

    async def _return_fixture_to_fetched(
        self,
        manipulator: ManipulationContextManager,
        approach_options: Mapping[str, Any],
    ) -> None:
        """Park the attached fixture after normal completion, with the glass opaque."""
        self.log("GAZE_RETURN_START goal=fetched")
        try:
            await manipulator.plan_and_move(
                goal="fetched", cache_trajectories=False, **approach_options
            )
        except PlanningError as exc:
            if _contains_start_state_collision(exc):
                raise
            # Acquisition is already complete. A rejected parking route
            # must not trigger an alternate automatic move to idle.
            self.log(
                f"GAZE_RETURN_SKIPPED goal=fetched: no valid return plan ({exc})",
                severity="WARN",
            )
            return
        self.log("GAZE_RETURN_END goal=fetched")

    def _get_allowed_collisions(self) -> list[tuple[str, str]]:
        """Get list of allowed robot-region collisions for smooth pursuit.

        Returns collision pairs that should be allowed during smooth pursuit
        execution: robot links (including manipulated object) against
        presentation walls and dividers.

        Returns:
            List of (object1, object2) collision pairs to allow.
        """
        robot_collision_ids = [
            "right_base_link_inertia",
            "right_shoulder_link",
            "right_upper_arm_link",
            "right_forearm_link",
            "right_wrist_1_link",
            "right_wrist_2_link",
            "right_wrist_3_link",
            "right_eef_link",
            "right_eef_sphere",
            self._object_id,
        ]
        region_collision_ids = [
            "robot_divider",
            "robot_divider_front",
            "left_presentation_wall",
            "right_presentation_wall",
        ]
        return [
            (x, y) for x in robot_collision_ids for y in region_collision_ids
        ]

    async def run(self):
        """Run the smooth pursuit task.

        Executes the following sequence:
        1. Move to the trajectory start position
        2. Plan the full concatenated trajectory
        3. Reveal the smartglass to the subject
        4. Concurrently execute the trajectory and monitor eye tracking
        5. After normal completion, occlude the glass and return to the fetched pose

        The trajectory is executed for the specified number of repetitions.
        Eye tracking rewards are delivered by smooth_pursuit_and_reward().
        """
        self.log("Starting smooth pursuit task")

        # Occlude smartglass before running
        await self.commander.occlude_smartglass()

        async with self.commander.presentation_manipulation_context(
            self._robot_name
        ) as manipulator:
            # TODO: FIX!!!!!!!!!!
            # await manipulator.fetch_object(self._object_id)
            await manipulator.manually_attach_object(self._object_id)
            if self._robot_name == "right_manipulator":
                # Repeat startup validation with the fixture now modeled,
                # before applying the task's normal pursuit allowances.
                self.commander.validate_task_start_state(
                    allow_right_presentation_wall=True,
                    attached_object_id=self._object_id,
                )

            collisions_to_allow = self._get_allowed_collisions()
            modified_collisions = self.commander._moveit.allow_collision(
                *zip(*collisions_to_allow)
            )

            # The manually mounted fixture is already on the gripper during
            # both approach moves. Replan them at reduced limits without
            # reusing or populating the shared trajectory cache.
            approach_options = {
                "velocity_scaling_factor": 0.05,
                "acceleration_scaling_factor": 0.10,
                "use_cache": False,
            }
            try:
                await manipulator.plan_and_move(
                    goal="fetched",
                    cache_trajectories=False,
                    **approach_options,
                )
                if self._motion_type == "static_grid":
                    await self._run_timed_static_grid(manipulator, approach_options)
                    await self._return_fixture_to_fetched(manipulator, approach_options)
                    return
                assert self._motion_fn is not None
                for i in range(self._max_motion_generation_attempts):
                    goals = self._motion_fn(**self._motion_kwargs)

                    try:
                        # Plan to first waypoint using default planning pipeline
                        start_trajectory = await manipulator.plan(
                            goal=goals[0],
                            group_name=self._robot_name,
                            **approach_options,
                        )

                        # Plan the full concatenated trajectory through remaining waypoints
                        sweep_options = (
                            {
                                "apply_smoothing": True,
                                "acceleration_scaling_factor": 0.10,
                            }
                            if self._pursuit_event is not None
                            else {}
                        )
                        if self._motion_type == "bearing_depth":
                            # These limits apply to LIN generation, not just
                            # to the final TOTG/Ruckig pass.
                            sweep_options.update(_BEARING_DEPTH_LIN_OPTIONS)
                        if self._corner_speed_fraction is not None:
                            # Reducing the speed of the short LIN planning
                            # segments is separate from the final lap clock.
                            sweep_options.update(
                                planning_velocity_scaling_factor=0.20,
                                planning_acceleration_scaling_factor=0.15,
                            )
                        trajectory = await manipulator.plan(
                            goals=goals[1:],
                            group_name=self._robot_name,
                            start_state=start_trajectory[
                                len(start_trajectory) - 1
                            ],
                            velocity_scaling_factor=self._velocity_scaling_factor,
                            post_process_after_concat=self._post_process_after_concat,
                            # Depth paths already end at the center. A
                            # joint-space closing move would disturb the curve.
                            loop=self._pursuit_event is None,
                            planning_pipeline="linear",
                            use_cache=False,
                            **sweep_options,
                        )
                        break
                    except PlanningError as e:
                        self.log(
                            f"Error while planning smooth pursuit trajectory: {type(e).__name__}: {e}",
                            severity="ERROR",
                        )
                        if (
                            remaining := self._max_motion_generation_attempts
                            - i
                            - 1
                        ) > 0:
                            self.log(f"Trying again {remaining} more times")
                else:
                    raise RuntimeError(
                        f"Could not plan smooth pursuit trajectory after {self._max_motion_generation_attempts} attempts"
                    )

                trajectory = self._retime_pursuit(trajectory)
                static_plan = (
                    await self._plan_static_block(
                        manipulator, goals[0], trajectory
                    )
                    if self._static_hold_seconds is not None
                    else None
                )

                # Move to first waypoint
                await manipulator.move(start_trajectory)

                # Make stimulus visible to subject
                await self.commander.reveal_smartglass()

                async with asyncio.TaskGroup() as tg:
                    smooth_pursuit_task = tg.create_task(
                        self.commander.smooth_pursuit_and_reward()
                    )

                    # Wait before moving to get a baseline for no eye movement
                    wait_time = 5
                    self.log(
                        f"Waiting {wait_time} seconds before moving to get baseline for no smooth pursuit"
                    )
                    await asyncio.sleep(wait_time)

                    # Run trajectory execution and eye tracking concurrently
                    execution_task = tg.create_task(
                        self.execute_loop(manipulator, trajectory)
                    )

                    # Wait for either task to complete, then cancel the other
                    finished, _ = await asyncio.wait(
                        [smooth_pursuit_task, execution_task],
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    smooth_pursuit_task.cancel()
                    if execution_task not in finished:
                        execution_task.cancel()

                if execution_task not in finished or execution_task.cancelled():
                    if static_plan is not None:
                        raise RuntimeError(
                            "Pursuit lap did not finish; static block skipped"
                        )
                    return
                if static_plan is not None:
                    # Gaze monitoring and reward have finished their cleanup.
                    # Leave the glass transparent and start the first move.
                    await self._execute_static_block(manipulator, static_plan)
                await self.commander.occlude_smartglass()
                await self._return_fixture_to_fetched(manipulator, approach_options)
            finally:
                if len(modified_collisions) > 0:
                    self.commander._moveit.disallow_collision(
                        *zip(*modified_collisions)
                    )

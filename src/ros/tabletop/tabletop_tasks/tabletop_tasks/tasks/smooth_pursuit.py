"""Smooth pursuit eye tracking task.

This module provides a task for smooth pursuit eye tracking experiments.
The robot moves an object along predefined trajectories (spiral or random)
while tracking eye movements and providing rewards for successful tracking.

The task supports two motion types:
- Spiral: Helical trajectory around a center point
- Random: Random waypoints within a bounding box

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
from collections.abc import Mapping
from typing import Any, Literal

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from tabletop_rig.exceptions import PlanningError
from tabletop_rig.nodes import Commander
from tabletop_rig.nodes.commander import ManipulationContextManager
from tabletop_rig.utils.ros import pose_stamped_msg

from tabletop_tasks.tasks.base import BaseTask


class SmoothPursuitTask(BaseTask):
    """Task for smooth pursuit eye tracking experiments.

    Moves an attached object along a predefined trajectory while
    monitoring eye tracking. Supports spiral and random motion patterns.
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
        motion_type: Literal["spiral", "sin", "random"],
        motion_kwargs: Mapping[str, Any],
        num_repetitions: int,
        object_id: str,
        robot_name: str,
        velocity_scaling_factor: float = 1.0,
        max_motion_generation_attempts: int = 5,
    ):
        """Initialize the smooth pursuit task.

        Args:
            commander: Commander instance for robot interaction.
            motion_type: Type of trajectory to generate ("spiral", "sin",
                or "random").
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

        Raises:
            ValueError: If motion_type is not "spiral", "sin", or "random".
        """
        super().__init__("smooth_pursuit_task", commander)
        match motion_type:
            case "spiral":
                self._post_process_after_concat = True
                self._motion_fn = self.generate_spiral
            case "sin":
                self._post_process_after_concat = True
                self._motion_fn = self.generate_sin
            case "random":
                self._post_process_after_concat = False
                self._motion_fn = self.generate_random
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
        for _ in range(self._num_repetitions):
            await manipulator.move(trajectory)

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
        5. Stop when either trajectory completes or tracking ends

        The trajectory is executed for the specified number of repetitions.
        Eye tracking rewards are delivered by smooth_pursuit_and_reward().
        """
        self.log("Starting smooth pursuit task")

        # Occlude smartglass before running
        await self.commander.occlude_smartglass()

        async with self.commander.manipulation_context(
            self._robot_name
        ) as manipulator:
            # TODO: FIX!!!!!!!!!!
            # await manipulator.fetch_object(self._object_id)
            await manipulator.manually_attach_object(self._object_id)

            collisions_to_allow = self._get_allowed_collisions()
            modified_collisions = self.commander._moveit.allow_collision(
                *zip(*collisions_to_allow)
            )

            # TODO: Maybe remove
            await manipulator.plan_and_move(goal="fetched")

            try:
                for i in range(self._max_motion_generation_attempts):
                    goals = self._motion_fn(**self._motion_kwargs)

                    try:
                        # Plan to first waypoint using default planning pipeline
                        start_trajectory = await manipulator.plan(
                            goal=goals[0],
                            group_name=self._robot_name,
                            use_cache=False,
                        )

                        # Plan the full concatenated trajectory through remaining waypoints
                        trajectory = await manipulator.plan(
                            goals=goals[1:],
                            group_name=self._robot_name,
                            start_state=start_trajectory[
                                len(start_trajectory) - 1
                            ],
                            velocity_scaling_factor=self._velocity_scaling_factor,
                            post_process_after_concat=self._post_process_after_concat,
                            loop=True,
                            planning_pipeline="linear",
                            use_cache=False,
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
                    await asyncio.wait(
                        [smooth_pursuit_task, execution_task],
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    smooth_pursuit_task.cancel()
                    execution_task.cancel()
            finally:
                if len(modified_collisions) > 0:
                    self.commander._moveit.disallow_collision(
                        *zip(*modified_collisions)
                    )

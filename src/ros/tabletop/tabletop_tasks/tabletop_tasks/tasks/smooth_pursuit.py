import asyncio
import time
from collections.abc import Mapping
from typing import Any, Literal

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from tabletop_rig.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.trial_generators.base import TrialFeedback, TrialSpec


class SmoothPursuitTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        *,
        motion_type: Literal["spiral", "random"],
        motion_kwargs: Mapping[str, Any],
        num_repetitions: int,
        object_id: str,
        velocity_scaling_factor: float = 1.0,
    ):
        super().__init__(commander, logger_name="smooth_pursuit_task")
        if motion_type == "spiral":
            self._goals = self.generate_spiral(**motion_kwargs)
            self._post_process_after_concat = True
        elif motion_type == "random":
            self._goals = self.generate_random(**motion_kwargs)
            self._post_process_after_concat = False
        else:
            raise ValueError(f"Unsupported motion type: {motion_type}")

        self._num_repetitions = num_repetitions
        self._velocity_scaling_factor = velocity_scaling_factor

        self.commander.attach_object_manually(object_id)

    def generate_spiral(
        self,
        center_pose_kwargs: Mapping[str, Any],
        radius: float,
        length: float,
        num_revolutions: int,
        num_segments: int,
    ) -> list[PoseStamped]:
        """Generate spiral trajectory

        Args:
            center_pose_kwargs: TODO
            radius: TODO
            length: TODO
            num_revolutions: TODO
            num_segments: TODO

        Returns:
            goals: List of PoseStamped messages to pass to Commander.plan()
        """
        self.log("Generating spiral trajectory")

        center = self.commander.create_pose_stamped(**center_pose_kwargs)

        goals: list[PoseStamped] = []

        for i in range(num_segments + 1):
            theta_xz = (2 * np.pi * i * num_revolutions) / num_segments
            theta_y = (2 * np.pi * i) / num_segments

            x = center.pose.position.x + radius * np.cos(theta_xz)
            y = center.pose.position.y - (length / 2) * np.cos(theta_y)
            z = center.pose.position.z + radius * np.sin(theta_xz)

            goal = self.commander.create_pose_stamped(
                position=[x, y, z],
                orientation=center.pose.orientation,
            )
            goals.append(goal)

        return goals

    def generate_random(
        self,
        start_pose_kwargs: Mapping[str, Any],
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        min_z: float,
        max_z: float,
        num_waypoints: int,
    ) -> list[PoseStamped]:
        """Generate random line trajectory

        Args:
            center_pose_kwargs: TODO
            radius: TODO
            length: TODO
            num_revolutions: TODO
            num_segments: TODO

        Returns:
            goals: List of PoseStamped messages to pass to Commander.plan()
        """
        self.log("Generating random trajectory")

        start = self.commander.create_pose_stamped(**start_pose_kwargs)

        goals: list[PoseStamped] = []
        goals.append(start)
        # np.random.seed(0)

        for i in range(num_waypoints):
            x = (max_x - min_x) * np.random.random_sample() + min_x
            y = (max_y - min_y) * np.random.random_sample() + min_y
            z = (max_z - min_z) * np.random.random_sample() + min_z

            print(f"Goal {i}: {x}, {y}, {z}")

            goal = self.commander.create_pose_stamped(
                position=[x, y, z],
                orientation=start.pose.orientation,
            )
            goals.append(goal)

        return goals

    async def execute_loop(self, trajectory: RobotTrajectory):
        for _ in range(self._num_repetitions):
            await self.commander.execute(trajectory)

    async def run_trial(
        self, trial_spec: TrialSpec | None
    ) -> TrialFeedback | None:
        pass

    async def run(self):
        self.log("Starting smooth pursuit task")
        async with self.commander:
            # Plan to start location
            await self.commander.plan_and_execute(goal=self._goals[0])

            # Plan concatenated trajectory
            start = time.time()
            trajectory = await self.commander.plan(
                goals=self._goals[1:],
                velocity_scaling_factor=self._velocity_scaling_factor,
                post_process_after_concat=self._post_process_after_concat,
                loop=True,
                planning_pipeline="linear",
                use_cache=False,
            )
            self.log(f"Time Taken: {time.time() - start}")
            assert trajectory is not None

            # Reveal smartglass
            await self.commander.reveal_smartglass()

            # Schedule smooth pursuit and execution tasks
            smooth_pursuit_task = asyncio.create_task(
                self.commander.smooth_pursuit_and_reward()
            )
            execution_task = asyncio.create_task(self.execute_loop(trajectory))

            # Wait for tasks to finish, cancelling remaining tasks when one
            # completes
            done, pending = await asyncio.wait(
                [smooth_pursuit_task, execution_task],
                return_when=asyncio.FIRST_COMPLETED,
            )

            for task in pending:
                task.cancel()

            for task in done:
                task.result()

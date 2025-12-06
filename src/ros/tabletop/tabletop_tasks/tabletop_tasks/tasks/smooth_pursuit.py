import asyncio
from collections.abc import Mapping
from typing import Any

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from tabletop_rig.interfaces.moveit.requests import PlanGoalT
from tabletop_rig.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.trial_generators.base import TrialFeedback, TrialSpec


class SmoothPursuitTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        *,
        center_pose: Mapping[str, Any],
        radius: float,
        length: float,
        num_revolutions: int,
        num_segments: int,
        num_cycles: int,
        reward_period: float,
        reward_duration: float,
        object_id: str,
        execute_request_params: Mapping[str, Any],
    ):
        super().__init__(commander, logger_name="smooth_pursuit_task")
        self._center_pose = self.commander.create_pose_stamped(**center_pose)
        self._radius = radius
        self._length = length
        self._num_revolutions = num_revolutions
        self._num_segments = num_segments
        self._num_cycles = num_cycles
        self._reward_period = reward_period
        self._reward_duration = reward_duration
        self._execute_request_params = execute_request_params

        self.commander.attach_object_manually(object_id)

    def generate_goals(self) -> list[PoseStamped]:
        """Generate spiral trajectory

        Returns:
            pre_trajectory: Trajectory to move to the start of the spiral
            spiral_trajectory: Spiral trajectory
        """
        self.log(f"Generating {self._num_revolutions} revolutions")

        goals: list[PoseStamped] = []
        for i in range(self._num_segments + 1):
            # Calculate x and z coordinates of spiral (circular motion)
            theta_xz = (
                2 * np.pi * i * self._num_revolutions
            ) / self._num_segments
            x = self._center_pose.pose.position.x + self._radius * np.cos(
                theta_xz
            )
            z = self._center_pose.pose.position.z + self._radius * np.sin(
                theta_xz
            )
            # Calculate y coordinate of spiral (forward-backward motion)
            theta_y = (2 * np.pi * i) / self._num_segments
            y = self._center_pose.pose.position.y - (
                self._length / 2
            ) * np.cos(theta_y)
            goal = self.commander.create_pose_stamped(
                position=[x, y, z],
                orientation=self._center_pose.pose.orientation,
            )
            goals.append(goal)

        return goals

    async def execute_loop(self, trajectory: RobotTrajectory):
        for _ in range(self._num_cycles):
            await self.commander.execute(trajectory)

    async def execute_multiple(self, trajectories: list[RobotTrajectory]):
        for _ in range(self._num_cycles):
            await self.commander.execute(trajectories)

    async def run_trial(
        self, trial_spec: TrialSpec | None
    ) -> TrialFeedback | None:
        pass

    async def run(self):
        self.log("Starting smooth pursuit task")
        async with self.commander:
            # await self.commander.smooth_pursuit_and_reward()
            goals: list[PlanGoalT] = []
            goals.extend(self.generate_goals())

            start = goals.pop(0)
            await self.commander.plan_and_execute(start)

            # Concat
            trajectory = await self.commander.plan(
                goals=goals, loop=True, post_process_after_concat=True
            )
            assert trajectory is not None

            smooth_pursuit_task = asyncio.create_task(
                self.commander.smooth_pursuit_and_reward()
            )

            execution_task = asyncio.create_task(self.execute_loop(trajectory))

            done, pending = await asyncio.wait(
                [smooth_pursuit_task, execution_task],
                return_when=asyncio.FIRST_COMPLETED,
            )

            for task in pending:
                task.cancel()

            for task in done:
                task.result()

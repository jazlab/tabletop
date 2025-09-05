"""SmoothPursuit task."""

import asyncio
from collections.abc import Mapping
from typing import Any

import numpy as np
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore

from tabletop_server.nodes import Commander
from tabletop_tasks.tasks.base import BaseTask
from tabletop_utils.ros import robot_trajectory_copy


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
        super().__init__(commander)
        self._center_pose = self.commander.create_pose_stamped(**center_pose)
        self._radius = radius
        self._length = length
        self._num_revolutions = num_revolutions
        self._num_segments = num_segments
        self._num_cycles = num_cycles
        self._reward_period = reward_period
        self._reward_duration = reward_duration
        self._execute_request_params = execute_request_params

        self.commander.add_manually_attached_collision_object(object_id)

    async def generate_trajectory(self) -> RobotTrajectory:
        """Generate spiral trajectory

        Returns:
            pre_trajectory: Trajectory to move to the start of the spiral
            spiral_trajectory: Spiral trajectory
        """
        self.log(f"Generating {self._num_revolutions} revolutions")

        last_waypoint: RobotState | None = None
        for i in range(self._num_segments + 1):
            self.log(f"Generating segment {i} of {self._num_segments}")
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
            waypoint_pose_stamped = self.commander.create_pose_stamped(
                position=[x, y, z],
                orientation=self._center_pose.pose.orientation,
            )

            if i == 0:
                # Plan to start of spiral
                trajectory = await self.commander.plan(
                    goal=waypoint_pose_stamped
                )
                if trajectory is None:
                    self.log(
                        "Initial robot pose close enough to first waypoint, skipping planning",
                        severity="INFO",
                    )
                    last_waypoint = self.commander.current_state
                    continue
            else:
                # Plan segment of spiral
                trajectory = await self.commander.plan(
                    goal=waypoint_pose_stamped,
                    start_state=last_waypoint,
                    planning_pipeline="linear",
                )
                if trajectory is None:
                    self.log(
                        "Waypoints may be too close, skipping segment",
                        severity="WARN",
                    )
                    continue

                if i == 1:
                    spiral_trajectory = trajectory
                else:
                    spiral_trajectory.append(
                        trajectory, dt=0.01, start_index=1
                    )

            last_waypoint = trajectory[len(trajectory) - 1]

        request, unused_kwargs = self.commander.create_execute_request(
            spiral_trajectory, **self._execute_request_params
        )
        if unused_kwargs:
            raise ValueError(
                f"Smooth pursuit execute request has unused kwargs: {unused_kwargs}"
            )
        spiral_trajectory = self.commander.preprocess_trajectory(request)

        full_trajectory = robot_trajectory_copy(spiral_trajectory)
        for i in range(self._num_cycles):
            full_trajectory.append(spiral_trajectory, dt=0.0001, start_index=1)

        return full_trajectory

    async def run(self) -> None:
        self.log("Starting smooth pursuit task")
        async with self.commander:
            # await self.commander.smooth_pursuit_and_reward()
            trajectory = await self.generate_trajectory()

            self.log("Moving to start of spiral")
            await self.commander.plan_and_execute(trajectory[0])

            self.log(
                f"Executing spiral trajectory for {trajectory.duration} seconds"
            )

            smooth_pursuit_task = self.commander.smooth_pursuit_and_reward()
            execution_task = asyncio.create_task(
                self.commander.execute(trajectory, preprocess_trajectory=False)
            )

            done, pending = await asyncio.wait(
                [smooth_pursuit_task, execution_task],
                return_when=asyncio.FIRST_COMPLETED,
            )

            for task in pending:
                task.cancel()

            for task in done:
                task.result()

"""SmoothPursuit task."""

from collections.abc import Mapping
from typing import Any

import numpy as np
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore
from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask


class SmoothPursuitTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        center_pose: Mapping[str, Any],
        radius: float,
        length: float,
        num_revolutions: int,
        num_segments: int = 100,
        velocity_scaling_factor: float = 0.5,
        acceleration_scaling_factor: float = 0.5,
        reward_period: float = 1.0,
        reward_duration: float = 0.1,
        num_cycles: int = 1,
    ):
        super().__init__(commander)
        self._center_pose = self.commander.create_pose_stamped(**center_pose)
        self._radius = radius
        self._length = length
        self._num_revolutions = num_revolutions
        self._num_segments = num_segments
        self._reward_period = reward_period
        self._reward_duration = reward_duration
        self._velocity_scaling_factor = velocity_scaling_factor
        self._acceleration_scaling_factor = acceleration_scaling_factor
        self._num_cycles = num_cycles

    async def generate_trajectories(self) -> RobotTrajectory:
        """Generate spiral trajectory

        Returns:
            pre_trajectory: Trajectory to move to the start of the spiral
            spiral_trajectory: Spiral trajectory
        """
        self.log(f"Generating {self._num_revolutions} revolutions")

        last_waypoint: RobotState | None = None
        for i in range(self._num_segments):
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
            y = self._center_pose.pose.position.y + (
                self._length / 2
            ) * np.sin(theta_y)
            waypoint_pose_stamped = self.commander.create_pose_stamped(
                position=[x, y, z],
                orientation=self._center_pose.pose.orientation,
            )

            if i == 0:
                # Plan to start of spiral
                response = await self.commander.plan(
                    goal=waypoint_pose_stamped
                )
            else:
                # Plan segment of spiral
                response = await self.commander.plan(
                    goal=waypoint_pose_stamped,
                    start_state=last_waypoint,
                    planning_pipeline="linear",
                )
                if i == 1:
                    spiral_trajectory: RobotTrajectory = response.trajectory
                else:
                    spiral_trajectory.append(
                        response.trajectory, dt=0.01, start_index=1
                    )

            last_waypoint = response.trajectory[len(response.trajectory) - 1]

        if not spiral_trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=self._velocity_scaling_factor,
            acceleration_scaling_factor=self._acceleration_scaling_factor,
        ):
            raise RuntimeError("Failed to apply time parameterization")
        # trajectory.apply_ruckig_smoothing()
        return spiral_trajectory

    async def run(self) -> None:
        self.log("Starting smooth pursuit task")
        async with self.commander.context_manager():
            spiral_trajectory = await self.generate_trajectories()
            for i in range(self._num_cycles):
                self.log(f"Executing cycle {i} of {self._num_cycles}")
                self.log("Executing pre-trajectory")
                await self.commander.plan_and_execute(spiral_trajectory[0])
                self.log("Executing spiral trajectory")
                await self.commander.execute(spiral_trajectory)

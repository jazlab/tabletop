"""SmoothPursuit task."""

import asyncio
import time
from typing import List

import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask


class SmoothPursuit(BaseTask):
    def __init__(
        self,
        commander: Commander,
        center_pose: Pose,
        radius: float,
        total_time: float,
        reward_period_s: float = 1.0,
        reward_duration_s: float = 0.1,
        loop_period: float = 0.01,
    ):
        super().__init__(commander)
        self._total_time = total_time
        self._reward_period_s = reward_period_s
        self._reward_duration_s = reward_duration_s
        self._loop_period = loop_period

        # Make circular path
        self._path: List[PoseStamped] = []
        for i in range(100):
            theta = 2 * np.pi * i / 100
            x = center_pose.position.x + radius * np.cos(theta)
            y = center_pose.position.y + radius * np.sin(theta)
            z = center_pose.position.z
            waypoint = PoseStamped(
                header=Header(frame_id="world"),
                pose=Pose(
                    position=Point(x=x, y=y, z=z),
                    orientation=center_pose.orientation,
                ),
            )
            self._path.append(waypoint)

    async def run(self) -> None:
        start_time = time.time()
        last_reward_time = start_time

        try:
            plan_result = self._commander.plan(self._path[0])
        except Exception as e:
            self._commander.log(f"Failed to plan path: {e}", severity="ERROR")

        future = asyncio.create_task(
            self._commander.execute_async(plan_result.trajectory)
        )

        while time.time() - start_time < self._total_time:
            # TODO: Reward logic
            if future.done():
                await future
                future = asyncio.create_task(
                    self._commander.execute_async(plan_result.trajectory)
                )
            else:
                if time.time() - last_reward_time > self._reward_period_s:
                    await self._commander.reward_async(self._reward_duration_s)
                    last_reward_time = time.time()

            await asyncio.sleep(self._loop_period)

        await future

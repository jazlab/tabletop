"""SmoothPursuit task."""

import abc
import asyncio
import time

import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from tabletop_server.nodes.commander import Commander
from tabletop_server.utils import quaternion_from_euler


class SmoothPursuit(abc.ABC):
    def __init__(
        self,
        commander: Commander,
        center_pose: Pose,
        radius: float,
        total_time: float,
        reward_period: float = 1.0,
        reward_duration: float = 0.1,
        loop_period: float = 0.01,
    ):
        self._commander = commander
        self._total_time = total_time
        self._reward_period = reward_period
        self._reward_duration = reward_duration
        self._loop_period = loop_period

        # Make circular path
        self._path = []
        center_x = center_pose.position.x
        center_y = center_pose.position.y
        center_z = center_pose.position.z
        for i in range(100):
            theta = 2 * np.pi * i / 100
            x = center_x + radius * np.cos(theta)
            y = center_y + radius * np.sin(theta)
            z = center_z
            self._path.append(
                PoseStamped(
                    header=Header(frame_id="world"),
                    pose=Pose(
                        position=Point(x=x, y=y, z=z),
                        orientation=quaternion_from_euler(0.0, 0.0, 0.0),
                    ),
                )
            )

    async def run(self):
        start_time = time.time()
        last_reward_time = start_time

        try:
            plan_result = self._commander.plan(self._path)
        except Exception as e:
            self._commander.log(f"Failed to plan path: {e}", severity="ERROR")

        future = self._commander.execute_async(plan_result.trajectory)

        while time.time() - start_time < self._total_time:
            # TODO: Reward logic
            if future.done():
                # Raise exception if execution failed
                future.result()

                future = self._commander.execute_async(plan_result.trajectory)
            else:
                if time.time() - last_reward_time > self._reward_period:
                    self._commander.reward(self._reward_duration)
                    last_reward_time = time.time()

            await asyncio.sleep(self._loop_period)

        await future

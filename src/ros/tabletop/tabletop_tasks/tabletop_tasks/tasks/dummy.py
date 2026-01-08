"""Dummy task for testing and placeholder purposes.

This module provides a minimal task implementation that does nothing
but keep the commander context alive. Useful for testing the task
infrastructure or as a placeholder during development.

Example:
    task = DummyTask(commander)
    await task.run()  # Runs indefinitely, sleeping each second
"""

import asyncio
import random

import numpy as np
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.ros import (
    arrays_from_pose_msg,
    change_reference_frame_pose,
    matrix_from_pose_msg,
    pose_msg,
)

from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.trial_generators.base import (
    TrialFeedback,
    TrialSpec,
)


class DummyTask(BaseTask):
    """Minimal placeholder task that runs indefinitely.

    This task maintains an active commander context while doing no
    actual work. It's useful for:
    - Testing the task infrastructure
    - Keeping robot connections alive during debugging
    - Serving as a template for new task implementations

    Unlike other tasks, DummyTask does not use a trial generator
    and overrides run() to provide its own infinite loop.
    """

    def __init__(self, commander: Commander) -> None:
        """Initialize the dummy task.

        Args:
            commander: Commander instance for robot interaction.
        """
        super().__init__(commander, logger_name="dummy_task")

    async def run_trial(
        self, trial_spec: TrialSpec | None
    ) -> TrialFeedback | None:
        """Not implemented for dummy task.

        This method exists only to satisfy the abstract base class
        requirement. DummyTask overrides run() directly.

        Args:
            trial_spec: Unused trial specification.

        Returns:
            None (never called).
        """
        pass

    async def run0(self) -> None:
        """Test method for debugging object grid positioning.

        Continuously logs the end-effector position relative to the
        grid origin. Useful for calibrating object placement.
        """
        grid_origin_kwargs = self.commander.param(
            "planning_scene.object_meshes.grid_origin"
        )
        grid_origin = pose_msg(**grid_origin_kwargs)
        grid_origin_matrix = matrix_from_pose_msg(grid_origin)
        position, euler = arrays_from_pose_msg(grid_origin, euler=True)
        self.commander.log(
            f"Object grid origin position: {position.round(4)}, euler: {euler.round(4)}"
        )

        while True:
            pose_stamped = self.commander.moveit.get_link_pose_stamped(
                self.commander.moveit.default_pose_link
            )
            old_frame_transform = self.commander.moveit.get_frame_transform(
                pose_stamped.header.frame_id
            )
            rel_pose = change_reference_frame_pose(
                old_pose=pose_stamped.pose,
                old_frame_transform=old_frame_transform,
                new_frame_transform=grid_origin_matrix,
            )
            position, euler = arrays_from_pose_msg(rel_pose, euler=True)
            self.commander.log(
                f"Eef relative position: {position.round(4).tolist()}, euler: {euler.round(4).tolist()}"
            )
            await asyncio.sleep(1.0)

    async def run(self) -> None:
        """Test Flic button response times across multiple objects.

        Iterates through small objects 15-29 and measures Flic button
        response times, computing average and standard deviation.
        """
        flic_rts = []
        total_rts = []
        for idx in range(15, 30):
            bd_addr = self.commander.param(f"flic.bd_addrs.small_object_{idx}")
            start_time = self.commander.ros_time()
            flic_rt = await self.commander.flic.response_time(bd_addr)
            total_rt = self.commander.ros_time() - start_time
            self.log(f"Reported: {flic_rt:.4f}s | Total: {total_rt}s")
            if idx != 15:
                flic_rts.append(flic_rt)
                total_rts.append(total_rt)

        flic_avg = np.mean(flic_rts)
        flic_std = np.std(flic_rts)
        total_avg = np.mean(total_rts)
        total_std = np.std(total_rts)
        self.log(f"Reported avg: {flic_avg:.4f}s, std: {flic_std:.6f}")
        self.log(f"Total avg: {total_avg:.4f}s, std: {total_std:.6f}")

    async def run2(self) -> None:
        """Test Flic response times with smartglass occlusion.

        Tests the full trial sequence with smartglass occlusion and
        reveal, measuring corrected response times that account for
        the occlusion period.
        """
        rts = []
        try:
            for idx in range(15, 30):
                bd_addr = self.commander.param(
                    f"flic.bd_addrs.small_object_{idx}"
                )
                async with asyncio.TaskGroup() as tg:
                    start_time = self.commander.ros_time()
                    flic_task = tg.create_task(
                        self.commander.flic.response_time(bd_addr)
                    )

                    await asyncio.sleep(2.0)
                    await self.commander.occlude_smartglass()

                    await asyncio.sleep(2 + 5 * random.random())
                    await self.commander.reveal_smartglass()
                    correction = self.commander.ros_time() - start_time

                    start_time = self.commander.ros_time()
                    flic_rt = await flic_task
                    total_rt = self.commander.ros_time() - start_time

                    assert flic_rt is not None

                    flic_rt = flic_rt - correction
                    self.log(f"Reported: {flic_rt:.4f}s | Total: {total_rt}")
                    rts.append(total_rt)
        finally:
            avg = sum(rts) / len(rts)
            self.log(f"Average response time: {avg}")

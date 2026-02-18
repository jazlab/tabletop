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
from typing import cast

import numpy as np
from rclpy.time import Time
from tabletop_interfaces.srv import Ping
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
        super().__init__("dummy_task", commander)

    async def run_trial(self, trial_spec: TrialSpec) -> TrialFeedback:  # pyright: ignore[reportReturnType]
        """Not implemented for dummy task.

        This method exists only to satisfy the abstract base class
        requirement. DummyTask overrides run() directly.

        Args:
            trial_spec: Unused trial specification.

        Returns:
            None (never called).
        """
        pass

    async def test_robot_position(self) -> None:
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

    async def test_teensy_latency(self):
        client = self.commander.create_client(Ping, "/teensy/ping")

        roundtrips = []
        forwards = []
        backwards = []

        for i in range(1050):
            start_time = self.commander.get_clock().now()
            response = cast(
                Ping.Response,
                await self.commander.service_call_async(
                    srv_request=Ping.Request(sent_time=start_time.to_msg()),
                    srv_client=client,
                ),
            )
            end_time = self.commander.get_clock().now()
            response = cast(Ping.Response, response)
            teensy_time = Time.from_msg(response.received_time)

            roundtrip = (end_time - start_time).nanoseconds / 1e6
            forward = (teensy_time - start_time).nanoseconds / 1e6
            backward = (end_time - teensy_time).nanoseconds / 1e6

            # Warmup period
            if i > 50:
                roundtrips.append(roundtrip)
                forwards.append(forward)
                backwards.append(backward)

        roundtrip_mean = np.mean(roundtrips)
        roundtrip_std = np.std(roundtrips)
        forward_mean = np.mean(forwards)
        forward_std = np.std(forwards)
        backward_mean = np.mean(backwards)
        backward_std = np.std(backwards)

        self.log(
            "----------------------------------------------------------\n"
            f"Teensy latency stats (N={len(roundtrips)}):\n"
            f"Roundtrip: {roundtrip_mean:.2f} ± {roundtrip_std:.2f} ms\n"
            f"Forward:   {forward_mean:.2f} ± {forward_std:.2f} ms\n"
            f"Backward:  {backward_mean:.2f} ± {backward_std:.2f} ms\n"
            "----------------------------------------------------------"
        )

    async def test_flic_latency_pre_pressed(self) -> None:
        """Test Flic button latency across multiple objects (deprecated)

        Don't use this test anymore now that auto-disconnect time has
        been set to 0 on Flic node, which I believe prevents pre-pressing
        the buttons from correctly queuing them.

        Iterates through small objects 0-29 and measures Flic button
        latencies, computing average and standard deviation.
        Assumes all buttons have been "pre-pressed" after the first one.
        This should trigger the button press immediately upon connect.
        """
        reported_latencies = []
        total_latencies = []
        overheads = []
        for i in range(0, 30):
            bd_addr = self.commander.param(f"flic.bd_addrs.small_object_{i}")

            start_time = self.commander.ros_time()
            reported_rt = await self.commander.flic.response_time(bd_addr)
            assert reported_rt is not None
            reported_latency = reported_rt - start_time
            total_latency = self.commander.ros_time() - start_time
            overhead = total_latency - reported_latency

            self.log(
                f"Reported: {reported_latency:.4f}s | "
                f"Total: {total_latency:.4f}s | "
                f"Overhead: {overhead:.4f}"
            )
            if i != 0:
                reported_latencies.append(reported_latency)
                total_latencies.append(total_latency)
                overheads.append(overhead)

        reported_latencies = np.array(reported_latencies)
        total_latencies = np.array(total_latencies)
        overheads = np.array(overheads)

        reported_avg = np.mean(reported_latencies)
        reported_std = np.std(reported_latencies)
        total_avg = np.mean(total_latencies)
        total_std = np.std(total_latencies)
        overhead_avg = np.mean(overheads)
        overhead_std = np.mean(overheads)

        self.log(
            "----------------------------------------------------------\n"
            f"Flic pre-pressed latency stats (N={len(reported_latencies)}):\n"
            f"Reported: {reported_avg:.2f} ± {reported_std:.2f} ms\n"
            f"Total:    {total_avg:.2f} ± {total_std:.2f} ms\n"
            f"Overhead: {overhead_avg:.2f} ± {overhead_std:.2f} ms\n"
            "----------------------------------------------------------"
        )

    async def test_flic_latency_human(self) -> None:
        """Test Flic button latency using human response time as ground truth

        Tests the full trial sequence with smartglass occlusion and
        reveal, measuring corrected response times that account for
        the occlusion period.
        """
        rts = []
        try:
            for idx in range(0, 30):
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
            avg = np.mean(rts)
            std = np.std(rts)
            self.log(
                f"----------------------------------------------------------\n"
                f"Flic human latency stats (N={len(rts)}):\n"
                f"Latency: {avg:.2f} ± {std:.2f} ms\n"
                f"Calculate your own reaction time "
                f"(https://humanbenchmark.com/tests/reactiontime),"
                f"then subtract this from the avg here to get the flic latency"
                f"----------------------------------------------------------"
            )

    async def test_flic_latency_button(self) -> None:
        """Test Flic button latency using the teensy button as ground truth"""
        latencies = []
        try:
            while True:
                for i in range(0, 30):
                    self.log(
                        f"Press flic button {i} and teensy button simultaneously (smash them)"
                    )
                    bd_addr = self.commander.param(
                        f"flic.bd_addrs.small_object_{i}"
                    )
                    flic_time = await self.commander.flic.response_time(
                        bd_addr
                    )
                    assert flic_time is not None

                    teensy_time_msg = self.commander.teensy.last_teensy_sensor.button_last_time_pressed
                    teensy_time = (
                        float(Time.from_msg(teensy_time_msg).nanoseconds) / 1e9
                    )

                    latency = flic_time - teensy_time
                    self.log(f"Latency: {latency:.4f}s")
                    latencies.append(latency)
        finally:
            avg = np.mean(latencies)
            std = np.std(latencies)
            self.log(
                f"----------------------------------------------------------\n"
                f"Flic latency (using teensy button as ground truth) stats (N={len(latencies)}):\n"
                f"Latency: {avg:.2f} ± {std:.2f} ms\n"
                f"----------------------------------------------------------"
            )

    async def test_optitrack_latency_solenoid(self):
        """Test using a solenoid to press the button"""
        await asyncio.sleep(1000)
        # client = self.commander.create_client(
        #     SetSolenoid, "/teensy/set_solenoid"
        # )
        # await self.commander.service_call_async(
        #     srv_request=SetSolenoid.Request(activate=True),
        #     srv_client=client,
        # )
        # try:
        #     while True:
        #         await asyncio.sleep(1.0)
        # finally:
        #     await self.commander.service_call_async(
        #         srv_request=SetSolenoid.Request(activate=False),
        #         srv_client=client,
        #     )

    async def test_smooth_pursuit(self):
        """Test the smooth pursuit action client."""
        self.log("Starting smooth pursuit task")
        async with self.commander:
            await self.commander.reveal_smartglass()
            await self.commander.smooth_pursuit_and_reward()

    async def test_sound(self):
        while True:
            await self.commander.play_sound()
            await asyncio.sleep(1)

    async def run(self) -> None:
        """Run one or more of the tests"""
        async with self.commander:
            await self.test_teensy_latency()
            # await self.test_flic_latency_pre_pressed()
            # await self.test_flic_latency_human()
            # await self.test_robot_position()
            # await self.test_flic_latency_button()
            # await self.test_optitrack_latency_solenoid()
            # await self.test_sound()
            await self.test_smooth_pursuit()

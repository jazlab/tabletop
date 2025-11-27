"""Foraging task."""

import asyncio
from collections.abc import Mapping
from typing import Any, Literal, Optional

from geometry_msgs.msg import PoseStamped
from tabletop_rig.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class ForagingTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
        stimulus_duration: float,
        delay_duration: float,
        reward_duration: float,
        reward_sound: bool,
        reveal_duration: float,
        response_timeout: float,
        sound_kwargs: Optional[Mapping[str, Any]] = None,
    ):
        super().__init__(
            commander, trial_generator, logger_name="foraging_task"
        )

        self.stimulus_duration = stimulus_duration
        self.delay_duration = delay_duration
        self.response_timeout = response_timeout
        self.reward_duration = reward_duration
        self.reward_sound = reward_sound
        self.reveal_duration = reveal_duration
        self.sound_kwargs = sound_kwargs if sound_kwargs is not None else {}

    ############################################################
    # Phases
    ############################################################

    async def present(self, pose: PoseStamped):
        """Present object."""
        self.log("Present phase")
        await self.commander.plan_and_execute(
            goal=pose, planning_pipeline="linear"
        )

    async def stimulus(self):
        """Present stimulus."""
        self.log("Stimulus phase")
        await self.commander.reveal_smartglass()
        await asyncio.sleep(self.stimulus_duration)

    async def delay(self, occlude: bool):
        """Delay phase."""
        self.log("Delay phase")
        if occlude:
            await self.commander.occlude_smartglass()
        await asyncio.sleep(self.delay_duration)

    async def response(
        self, arm: Literal["left", "right", "both"]
    ) -> TrialFeedback:
        """Response phase."""
        self.log("Response phase")
        await self.commander.release_arm(arm)

        # Wait for response
        # Use the ROS2 clock to measure reaction time (for consistency with
        # ROS2 synchronization)
        if reaction_time := await self.commander.flic_response_time(
            self.response_timeout
        ):
            # Response received
            sound_task = None
            if self.reward_sound:
                sound_task = asyncio.create_task(
                    self.commander.play_sound(**self.sound_kwargs)
                )
            reward_task = asyncio.create_task(
                self.commander.start_reward_and_wait(self.reward_duration)
            )
            self.log(f"Reaction time: {reaction_time}")
            if sound_task is not None:
                await sound_task
            await reward_task
            return TrialFeedback(reaction_time=reaction_time, timeout=False)
        else:
            # Response not received
            self.log("Response timeout")
            return TrialFeedback(reaction_time=None, timeout=True)

    async def reveal(self):
        """Reveal object."""
        self.log("Reveal phase")
        await self.commander.reveal_smartglass()
        await asyncio.sleep(self.reveal_duration)

    async def run_trial(
        self, trial_spec: TrialSpec | None
    ) -> TrialFeedback | None:
        """Run a trial."""
        if trial_spec is None:
            raise ValueError("trial_spec should not be None for foraging task")

        self.log(f"Foraging task trial spec: {trial_spec}")
        await self.present(trial_spec.object_pose)
        await self.stimulus()
        await self.delay(trial_spec.occlude)
        feedback = await self.response(trial_spec.arm)
        if not feedback.timeout:
            await self.reveal()
        return feedback

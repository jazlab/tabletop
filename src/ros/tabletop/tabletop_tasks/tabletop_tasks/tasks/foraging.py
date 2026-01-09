"""Foraging task."""

import asyncio
import random
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

    async def prepare(self, pose: PoseStamped):
        """Prepare trial by moving object to desired location."""
        self.log("Prepare phase")
        await self.commander.plan_and_execute(
            goal=pose, planning_pipeline="linear"
        )

    async def stimulus(self):
        """Reveal stimulus for stimulus by revealing smartglass"""
        self.log("Stimulus phase")
        await self.commander.reveal_smartglass()
        await asyncio.sleep(self.stimulus_duration)

    async def delay(self, occlude: bool):
        """Optionally occlude smartglass and wait for delay duration"""
        self.log("Delay phase")
        if occlude:
            await self.commander.occlude_smartglass()
        # await asyncio.sleep(self.delay_duration)
        # TODO: Revert
        await asyncio.sleep(random.random() * 5)

    async def response(
        self, arm: Literal["left", "right", "both"]
    ) -> TrialFeedback:
        """Response phase."""
        self.log("Response phase")
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.commander.reveal_smartglass())
            tg.create_task(self.commander.release_arm(arm))

        # Wait for response from monkey button press
        if reaction_time := await self.commander.flic_response_time(
            self.response_timeout
        ):
            # Reward monkey and play sound
            async with asyncio.TaskGroup() as tg:
                if self.reward_sound:
                    tg.create_task(
                        self.commander.play_sound(**self.sound_kwargs)
                    )
                tg.create_task(
                    self.commander.start_reward_and_wait(self.reward_duration)
                )
                self.log(f"Reaction time: {reaction_time}")

            return TrialFeedback(reaction_time=reaction_time, timeout=False)
        else:
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
        await self.prepare(trial_spec.object_pose)
        await self.stimulus()
        await self.delay(trial_spec.occlude)
        feedback = await self.response(trial_spec.arm)
        if not feedback.timeout:
            await self.reveal()
        return feedback

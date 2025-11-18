"""Foraging task."""

import asyncio
import enum
from collections.abc import Mapping
from typing import Any, Literal, Optional

from geometry_msgs.msg import PoseStamped
from tabletop_rig.nodes import Commander

from tabletop_tasks.tasks import BaseTask
from tabletop_tasks.trial_generators import BaseTrialGenerator
from tabletop_tasks.trial_generators.base import TrialFeedback, TrialSpec


class ForagingState(enum.Enum):
    """Foraging state."""

    NEXT_TRIAL_SPEC = 0
    FETCH = 1
    PRESENT = 2
    FIXATION = 3
    STIMULUS = 4
    DELAY = 5
    RESPONSE = 6
    REVEAL = 7
    SEND_FEEDBACK = 8
    UNPRESENT = 9
    RETURN = 10
    FINISHED = 11


class ForagingTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
        fixation_duration: float,
        stimulus_duration: float,
        delay_duration: float,
        reward_duration: float,
        reveal_duration: float,
        response_timeout: float,
        fixation_timeout: float,
        play_sound: bool,
        sound_kwargs: Optional[Mapping[str, Any]] = None,
    ):
        super().__init__(
            commander, trial_generator, logger_name="foraging_task"
        )

        self._fixation_duration = fixation_duration
        self._stimulus_duration = stimulus_duration
        self._delay_duration = delay_duration
        self._response_timeout = response_timeout
        self._fixation_timeout = fixation_timeout
        self._reward_duration = reward_duration
        self._reveal_duration = reveal_duration
        self._play_sound = play_sound
        self._sound_kwargs = sound_kwargs if sound_kwargs is not None else {}

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
        await asyncio.sleep(self._stimulus_duration)

    async def delay(self, occlude: bool):
        """Delay phase."""
        self.log("Delay phase")
        if occlude:
            await self.commander.occlude_smartglass()
        await asyncio.sleep(self._delay_duration)

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
            self._response_timeout
        ):
            # Response received
            sound_task = None
            if self._play_sound:
                sound_task = asyncio.create_task(
                    self.commander.play_sound(**self._sound_kwargs)
                )
            reward_task = asyncio.create_task(
                self.commander.start_reward_and_wait(self._reward_duration)
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
        await asyncio.sleep(self._reveal_duration)

    async def unpresent(self):
        """Unpresent object."""
        self.log("Unpresent phase")
        arm_lock_task = self.commander.lock_arms_and_wait()
        smartglass_task = self.commander.occlude_smartglass()

        await arm_lock_task
        await smartglass_task

        await self.commander.unpresent_object()

    async def run_trial(self, trial_spec: TrialSpec | None) -> TrialFeedback:
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

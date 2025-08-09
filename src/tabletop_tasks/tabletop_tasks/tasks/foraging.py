"""Foraging task."""

import asyncio
import enum
import importlib
from collections.abc import Mapping
from typing import Any

from tabletop_server.nodes import Commander
from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
)


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
    ):
        super().__init__(commander)

        # Create trial_generator if necessary
        if isinstance(trial_generator, Mapping):
            trial_generator_tmp: BaseTrialGenerator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
        else:
            trial_generator_tmp = trial_generator

        # Whether to generate a new trial spec
        self._next_trial = True

        self._trial_generator = trial_generator_tmp
        self._fixation_duration = fixation_duration
        self._stimulus_duration = stimulus_duration
        self._delay_duration = delay_duration
        self._response_timeout = response_timeout
        self._fixation_timeout = fixation_timeout
        self._reward_duration = reward_duration
        self._reveal_duration = reveal_duration

    ############################################################
    # Phases
    ############################################################

    def _next_trial_spec(self) -> bool:
        """Get next trial spec."""
        self.log("Next trial spec phase")

        if self._next_trial:
            try:
                self._trial_spec = next(self._trial_generator)
            except StopIteration:
                self.log("Trial generator finished")
                return False

        self.log(f"Next trial spec: {self._trial_spec}")
        self._trial_feedback = TrialFeedback()
        self._next_trial = False
        return True

    async def _fetch(self):
        """Fetch object for trial."""
        self.log("Fetch phase")
        await self.commander.smartglass_occlude()
        await self.commander.fetch_object(
            object_id=self._trial_spec.object_id,
        )

    async def _present(self):
        """Present object."""
        self.log("Present phase")
        await self.commander.present_object(goal=self._trial_spec.object_pose)

    async def _stimulus(self):
        """Present stimulus."""
        self.log("Stimulus phase")
        # Stimulus has started, so we cannot retry this trial anymore if something fails
        self._next_trial = True
        await self.commander.smartglass_reveal()
        await asyncio.sleep(self._stimulus_duration)

    async def _delay(self):
        """Delay phase."""
        self.log("Delay phase")
        if self._trial_spec.occlude:
            await self.commander.smartglass_occlude()
        await asyncio.sleep(self._delay_duration)

    async def _response(self) -> bool:
        """Response phase."""
        self.log("Response phase")
        await self.commander.arm_release(self._trial_spec.arm)

        # Wait for response
        # Use the ROS2 clock to measure reaction time (for consistency with
        # ROS2 synchronization)
        if reaction_time := await self.commander.flic_response_time(
            self._response_timeout
        ):
            # Response received
            self._trial_feedback.reaction_time = reaction_time
            self.log(f"Reaction time: {self._trial_feedback.reaction_time}")
            await self.commander.reward_and_wait(self._reward_duration)
            return True
        else:
            # Response not received
            self.log("Response timeout")
            self._trial_feedback.timeout = True
            return False

    async def _reveal(self):
        """Reveal object."""
        self.log("Reveal phase")
        await self.commander.smartglass_reveal()
        await asyncio.sleep(self._reveal_duration)

    async def _send_feedback(self):
        """Send feedback."""
        self.log("Send feedback phase")
        self._trial_generator.send(self._trial_feedback)

    async def _unpresent(self):
        """Unpresent object."""
        self.log("Unpresent phase")
        arm_lock_task = self.commander.arm_lock_and_wait()
        smartglass_task = self.commander.smartglass_occlude()

        await arm_lock_task
        await smartglass_task

        await self.commander.unpresent_object()

    async def _return(self):
        """Return object."""
        self.log("Return phase")
        self.return_task = self.commander.return_object()

    async def run(self):
        """Run a trial."""
        self.log("Starting foraging task")
        while True:
            async with self.commander:
                self._state = ForagingState.NEXT_TRIAL_SPEC
                while True:
                    match self._state:
                        case ForagingState.NEXT_TRIAL_SPEC:
                            if self._next_trial_spec():
                                self._state = ForagingState.FETCH
                            else:
                                self._state = ForagingState.FINISHED
                        case ForagingState.FETCH:
                            await self._fetch()
                            self._state = ForagingState.PRESENT
                        case ForagingState.PRESENT:
                            await self._present()
                            self._state = ForagingState.STIMULUS
                        case ForagingState.STIMULUS:
                            await self._stimulus()
                            self._state = ForagingState.DELAY
                        case ForagingState.DELAY:
                            await self._delay()
                            self._state = ForagingState.RESPONSE
                        case ForagingState.RESPONSE:
                            if await self._response():
                                self._state = ForagingState.REVEAL
                            else:
                                self._state = ForagingState.SEND_FEEDBACK
                        case ForagingState.REVEAL:
                            await self._reveal()
                            self._state = ForagingState.SEND_FEEDBACK
                        case ForagingState.SEND_FEEDBACK:
                            await self._send_feedback()
                            self._state = ForagingState.UNPRESENT
                        case ForagingState.UNPRESENT:
                            await self._unpresent()
                            self._state = ForagingState.RETURN
                        case ForagingState.RETURN:
                            await self._return()
                            self._state = ForagingState.NEXT_TRIAL_SPEC
                        case ForagingState.FINISHED:
                            self.log("Foraging task finished")
                            return

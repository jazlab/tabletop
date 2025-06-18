"""Foraging task."""

import asyncio
import enum
import importlib
from collections.abc import Mapping
from typing import Any

from tabletop_server.nodes import Commander
from tabletop_utils.ros import CommanderRecoverableError

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

        self._trial_generator = trial_generator_tmp
        self._fixation_duration = fixation_duration
        self._stimulus_duration = stimulus_duration
        self._delay_duration = delay_duration
        self._response_timeout = response_timeout
        self._fixation_timeout = fixation_timeout
        self._reward_duration = reward_duration
        self._reveal_duration = reveal_duration

        self._fixation_release_task: asyncio.Task | None = None

    def start_hand_fixation(self):
        assert self._fixation_release_task is None
        self._fixation_release_task = (
            self.commander.wait_for_hand_fixation_release()
        )

    def end_hand_fixation(self):
        assert (
            self._fixation_release_task is not None
            and not self._fixation_release_task.done()
        )
        self._fixation_release_task.cancel()
        self._fixation_release_task = None

    def broke_fixation(self):
        self.log("Hand fixation broken")
        assert (
            self._fixation_release_task is not None
            and self._fixation_release_task.done()
        )
        self._fixation_release_task = None

        self._trial_feedback.broke_fixation = True

    async def wait_while_fixating(self, *tasks: asyncio.Task) -> bool:
        """Wait while hand fixation is maintained.

        Returns:
            True if fixation is not broken, False otherwise.
        """
        assert self._fixation_release_task is not None
        pending = set(tasks) | {self._fixation_release_task}
        done = []
        while not self._fixation_release_task.done() and len(done) < len(
            tasks
        ):
            done, pending = await asyncio.wait(
                pending, return_when=asyncio.FIRST_COMPLETED
            )

        if self._fixation_release_task.done():
            # assert len(pending) > 0
            return False
        else:
            assert len(pending) == 1 and self._fixation_release_task in pending
            assert not self._fixation_release_task.done()
            return True

    ############################################################
    # Phases
    ############################################################

    def next_trial_spec(self) -> bool:
        """Get next trial spec."""
        self.log("Next trial spec phase")

        try:
            self._trial_spec = next(self._trial_generator)
        except StopIteration:
            self.log("Trial generator finished")
            return False
        else:
            self.log(f"Next trial spec: {self._trial_spec}")
            self._trial_feedback = TrialFeedback()
            return True

    async def _fetch(self):
        """Fetch object for trial."""
        self.log("Fetch phase")

        # Make smartglass opaque
        await self.commander.smartglass_occlude()

        # Fetch object
        await self.commander.fetch_object(
            object_id=self._trial_spec.object_id,
        )

    async def _present(self):
        """Present object."""
        self.log("Present phase")

        # Present object
        await self.commander.present_object(goal=self._trial_spec.object_pose)

    # TODO: Wait indefinitely until fixation is broken
    async def _fixation(self) -> bool:
        """Wait for hand fixation."""
        self.log("Fixation phase")

        # Wait for hand fixation onset
        if not await self.commander.wait_for_hand_fixation_press(
            self._fixation_timeout
        ):
            self.log("Timeout waiting for fixation onset, waiting again...")
            return False

        self.log("Hand fixation acquired")

        self.start_hand_fixation()

        sleep_task = self.schedule_sleep(self._fixation_duration)
        if await self.wait_while_fixating(sleep_task):
            return True
        else:
            self.broke_fixation()
            return False

    async def _stimulus(self) -> bool:
        """Present stimulus."""
        self.log("Stimulus phase")

        # Now that stimulus phase has started, this trial is over
        self._trial_feedback.next_trial_spec = True

        # Reveal stimulus
        smartglass_task = self.commander.smartglass_reveal()
        if not await self.wait_while_fixating(smartglass_task):
            self.broke_fixation()
            return False

        # Wait for stimulus duration, terminating if hand fixation is broken
        sleep_task = self.schedule_sleep(self._stimulus_duration)
        if await self.wait_while_fixating(sleep_task):
            return True
        else:
            self.broke_fixation()
            return False

    async def _delay(self) -> bool:
        """Delay phase."""
        self.log("Delay phase")

        # Occlude smartglass if necessary
        if self._trial_spec.occlude:
            occlude_task = self.commander.smartglass_occlude()
            if not await self.wait_while_fixating(occlude_task):
                self.broke_fixation()
                return False

        # Wait for delay duration, terminating if hand fixation is broken
        sleep_task = self.schedule_sleep(self._delay_duration)
        if await self.wait_while_fixating(sleep_task):
            self.end_hand_fixation()
            return True
        else:
            self.broke_fixation()
            return False

    async def _response(self) -> bool:
        """Response phase."""
        self.log("Response phase")

        # Open arm door
        await self.commander.arm_door_open_and_wait()

        # Wait for response
        # Use the ROS2 clock to measure reaction time (for consistency with
        # ROS2 synchronization)
        response_start_time = self.commander.time()
        if await self.commander.wait_for_flic_press(self._response_timeout):
            # Response received
            reaction_time = self.commander.time() - response_start_time
            self._trial_feedback.reaction_time = reaction_time
            self.log(f"Reaction time: {reaction_time}")
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

        # Reveal object
        await self.commander.smartglass_reveal()

        # Wait for reveal duration
        await asyncio.sleep(self._reveal_duration)

    def send_trial_feedback(self):
        """Send trial feedback to trial generator."""
        self.log(f"Sending trial feedback: {self._trial_feedback}")
        self._trial_generator.send(self._trial_feedback)

    async def _unpresent(self):
        """Unpresent object."""
        self.log("Unpresent phase")
        arm_door_task = self.commander.arm_door_close_and_wait()
        smartglass_task = self.commander.smartglass_occlude()

        await arm_door_task
        await smartglass_task

        await self.commander.unpresent_object()

    async def _return(self):
        """Return object."""
        self.log("Returning phase")
        self.return_task = self.commander.return_object()

    async def run(self):
        """Run a trial."""
        self.log("Starting foraging task")
        while True:
            async with self.commander:
                self._state = ForagingState.NEXT_TRIAL_SPEC
                try:
                    while True:
                        match self._state:
                            case ForagingState.NEXT_TRIAL_SPEC:
                                if self.next_trial_spec():
                                    self._state = ForagingState.FETCH
                                else:
                                    self._state = ForagingState.FINISHED
                            case ForagingState.FETCH:
                                await self._fetch()
                                self._state = ForagingState.PRESENT
                            case ForagingState.PRESENT:
                                await self._present()
                                self._state = ForagingState.FIXATION
                            case ForagingState.FIXATION:
                                if await self._fixation():
                                    self._state = ForagingState.STIMULUS
                                else:
                                    self._state = ForagingState.SEND_FEEDBACK
                            case ForagingState.STIMULUS:
                                if await self._stimulus():
                                    self._state = ForagingState.DELAY
                                else:
                                    self._state = ForagingState.SEND_FEEDBACK
                            case ForagingState.DELAY:
                                if await self._delay():
                                    self._state = ForagingState.RESPONSE
                                else:
                                    self._state = ForagingState.SEND_FEEDBACK
                            case ForagingState.RESPONSE:
                                if await self._response():
                                    self._state = ForagingState.REVEAL
                                else:
                                    self._state = ForagingState.SEND_FEEDBACK
                            case ForagingState.REVEAL:
                                await self._reveal()
                                self._state = ForagingState.SEND_FEEDBACK
                            case ForagingState.SEND_FEEDBACK:
                                self.send_trial_feedback()
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
                            case _:
                                assert False, f"Invalid state: {self._state}"
                except (TimeoutError, CommanderRecoverableError):
                    self.log("Recoverable error, sending trial feedback")
                    if self._state != ForagingState.RETURN:
                        self.send_trial_feedback()
                    raise

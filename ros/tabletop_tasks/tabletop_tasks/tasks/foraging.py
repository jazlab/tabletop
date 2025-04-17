"""Foraging task."""

import asyncio
import enum
import importlib
import time
from collections.abc import Mapping
from typing import Any

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask
from tabletop_tasks.trial_generators.base_trial_generator import (
    BaseTrialGenerator,
)


class ForagingState(enum.Enum):
    """Foraging state."""

    IDLE = 0
    NEXT_TRIAL_SPEC = 1
    FETCH = 2
    FIXATION = 3
    STIMULUS = 4
    DELAY = 5
    RESPONSE = 6
    REVEAL = 7
    RETURN = 8
    FINISHED = 9


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

        # Logging
        self.log(
            "ForagingTask(\n"
            f"  trial_generator={trial_generator},\n"
            f"  fixation_duration={fixation_duration},\n"
            f"  stimulus_duration={stimulus_duration},\n"
            f"  delay_duration={delay_duration},\n"
            f"  response_timeout={response_timeout},\n"
            f"  fixation_timeout={fixation_timeout},\n"
            f"  reward_duration={reward_duration},\n"
            f"  reveal_duration={reveal_duration},\n"
            ")"
        )

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
        self._state = ForagingState.IDLE

        self._fixation_release_task: asyncio.Task | None = None

    def start_hand_fixation(self):
        assert self._fixation_release_task is None
        self._fixation_release_task = (
            self.commander.wait_for_hand_fixation_release_async()
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
        self._trial_feedback["broke_fixation"] = True
        self._state = ForagingState.RETURN

    async def wait_while_fixating(self, *tasks: asyncio.Task):
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

    def _next_trial_spec(self):
        """Get next trial spec."""
        self.log("Next trial spec phase")

        try:
            self._trial_spec = next(self._trial_generator)
        except StopIteration:
            self.log("Trial generator finished")
            self._state = ForagingState.FINISHED
        else:
            self._state = ForagingState.FETCH

    ############################################################
    # Phases
    ############################################################

    async def _fetch(self):
        """Fetch object for trial."""
        self.log("Fetch phase")

        self._trial_feedback: dict[str, Any] = dict(
            broke_fixation=False,
            reaction_time=None,
            timeout=None,
        )

        # Make smartglass opaque
        await self.commander.smartglass_occlude()

        # Fetch object
        object_id = self._trial_spec.object_id
        object_pose = self._trial_spec.object_pose
        await self.commander.fetch_object_async(
            object_id=object_id, end_goal=object_pose
        )

        # Transition to fixation state
        self._state = ForagingState.FIXATION

    async def _fixation(self):
        """Wait for hand fixation."""
        self.log("Fixation phase")

        # Wait for hand fixation onset
        if not await self.commander.wait_for_hand_fixation_press_async(
            self._fixation_timeout
        ):
            self.log("Timeout waiting for fixation onset, waiting again...")
            return

        self.log("Hand fixation acquired")

        self.start_hand_fixation()

        sleep_task = self.schedule_sleep(self._fixation_duration)
        if await self.wait_while_fixating(sleep_task):  # type: ignore
            self._state = ForagingState.STIMULUS
        else:
            self.broke_fixation()

    async def _stimulus(self):
        """Present stimulus."""
        self.log("Stimulus phase")

        # Reveal stimulus
        smartglass_task = self.commander.smartglass_reveal()
        if not await self.wait_while_fixating(smartglass_task):
            self.broke_fixation()
            return

        # Wait for stimulus duration, terminating if hand fixation is broken
        sleep_task = self.schedule_sleep(self._stimulus_duration)
        if await self.wait_while_fixating(sleep_task):  # type: ignore
            self._state = ForagingState.DELAY
        else:
            self.broke_fixation()

    async def _delay(self):
        """Delay phase."""
        self.log("Delay phase")

        # Occlude smartglass if necessary
        if self._trial_spec.occlude:
            occlude_task = self.commander.smartglass_occlude()
            if not await self.wait_while_fixating(occlude_task):
                self.broke_fixation()
                return

        # Wait for delay duration, terminating if hand fixation is broken
        sleep_task = self.schedule_sleep(self._delay_duration)
        if await self.wait_while_fixating(sleep_task):  # type: ignore
            self.end_hand_fixation()
            self._state = ForagingState.RESPONSE
        else:
            self.broke_fixation()

    async def _response(self):
        """Response phase."""
        self.log("Response phase")

        # Open arm door
        await self.commander.arm_door_open_and_wait()

        # Wait for response
        response_start_time = time.time()
        if await self.commander.wait_for_flic_press_async(
            self._response_timeout
        ):
            # Response received
            reaction_time = time.time() - response_start_time
            self._trial_feedback["reaction_time"] = reaction_time
            self.log(f"Reaction time: {reaction_time}")
            await self.commander.reward_and_wait(self._reward_duration)
            self._state = ForagingState.REVEAL
        else:
            # Response not received
            self.log("Response timeout")
            self._trial_feedback["timeout"] = True
            self._state = ForagingState.RETURN

    async def _reveal(self):
        """Reveal object."""
        self.log("Reveal phase")

        # Reveal object
        await self.commander.smartglass_reveal()

        # Wait for reveal duration
        await asyncio.sleep(self._reveal_duration)

        # Transition to return state
        self._state = ForagingState.RETURN

    async def _return(self):
        """Return object."""
        self.log("Returning object")

        # Give feedback to trial generator
        self.log("Giving feedback to trial generator")
        self._trial_generator.send(**self._trial_feedback)

        arm_door_task = self.commander.arm_door_close_and_wait()
        smartglass_task = self.commander.smartglass_occlude()
        return_task = self.commander.return_object_async()

        await return_task
        await arm_door_task
        await smartglass_task

        await self.commander.plan_and_execute_async("erect")

        # Transition to next trial spec state
        self._state = ForagingState.NEXT_TRIAL_SPEC

    async def run(self):
        """Run a trial."""
        while True:
            async with self.commander.context_manager():
                try:
                    while True:
                        match self._state:
                            case ForagingState.IDLE:
                                self._state = ForagingState.NEXT_TRIAL_SPEC
                            case ForagingState.NEXT_TRIAL_SPEC:
                                self._next_trial_spec()
                            case ForagingState.FETCH:
                                await self._fetch()
                            case ForagingState.FIXATION:
                                await self._fixation()
                            case ForagingState.STIMULUS:
                                await self._stimulus()
                            case ForagingState.DELAY:
                                await self._delay()
                            case ForagingState.RESPONSE:
                                await self._response()
                            case ForagingState.REVEAL:
                                await self._reveal()
                            case ForagingState.RETURN:
                                await self._return()
                            case ForagingState.FINISHED:
                                self.log("Foraging task finished")
                                return
                            case _:
                                raise ValueError(
                                    f"Invalid state: {self._state}"
                                )
                finally:
                    # If an error occurs, reset to the fetch state so the next
                    # attempt will continue for the same trial
                    self._state = ForagingState.FETCH

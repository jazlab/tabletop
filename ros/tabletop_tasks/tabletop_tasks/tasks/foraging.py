"""Foraging task."""

import asyncio
import enum
import time
from typing import Any, Optional

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask
from tabletop_tasks.trial_iterables.base_trial_iterable import (
    BaseTrialIterable,
)


class ForagingState(enum.Enum):
    """Foraging state."""

    IDLE = 0
    FETCH = 1
    STIMULUS = 2
    DELAY = 3
    RESPONSE = 4
    REVEAL = 5
    RETURN = 6


class ForagingTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        trial_iterable: BaseTrialIterable,
        fixation_duration: float = 0.5,
        stimulus_duration: float = 0.5,
        delay_duration: float = 0.5,
        response_timeout: float = 10.0,
        reward_duration_ms: float = 100,
        reveal_duration: float = 0.5,
        logger: Any = None,
    ):
        super().__init__(commander)
        self._trial_iterable = trial_iterable
        self._fixation_duration = fixation_duration
        self._stimulus_duration = stimulus_duration
        self._delay_duration = delay_duration
        self._response_timeout = response_timeout
        self._reward_duration_ms = reward_duration_ms
        self._reveal_duration = reveal_duration

    async def _fetch(self):
        """Fetch object for trial."""
        # Sample new trial
        self._trial_spec = next(self._trial_generator)
        self._trial_feedback = dict(
            broke_fixation=False,
            reaction_time=None,
            timeout=None,
        )
        print("New trial")

        # Make smartglass opaque
        await self.commander.smartglass_occlude_async()

        # Fetch object
        object_id = self._trial_spec.object_id
        object_pose = self._trial_spec.object_pose
        await self.commander.fetch_object_async(object_id, object_pose)

        # Wait for hand fixation
        try:
            async with asyncio.timeout(self._fixation_duration):
                await self.commander.wait_for_hand_fixation_async()
        except asyncio.TimeoutError:
            print("Hand fixation timeout")
            self._trial_feedback["broke_fixation"] = True
            self._state = ForagingState.RETURN

        # Transition to stimulus state
        self._state = ForagingState.STIMULUS

    async def _stimulus(self):
        """Present stimulus."""
        # Reveal stimulus
        await self.commander.smartglass_reveal_async()

        # Wait for stimulus duration, terminating early if hand fixation is
        # broken
        t_stimulus_start = time.time()
        fixation_start_time = t_stimulus_start - self._fixation_duration
        while time.time() - t_stimulus_start < self._stimulus_duration:
            t_hand_off = self.commander.t_hand_fixation_off()
            if t_hand_off > fixation_start_time:
                self._trial_feedback["broke_fixation"] = True
                break
            time.sleep(0.01)

        # Transition to delay state or terminate trial
        if self._trial_feedback["broke_fixation"]:
            self._state = ForagingState.RETURN
        else:
            self._state = ForagingState.DELAY

    async def _delay(self):
        """Delay period."""
        # Occlude smartglass if necessary
        if self._trial_spec.occlude:
            self.commander.smartglass_occlude()

        # Wait for delay duration
        time.sleep(self._delay_duration)

        # Wait for stimulus duration, terminating early if hand fixation is
        # broken
        t_delay_start = time.time()
        fixation_start_time = (
            t_delay_start - self._fixation_duration - self._stimulus_duration
        )
        while time.time() - t_delay_start < self._delay_duration:
            t_hand_off = self.commander.t_hand_fixation_off()
            if t_hand_off > fixation_start_time:
                self._trial_feedback["broke_fixation"] = True
                break
            time.sleep(0.01)

        # Transition to response state or terminate trial
        if self._trial_feedback["broke_fixation"]:
            self._state = ForagingState.RETURN
        else:
            self._state = ForagingState.RESPONSE

    async def _response(self):
        """Response period."""
        # Open arm door
        await self.commander.arm_door_open_async()

        # Wait for response
        response_start_time = time.time()
        timeout = True
        while time.time() < response_start_time + self._response_timeout:
            t_response = self.commander.t_flic_button()
            if t_response > response_start_time:
                timeout = False
                self._trial_feedback["reaction_time"] = (
                    t_response - response_start_time
                )
                self.commander.reward(self._reward_duration_ms)
                break
            time.sleep(0.01)
        self._trial_feedback["timeout"] = timeout

        # Transition to reveal state or terminate trial
        if timeout:
            self._state = ForagingState.RETURN
        else:
            self._state = ForagingState.REVEAL

    async def _reveal(self):
        """Reveal object."""
        # Reveal object
        self.commander.smartglass_reveal()

        # Wait for reveal duration
        time.sleep(self._reveal_duration)

        # Transition to return state
        self._state = ForagingState.RETURN

    async def _return(self):
        """Return object."""
        # Give feedback to trial generator
        self._trial_generator.send(self._trial_feedback)

        # Close arm door
        arm_door_future = self.commander.arm_door_close_async()

        # Occlude smartglass
        smartglass_future = self.commander.smartglass_occlude_async()

        # Return object
        return_future = self.commander.return_object_async(
            self._trial_spec.object_id
        )

        await asyncio.gather(arm_door_future, smartglass_future, return_future)

        # Transition to fetch state
        self._state = ForagingState.FETCH

    async def _return_async(self):
        arm_door_task = self.commander.arm_door_close(blocking=False)
        smartglass_occlude_task = self.commander.smartglass_occlude(
            blocking=False
        )
        return_object_task = self.commander.return_object(
            self._trial_spec.object_id, blocking=False
        )

        self._trial_generator.send(self._trial_feedback)

        await arm_door_task
        await smartglass_occlude_task
        await return_object_task

        self._state = ForagingState.FETCH

    async def run(self, trial_iterable: Optional[BaseTrialIterable] = None):
        """Run a trial."""
        self._trial_generator = iter(
            trial_iterable
            if trial_iterable is not None
            else self._trial_iterable
        )
        try:
            while True:
                match self._state:
                    case ForagingState.FETCH:
                        await self._fetch()
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
                    case _:
                        raise ValueError(f"Invalid state: {self._state}")
        except StopIteration:
            print("Trial generator finished")

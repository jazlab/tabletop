import asyncio
from abc import ABCMeta, abstractmethod
from collections.abc import Mapping
from typing import Any

import rclpy.logging
from rclpy.impl.rcutils_logger import RcutilsLogger
from rpyutils.import_c_library import importlib
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.logging import LoggerMixin

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)

DEFAULT_NOTE = {
    "name": "C",
    "octave": 4,
    "velocity": 127,
    "channel": 0,
}


class NullTrialGenerator:
    def __init__(self):
        self.returned = False

    def __next__(self) -> None:
        """Generate a new trial."""
        if self.returned:
            raise StopIteration

        self.returned = True
        return None

    def send(self, *args, **kwargs):
        """Get trial feedback."""
        pass


class BaseTask(LoggerMixin, metaclass=ABCMeta):
    """Abstract base class for all TableTop tasks.

    Tasks should define a `run` coroutine method that asynchronously executes
    the task.
    """

    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any] | None = None,
        logger_name: str = "task",
    ):
        """Initialize base task.

        Args:
            commander: Commander instance for interacting with the system
            trial_generator: Trial generator instance or config associated with
                task, if applicable
            logger_name: Name to give logger
        """
        self._commander = commander
        self._logger = rclpy.logging.get_logger(logger_name)

        # Create trial_generator if necessary
        if isinstance(trial_generator, Mapping):
            self._trial_generator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
            if not isinstance(self._trial_generator, BaseTrialGenerator):
                raise ValueError(
                    "trial_generator class must be an instance of BaseTrialGenerator"
                )
        elif trial_generator is not None:
            self._trial_generator = trial_generator

    def get_logger(self) -> RcutilsLogger:
        """Get the task logger instance"""
        return self._logger

    @property
    def commander(self) -> Commander:
        """Get the commander instance."""
        return self._commander

    # @property
    # def trial_generator(self) -> BaseTrialGenerator:
    #     """Get the trial generator"""
    #     return self._trial_generator

    async def _occlude_and_lock(self):
        """Occlude smartglass, lock arms, and wait until safe to execute"""
        arm_lock_task = asyncio.create_task(
            self.commander.lock_arms_and_wait()
        )
        smartglass_task = asyncio.create_task(
            self.commander.occlude_smartglass()
        )

        await arm_lock_task
        await smartglass_task

    async def _prepare_trial(self, trial_spec: TrialSpec):
        """Fetch object for trial."""
        await self.commander.fetch_object(trial_spec.object_id)
        await self.commander.pre_present_object()

    async def _reset_trial(self):
        """Reset and return object from previous trial"""
        await self.commander.unpresent_object()
        await self.commander.return_object()

    @abstractmethod
    async def run_trial(
        self, trial_spec: TrialSpec | None
    ) -> TrialFeedback | None:
        """Run one trial

        Args:
            trial_spec: Trial specification for current trial. If no trial
                generator was provided at instantiation, trial_spec will
                be None

        Returns:
            TrialFeedback for the trial
        """

    async def run(self) -> None:
        """Run the task."""

        while True:
            async with self.commander:
                await self._occlude_and_lock()
                for trial_spec in self._trial_generator:
                    # Prepare object for trial
                    await self._prepare_trial(trial_spec)

                    # Run trial and send feedback to trial generator
                    feedback = await self.run_trial(trial_spec)
                    if feedback is not None:
                        self._trial_generator.send(feedback)

                    # Occlude smartglass and lock arms before moving
                    await self._occlude_and_lock()

                    # Reset object before next trial
                    await self._reset_trial()

                    # TODO: Allow resetting and preparing to happen concurrently
                    # reset_task = asyncio.create_task(self._reset_trial())
                    # await reset_task

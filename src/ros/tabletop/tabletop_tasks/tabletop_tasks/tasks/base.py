"""Base task classes for behavioral experiments.

This module provides the abstract base class for all experimental tasks
in the TableTop system. Tasks orchestrate the sequence of robot actions,
subject interactions, and trial logic for behavioral experiments.

The task architecture follows a standard trial-based structure:
1. Prepare trial (fetch and position object)
2. Run trial (present stimulus, collect response)
3. Reset trial (return object to home position)

Tasks work with trial generators to produce sequences of trials,
and can be configured via YAML configuration files.

Classes:
    NullTrialGenerator: Placeholder generator that yields a single None trial.
    BaseTask: Abstract base class for all experimental tasks.

Example:
    class MyTask(BaseTask):
        async def run_trial(self, trial_spec):
            await self.commander.reveal_smartglass()
            response = await self.commander.flic_response_time()
            return TrialFeedback(reaction_time=response)
"""

import asyncio
from abc import ABCMeta, abstractmethod
from collections.abc import Mapping
from typing import Any

from rclpy.impl.rcutils_logger import RcutilsLogger
from rpyutils.import_c_library import importlib
from tabletop_rig.exceptions import (
    ManipulationContextExitedError,
    StateTransitionError,
)
from tabletop_rig.nodes import Commander
from tabletop_rig.nodes.commander import ManipulationContextManager
from tabletop_rig.utils.logging import LoggerMixin

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    DefaultTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class BaseTask(LoggerMixin, metaclass=ABCMeta):
    """Abstract base class for all TableTop experimental tasks.

    Tasks define the logic for running behavioral experiments, including
    stimulus presentation, response collection, and reward delivery.
    Subclasses must implement run_trial() to define trial-specific logic.

    The default run() implementation provides a standard trial loop that:
    1. Locks arms and occludes smartglass for safety
    2. Prepares each trial (fetches and positions object)
    3. Runs the trial and collects feedback
    4. Resets the trial (returns object)

    Subclasses can override run() for custom experiment structures.

    Attributes:
        commander: Reference to the Commander node for robot control.
        _trial_generator: Generator producing TrialSpec objects.
        _logger: ROS logger for this task.
    """

    def __init__(
        self,
        name: str,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any] | None = None,
    ):
        """Initialize the base task.

        Args:
            commander: Commander instance for robot and peripheral control.
            trial_generator: Either a BaseTrialGenerator instance, or a dict
                with "class" and "kwargs" keys for dynamic instantiation.
                If None, tasks must handle trial generation themselves.
            logger_name: Optional name for the ROS logger (currently unused).

        Raises:
            ValueError: If trial_generator dict specifies a class that isn't
                a BaseTrialGenerator subclass.
        """
        self._commander = commander

        self._logger = commander.get_logger().get_child(name)

        # Create trial_generator from config dict if necessary
        if trial_generator is None:
            self._trial_generator = DefaultTrialGenerator(commander)
        elif isinstance(trial_generator, BaseTrialGenerator):
            self._trial_generator = trial_generator
        if isinstance(trial_generator, Mapping):
            self._trial_generator: BaseTrialGenerator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
            if not isinstance(self._trial_generator, BaseTrialGenerator):
                raise ValueError(
                    "trial_generator class must be an instance of BaseTrialGenerator"
                )

    def get_logger(self) -> RcutilsLogger:
        """Get the task logger instance.

        Returns:
            ROS logger for this task.
        """
        return self._logger

    @property
    def commander(self) -> Commander:
        """Get the commander instance.

        Returns:
            The Commander node reference for robot control.
        """
        return self._commander

    @abstractmethod
    async def run(self) -> None:
        """Run the complete task.

        Subclasses must implement this method, which is called in run.py and
        which defines the task runtime.

        Users must remember to use the Commander as an asynchronous context
        manager (`async with self.commander`) before calling any commander
        methods.
        """


class BaseObjectInteractionTask(BaseTask, metaclass=ABCMeta):
    """TODO"""

    def __init__(
        self,
        name: str,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any] | None = None,
    ):
        """Initialize the base task.

        Args:
            commander: Commander instance for robot and peripheral control.
            trial_generator: Either a BaseTrialGenerator instance, or a dict
                with "class" and "kwargs" keys for dynamic instantiation.
                If None, tasks must handle trial generation themselves.
            logger_name: Optional name for the ROS logger (currently unused).

        Raises:
            ValueError: If trial_generator dict specifies a class that isn't
                a BaseTrialGenerator subclass.
        """
        self._commander = commander

        self._logger = commander.get_logger().get_child(name)

        # Create trial_generator from config dict if necessary
        if trial_generator is None:
            self._trial_generator = DefaultTrialGenerator(commander)
        elif isinstance(trial_generator, BaseTrialGenerator):
            self._trial_generator = trial_generator
        if isinstance(trial_generator, Mapping):
            self._trial_generator: BaseTrialGenerator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
            if not isinstance(self._trial_generator, BaseTrialGenerator):
                raise ValueError(
                    "trial_generator class must be an instance of BaseTrialGenerator"
                )

        self._trial_lock = asyncio.Lock()
        self._trial_condition = asyncio.Condition()

    @abstractmethod
    async def run_trial(
        self, trial_spec: TrialSpec, manipulator: ManipulationContextManager
    ) -> TrialFeedback:
        """Run a single trial.

        Subclasses must implement this method to define the trial-specific
        logic including stimulus presentation, response collection, and
        reward delivery.

        Args:
            trial_spec: Specification for the current trial, or None if
                no trial generator was provided.

        Returns:
            TrialFeedback with behavioral measures from the trial,
            or None if no feedback should be sent to the generator.
        """

    async def _occlude_and_lock(self):
        """Occlude smartglass and lock arms concurrently.

        Ensures safety before robot motion by blocking the subject's
        view and constraining their arms. Waits for both operations
        to complete.
        """
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.commander.lock_arm("both"))
            tg.create_task(self.commander.occlude_smartglass())

    async def _run_one_trial(
        self, trial_spec: TrialSpec
    ) -> TrialFeedback | None:
        presented: bool = False
        unpresented: bool = False
        try:
            async with self.commander.manipulation_context(
                trial_spec.group_name
            ) as manipulator:
                if trial_spec.object_id == manipulator.current_manipulation_id:
                    assert manipulator.current_manipulation_id is not None
                    try:
                        await manipulator.reset_object(
                            manipulator.current_manipulation_id
                        )
                    except StateTransitionError:
                        await manipulator.reset_manipulation()
                else:
                    await manipulator.reset_manipulation()

                await manipulator.fetch_object(trial_spec.object_id)

                await self._trial_lock.acquire()
                presented = True

                await manipulator.present_object(trial_spec.object_id)
                # self.log("-" * 100)
                # joint_positions = get_joint_group_positions(
                #     self.commander._moveit.get_current_state(),
                #     trial_spec.group_name,
                # )
                # self.log(
                #     f"Robot {trial_spec.group_name} present state: {joint_positions}"
                # )
                # self.log("-" * 100)
                await manipulator.plan_and_move(goal=trial_spec.object_pose)

                feedback = await self.run_trial(trial_spec, manipulator)

                await self._occlude_and_lock()
                await manipulator.unpresent_object(trial_spec.object_id)

                unpresented = True
                self._trial_lock.release()

                return feedback
        except ManipulationContextExitedError:
            if presented and not unpresented:
                assert self._trial_lock.locked()
                self._trial_lock.release()
            return None

    async def _run_trials_asynchronously(self):
        # Occlude smartglass and lock arms before starting
        await self._occlude_and_lock()

        active_trials: dict[str, asyncio.Task[TrialFeedback | None] | None] = {
            x: None for x in self.commander.robot_names
        }
        active_specs: dict[str, TrialSpec | None] = {
            x: None for x in self.commander.robot_names
        }

        async with asyncio.TaskGroup() as tg:
            for next_spec in self._trial_generator:
                if active_trials[next_spec.group_name] is None:
                    active_specs[next_spec.group_name] = next_spec
                    active_trials[next_spec.group_name] = tg.create_task(
                        self._run_one_trial(next_spec)
                    )
                    continue

                self.log(
                    f"The next trial spec requested the robot "
                    f"{next_spec.group_name}, but this robot is "
                    f"already being used for an active trial. "
                    f"Waiting for any trials to finish.",
                    severity="WARN",
                )

                await asyncio.wait(
                    [x for x in active_trials.values() if x is not None],
                    return_when=asyncio.FIRST_COMPLETED,
                )
                for group_name, active_trial in active_trials.items():
                    if active_trial is not None and active_trial.done():
                        active_spec = active_specs[group_name]
                        assert active_spec is not None
                        feedback = await active_trial
                        self._trial_generator.send(active_spec, feedback)
                        active_trials[group_name] = None
                        active_specs[group_name] = None
                        break

    async def run(self) -> None:
        """Run the complete task.

        Executes the standard trial loop: for each trial from the
        generator, prepares the trial, runs it, collects feedback,
        and resets before the next trial.

        The loop runs indefinitely, restarting the trial generator
        when exhausted. Override this method for custom task structures.
        """
        await self._run_trials_asynchronously()

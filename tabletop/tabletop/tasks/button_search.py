"""Button search task.

This task has the following trial structure:
- Robot initiates trial setup
- Smartglass becomes transparent (after hand fixation)
- Arm door opens
- Delay until reward button is pressed or timeout
- Smartglass becomes opaque
- Arm door closes (after hand fixation)
- Robot initiates trial cleanup
"""

import time

from tabletop import io, trial_generators
from tabletop.logger import logger

from . import base


class ButtonSearch(base.BaseTask):
    """Button search task class."""

    def __init__(
        self,
        trial_generator: trial_generators.BaseTrialGenerator,
        robot: io.BaseRobot,
        reward_button: io.BaseRewardButton,
        juice_tube: io.BaseJuiceTube,
        hand_fixation: io.BaseHandFixation,
        smartglass: io.BaseSmartGlass,
        arm_door: io.BaseArmDoor,
        robot_setup_ms: float = 1000,
        hand_fixation_ms: float = 1000,
        visible_only_ms: float = 1000,
        timeout_seconds: float = 2,
        robot_cleanup_ms: float = 1000,
    ):
        """Initialize the ButtonSearch class.

        Args:
            trial_generator: Trial generator.
            robot: Robot I/O module.
            reward_button: Reward button I/O module.
            juice_tube: Juice tube I/O module.
            hand_fixation: Hand fixation I/O module.
            smartglass: Smartglass I/O module.
            arm_door: Arm door I/O module.
            robot_setup_ms: Time between initiation of robot setup and
                initiation of smartglass transparency (subject to hand
                fixation).
            hand_fixation_ms: Time for hand fixation in milliseconds.
            visible_only_ms: Time for smartglass to be transparent in
                milliseconds before arm door opens.
            timeout_seconds: If button is not pressed within this time, the
                trial is terminated.
            robot_cleanup_ms: Time for robot to cleanup trial in milliseconds.
        """
        self._trial_generator = trial_generator
        self._robot = robot
        self._reward_button = reward_button
        self._juice_tube = juice_tube
        self._hand_fixation = hand_fixation
        self._smartglass = smartglass
        self._arm_door = arm_door
        self._robot_setup_seconds = robot_setup_ms / 1000
        self._hand_fixation_seconds = hand_fixation_ms / 1000
        self._visible_only_seconds = visible_only_ms / 1000
        self._timeout_seconds = timeout_seconds
        self._robot_cleanup_seconds = robot_cleanup_ms / 1000

        # Initialize variable to track trial status
        self._trial_in_progress = False

    def run_trial(self) -> dict:
        """Run a single trial."""
        self._trial_in_progress = True

        # Sample a trial
        logger.info("    Generating trial")
        trial = self._trial_generator()

        # Setup the trial
        logger.info("    Robot setup")
        self._robot.setup_trial(trial)
        time.sleep(self._robot_setup_seconds)

        # Wait for hand fixation
        logger.info("    Waiting for hand fixation")
        self._hand_fixation.wait_for_fixation(self._hand_fixation_seconds)

        # Make smartglass transparent
        logger.info("    Making smartglass transparent")
        self._smartglass.make_transparent()
        time.sleep(self._visible_only_seconds)

        # Open the arm door
        logger.info("    Opening the arm door")
        self._arm_door.open()

        # Wait for button press or timeout
        logger.info("    Waiting for button press or timeout")
        start_time = time.time()
        trial_success = False
        reaction_time = None
        while time.time() - start_time < self._timeout_seconds:
            if self._reward_button.is_pressed():
                trial_success = True
                reaction_time = time.time() - start_time
                logger.info(
                    f"    Button pressed after {reaction_time:.3f} seconds"
                )
                break
        if reaction_time is None:
            logger.info("    Trial timeout")

        # Deliver juice if successful
        if trial_success:
            logger.info("    Delivering juice")
            self._juice_tube.reward()

        # Make smartglass opaque
        logger.info("    Making smartglass opaque")
        self._smartglass.make_opaque()

        # Wait for hand fixation
        logger.info("    Waiting for hand fixation")
        self._hand_fixation.wait_for_fixation(self._hand_fixation_seconds)

        # Close the arm door
        logger.info("    Closing the arm door")
        self._arm_door.close()

        # Clean up the trial
        logger.info("    Robot cleanup")
        self._robot.cleanup_trial(trial)

        # Prepare trial data
        trial_data = dict(
            trial_success=trial_success,
            reaction_time=reaction_time,
            **trial,
        )

        # Give feedback to the trial generator
        self._trial_generator.feedback(trial_data)

        self._trial_in_progress = False

        return trial_data

    def finish_trial(self) -> None:
        """Finish the current trial and stop the task."""
        logger.info("Finishing the current trial.")
        while self._trial_in_progress:
            time.sleep(0.01)
        return

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the task."""
        field_names = [
            "trial_success",
            "reaction_time",
        ] + self._trial_generator.field_names
        return field_names

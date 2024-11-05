"""Mock trial generator module for the tabletop app."""

import numpy as np

from tabletop.logger import logger

from .base import BaseTrialGenerator


class MockTrialGenerator(BaseTrialGenerator):
    """Mock trial generator class for the tabletop app."""

    def __init__(self):
        """Initialize the MockTrialGenerator class."""
        self._trial_count = 0

    def __call__(self) -> dict:
        """Generate a trial."""
        return {"trial_number": self._trial_count}

    @property
    def field_names(self) -> list:
        """Return the field names for the trial generator."""
        return ["trial_number"]


class MockBlockStructuredAffordance(BaseTrialGenerator):
    """Mock block structured affordance trial generator."""

    def __init__(
        self,
        affordance_to_object_ids: dict[str, list[int]],
        trials_per_block: int = 10,
    ):
        """Initialize the MockBlockStructuredAffordance class.

        Args:
            affordance_to_object_ids: A dictionary mapping affordances to object
                IDs.
            trials_per_block: The number of trials per block.
        """
        self._affordance_to_object_ids = affordance_to_object_ids
        self._trials_per_block = trials_per_block
        self._affordances = list(affordance_to_object_ids.keys())
        self._trial_count = 0
        self._since_block_change = 0
        self._current_affordance = np.random.choice(self._affordances)

    def __call__(self) -> dict:
        """Generate a trial."""
        # Switch block if necessary and possible
        should_switch_block = (
            len(self._affordances) > 1
            and self._since_block_change >= self._trials_per_block
        )
        if should_switch_block:
            options = self._affordances.copy()
            options.remove(self._current_affordance)
            new_affordance = np.random.choice(options)
            logger.info(
                f"\nSwitching block from {self._current_affordance} to "
                f"{new_affordance}\n"
            )
            self._current_affordance = new_affordance
            self._since_block_change = 0

        # Generate trial
        object_ids = self._affordance_to_object_ids[self._current_affordance]
        object_id = np.random.choice(object_ids)
        trial_data = {
            "trial_number": self._trial_count,
            "affordance": self._current_affordance,
            "object_id": object_id,
        }
        self._trial_count += 1
        self._since_block_change += 1
        return trial_data

    @property
    def field_names(self) -> list:
        """Return the field names for the trial generator."""
        return ["trial_number", "affordance", "object_id"]

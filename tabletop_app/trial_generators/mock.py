"""Mock trial generator module for the tabletop app."""

from trial_generators.base import BaseTrialGenerator


class MockTrialGenerator(BaseTrialGenerator):
    """Mock trial generator class for the tabletop app."""

    def __init__(self):
        """Initialize the MockTrialGenerator class."""
        self._trial_count = 0

    def __call__(self) -> dict:
        """Generate a trial."""
        return {"trial_number": self._trial_count}

    def feedback(self, trial_data: dict):
        """Provide feedback to the trial generator."""
        del trial_data

    @property
    def field_names(self) -> list:
        """Return the field names for the trial generator."""
        return ["trial_number"]

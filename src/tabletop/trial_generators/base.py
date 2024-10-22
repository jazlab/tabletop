"""Base trial generator module for the tabletop app."""

import abc


class BaseTrialGenerator(abc.ABC):
    """Base trial generator class from which all trial generators inherit."""

    @abc.abstractmethod
    def __call__(self) -> dict:
        """Generate a trial."""
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def field_names(self) -> list:
        """Return the field names for the trial generator."""
        raise NotImplementedError

    def feedback(self, trial_data: dict):
        """Provide feedback to the trial generator."""
        del trial_data

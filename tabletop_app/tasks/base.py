"""Base task module for the tabletop app."""

import abc


class BaseTask(abc.ABC):
    """Base task class from which all tasks inherit."""

    @abc.abstractmethod
    def run_trial(self) -> dict:
        """Run a single trial and return dictionary of trial data."""
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def field_names(self) -> list:
        """Return the field names for the task."""
        raise NotImplementedError

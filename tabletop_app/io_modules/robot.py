"""Robot module for setting up and cleaning up trials."""

import abc
import time

import numpy as np
from io_modules.base import BaseIO


class BaseRobot(BaseIO, metaclass=abc.ABCMeta):
    """BaseRobot module for setting up and cleaning up trials."""

    def __init__(self, name: str = "robot", **base_io_kwargs: dict):
        """Initialize the MockRobot class.

        Args:
            name: Name of the I/O module.
            base_io_kwargs: Keyword arguments for the BaseIO class.
        """
        super().__init__(name=name, **base_io_kwargs)

    @abc.abstractmethod
    def setup_trial(self, trial: dict):
        raise NotImplementedError

    @abc.abstractmethod
    def cleanup_trial(self, trial: dict):
        raise NotImplementedError


class MockRobot(BaseRobot):
    """MockRobot module for eye tracking."""

    def __init__(self, **base_robot_kwargs: dict):
        """Initialize the MockRobot class."""
        super().__init__(**base_robot_kwargs)
        self._position = np.zeros(3)

    def setup_trial(self, trial: dict):
        del trial
        self._position = np.random.rand(3)

    def cleanup_trial(self, trial: dict):
        del trial
        pass

    def _fetch_data(self) -> list[dict]:
        """Fetch a list of data dictionaries."""
        data_sample = dict(
            time=time.time(),
            x=self._position[0],
            y=self._position[1],
            z=self._position[2],
        )
        return [data_sample]

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the I/O module."""
        return ["time", "x", "y", "z"]

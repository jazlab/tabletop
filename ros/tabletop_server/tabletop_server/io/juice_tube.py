"""JuiceTube module for delivering reward."""

import abc
import time

from .base import BaseIO


class BaseJuiceTube(BaseIO, metaclass=abc.ABCMeta):
    """BaseJuiceTube module for delivering reward."""

    def __init__(self, name: str = "juice_tube", **base_io_kwargs: dict):
        """Initialize the BaseJuiceTube class.

        Args:
            name: Name of the I/O module.
            base_io_kwargs: Keyword arguments for the BaseIO class.
        """
        super().__init__(name=name, **base_io_kwargs)
        self._juice_tube_open = False

    @abc.abstractmethod
    def reward(self):
        """Deliver reward."""
        raise NotImplementedError

    def _fetch_data(self) -> list[dict]:
        """Fetch a list of data dictionaries."""
        data_sample = dict(
            time=time.time(),
            juice_tube_open=self._juice_tube_open,
        )
        return [data_sample]

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the I/O module."""
        return ["time", "juice_tube_open"]


class MockJuiceTube(BaseJuiceTube):
    """MockJuiceTube module for eye tracking."""

    def __init__(
        self, reward_duration_ms: float = 100, **base_juice_tube_kwargs: dict
    ):
        """Initialize the MockJuiceTube class."""
        super().__init__(**base_juice_tube_kwargs)
        self._reward_duration_seconds = reward_duration_ms / 1000

    def reward(self):
        """Deliver reward."""
        self._juice_tube_open = True
        time.sleep(self._reward_duration_seconds)
        self._juice_tube_open = False

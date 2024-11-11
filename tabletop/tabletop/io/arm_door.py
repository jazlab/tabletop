"""ArmDoor module for controlling scene access."""

import abc
import time

from .base import BaseIO


class BaseArmDoor(BaseIO, metaclass=abc.ABCMeta):
    """BaseArmDoor module for controlling scene access."""

    def __init__(self, name: str = "arm_door", **base_io_kwargs: dict):
        """Initialize the BaseArmDoor class.

        Args:
            name: Name of the I/O module.
            base_io_kwargs: Keyword arguments for the BaseIO class.
        """
        super().__init__(name=name, **base_io_kwargs)

    @abc.abstractmethod
    def open(self):
        """Open the arm door."""
        raise NotImplementedError

    @abc.abstractmethod
    def close(self):
        """Close the arm door."""
        raise NotImplementedError

    def _fetch_data(self) -> list[dict]:
        data_sample = dict(
            time=time.time(),
            open=self._is_open,
        )
        return [data_sample]

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the I/O module."""
        return ["time", "open"]


class MockArmDoor(BaseArmDoor):
    """MockArmDoor module for controlling scene access."""

    def __init__(self, **base_arm_door_kwargs: dict):
        super().__init__(**base_arm_door_kwargs)
        self._is_open = False

    def open(self):
        self._is_open = True

    def close(self):
        self._is_open = False

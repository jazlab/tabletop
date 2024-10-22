"""SmartGlass module for controlling scene visibility."""

import abc
import time

from .base import BaseIO


class BaseSmartGlass(BaseIO, metaclass=abc.ABCMeta):
    """BaseSmartGlass module for controlling scene visibility."""

    def __init__(self, name: str = "smartglass", **base_io_kwargs: dict):
        """Initialize the BaseSmartGlass class.

        Args:
            name: Name of the I/O module.
            base_io_kwargs: Keyword arguments for the BaseIO class.
        """
        super().__init__(name=name, **base_io_kwargs)

    @abc.abstractmethod
    def make_transparent(self):
        """Make the scene transparent."""
        raise NotImplementedError

    @abc.abstractmethod
    def make_opaque(self):
        """Make the scene opaque."""
        raise NotImplementedError

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the I/O module."""
        return ["time", "is_transparent"]


class MockSmartGlass(BaseSmartGlass):
    """MockSmartGlass module for controlling scene visibility."""

    def __init__(self, **base_smartglass_kwargs: dict):
        super().__init__(**base_smartglass_kwargs)
        self._transparent = False

    def make_transparent(self):
        self._transparent = True

    def make_opaque(self):
        self._transparent = False

    def _fetch_data(self) -> list[dict]:
        """Fetch a list of data dictionaries."""
        data_sample = dict(
            time=time.time(),
            is_transparent=self._transparent,
        )
        return [data_sample]

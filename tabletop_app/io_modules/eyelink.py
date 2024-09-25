"""Eyelink modules for eye tracking."""

import time
from collections import namedtuple

import numpy as np
from io_modules.base import BaseIO


class BaseEyelink(BaseIO):
    """BaseEyelink module from which ll Eyelink modules should inherit."""

    def __init__(self, name: str = "eyelink", **base_io_kwargs: dict):
        """Initialize the Eyelink class.

        Args:
            name: Name of the I/O module.
            base_io_kwargs: Keyword arguments for the BaseIO class.
        """
        super().__init__(name=name, **base_io_kwargs)

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the I/O module."""
        return ["time", "pupil_l", "pupil_r", "x_l", "y_l", "x_r", "y_r"]


class MockEyelink(BaseEyelink):
    """MockEyelink module for eye tracking."""

    def __init__(
        self, update_probability: float = 0.1, **base_eyelink_kwargs: dict
    ):
        """Initialize the Eyelink class.

        Args:
            update_probability: Probability of updating velocities.
            base_eyelink_kwargs: Keyword arguments for the BaseEyelink class.
        """
        super().__init__(**base_eyelink_kwargs)
        self._update_probability = update_probability

        # Initialize dynamics for each variable
        dynamics = namedtuple("Dynamics", ["mean", "max_speed", "timescale"])
        self._dynamics = {
            "pupil_l": dynamics(100, 2, 0.1),
            "pupil_r": dynamics(100, 2, 0.1),
            "x_l": dynamics(0, 5, 0.5),
            "y_l": dynamics(0, 5, 0.5),
            "x_r": dynamics(0, 5, 0.5),
            "y_r": dynamics(0, 5, 0.5),
        }
        self._eye_state = {k: d.mean for k, d in self._dynamics.items()}
        self._velocities = self._sample_new_velocities()

    def _sample_new_velocities(self) -> dict:
        velocities = {
            k: np.random.uniform(-d.max_speed, d.max_speed)
            for k, d in self._dynamics.items()
        }
        return velocities

    def _update_state(self):
        self._eye_state = {
            k: s + self._velocities[k] for k, s in self._eye_state.items()
        }

    def _update_velocities(self):
        self._velocities = {
            k: self._velocities[k]
            + d.timescale * (d.mean - self._eye_state[k])
            for k, d in self._dynamics.items()
        }

    def _fetch_data(self) -> list[dict]:
        """Fetch a list of data dictionaries."""
        if np.random.rand() < self._update_probability:
            self._velocities = self._sample_new_velocities()
        self._update_state()
        self._update_velocities()
        data_sample = {"time": time.time(), **self._eye_state}
        return [data_sample]

"""RewardButton module for triggering reward delivery."""

import abc
import time

import numpy as np

from .base import BaseIO


class BaseRewardButton(BaseIO, metaclass=abc.ABCMeta):
    """BaseRewardButton module for triggering reward delivery."""

    def __init__(self, name: str = "reward_button", **base_io_kwargs: dict):
        """Initialize the MockRewardButton class.

        Args:
            name: Name of the I/O module.
            base_io_kwargs: Keyword arguments for the BaseIO class.
        """
        super().__init__(name=name, **base_io_kwargs)

    @abc.abstractmethod
    def is_pressed(self) -> bool:
        """Check if the reward button is pressed."""
        raise NotImplementedError


class MockRewardButton(BaseRewardButton):
    """MockRewardButton module for triggering reward delivery."""

    def __init__(
        self,
        press_frequency_seconds: float = 2,
        **base_reward_button_kwargs: dict,
    ):
        """Initialize the MockRewardButton class.

        Args:
            press_frequency_seconds: Frequency of reward button presses, namely
                at what frequencey the button is pressed. The true presses are
                stochastic, subject to a Poisson process with this frequency.
            base_reward_button_kwargs: Keyword arguments for the
                BaseRewardButton class.
        """
        super().__init__(**base_reward_button_kwargs)
        self._press_frequency_seconds = press_frequency_seconds
        self._is_pressed = False
        self._last_fetched = time.time()

    def _fetch_data(self) -> list[dict]:
        """Fetch a list of data dictionaries."""
        # Update self._is_pressed
        elapsed_time = time.time() - self._last_fetched
        pressed_probability = elapsed_time / self._press_frequency_seconds
        self._is_pressed = np.random.rand() < pressed_probability
        self._last_fetched = time.time()

        # Return data sample
        data_sample = dict(
            time=time.time(),
            is_pressed=self._is_pressed,
        )
        return [data_sample]

    def is_pressed(self) -> bool:
        """Check if the reward button is pressed."""
        return self._is_pressed

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the I/O module."""
        return ["time", "is_pressed"]

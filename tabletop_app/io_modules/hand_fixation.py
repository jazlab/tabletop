"""HandFixation module for determining if subject has hand fixation."""

import abc
import time

import numpy as np
from io_modules.base import BaseIO


class BaseHandFixation(BaseIO, metaclass=abc.ABCMeta):
    """BaseHandFixation module for determining if subject has hand fixation."""

    def __init__(self, name: str = "hand_fixation", **base_io_kwargs: dict):
        """Initialize the MockHandFixation class.

        Args:
            name: Name of the I/O module.
            base_io_kwargs: Keyword arguments for the BaseIO class.
        """
        super().__init__(name=name, **base_io_kwargs)

    @abc.abstractmethod
    def wait_for_fixation(self, hand_fixation_seconds) -> bool:
        """Wait until hand fixation."""
        del hand_fixation_seconds
        raise NotImplementedError

    @property
    def field_names(self) -> list[str]:
        """Return the field names for the I/O module."""
        return ["time", "hand_fixating"]


class MockHandFixation(BaseHandFixation):
    """MockHandFixation module for triggering reward delivery."""

    def __init__(
        self,
        change_frequency_seconds: float = 1,
        waiting_period_ms: float = 10,
        **base_hand_fixation_kwargs: dict
    ):
        """Initialize the MockHandFixation class.

        Args:
            change_frequency_seconds: Frequency of hand fixating changes, namely
                at what frequencey the subject switches between hand fixating
                and not hand fixating. The true changes are stochastic, subject
                to a Poisson process with this frequency.
            waiting_period_ms: Time to sleep between fetches in milliseconds.
            base_hand_fixation_kwargs: Keyword arguments for the
                BaseHandFixation class.
        """
        super().__init__(**base_hand_fixation_kwargs)
        self._waiting_period_seconds = waiting_period_ms / 1000
        self._change_frequency_seconds = change_frequency_seconds
        self._hand_fixating = False
        self._last_onset_time = None
        self._last_fetch_time = time.time()

    def _fetch_data(self) -> list[dict]:
        """Fetch a list of data dictionaries."""
        # Update self._hand_fixating
        current_time = time.time()
        since_last_fetch = current_time - self._last_fetch_time
        self._last_fetch_time = current_time
        change_probability = since_last_fetch / self._change_frequency_seconds
        if np.random.rand() < change_probability:
            if self._hand_fixating:
                self._hand_fixating = False
                self._last_onset_time = None
            else:
                self._hand_fixating = True
                self._last_onset_time = time.time()

        # Return data sample
        data_sample = dict(
            time=time.time(),
            hand_fixating=self._hand_fixating,
        )
        return [data_sample]

    def wait_for_fixation(self, hand_fixation_seconds) -> bool:
        """Wait until hand fixating for hand_fixation_seconds."""
        finished = False
        while not finished:
            if self._hand_fixating:
                hand_fixation_duration = time.time() - self._last_onset_time
                if hand_fixation_duration >= hand_fixation_seconds:
                    finished = True
            time.sleep(self._waiting_period_seconds)
        return

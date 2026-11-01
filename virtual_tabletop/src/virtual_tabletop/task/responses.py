"""Where a trial's response (a Flic button press) comes from.

On the rig, the response window asks the Flic node for a press of the
presented object's button. In the MuJoCo task, a ``ResponseModel`` answers
instead:

- :class:`FlicSimResponse`: the rig's own simulation (press after
  ``uniform(min_delay, max_delay)`` s; ``flic.py`` with ``simulate:=true``);
- :class:`ScheduledResponse`: fixed reaction times per trial, e.g. taken
  from a bag for parity checks;
- :class:`NoResponse`: every trial times out.
"""

from collections.abc import Mapping
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from virtual_tabletop.task.sim import ForagingSim, Presentation


class ResponseModel:
    """Base class: never responds."""

    def open(self, sim: "ForagingSim", presentation: "Presentation") -> None:
        """Called when a response window opens (arms released)."""

    def pressed(
        self, sim: "ForagingSim", presentation: "Presentation"
    ) -> bool:
        """Polled every tick while the window is open."""
        return False


NoResponse = ResponseModel


class FlicSimResponse(ResponseModel):
    """Press after a uniform random delay, like the rig's Flic simulation."""

    def __init__(
        self, min_delay: float = 1.0, max_delay: float = 3.0, seed=None
    ):
        self.min_delay = min_delay
        self.max_delay = max_delay
        self._rng = np.random.default_rng(seed)
        self._press_at: float | None = None

    def open(self, sim, presentation):
        self._press_at = sim.time + self._rng.uniform(
            self.min_delay, self.max_delay
        )

    def pressed(self, sim, presentation):
        return self._press_at is not None and sim.time >= self._press_at


class ScheduledResponse(ResponseModel):
    """Press at a fixed delay after release, per trial number.

    Args:
        reaction_times: ``{trial_number: seconds or None}``; trials missing
            or ``None`` time out.
    """

    def __init__(self, reaction_times: Mapping[int, float | None]):
        self.reaction_times = dict(reaction_times)
        self._press_at: float | None = None

    def open(self, sim, presentation):
        rt = self.reaction_times.get(presentation.spec.trial_number)
        self._press_at = None if rt is None else sim.time + rt

    def pressed(self, sim, presentation):
        return self._press_at is not None and sim.time >= self._press_at

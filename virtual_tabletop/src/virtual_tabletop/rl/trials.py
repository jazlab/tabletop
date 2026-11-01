"""The RL tasks' trial sequence and phase timing (no MuJoCo).

Trials come from the task YAML's trial generator (as on the rig), one after
another with no returning or resetting. Each trial runs the rig's phases:

1. **intertrial**: the object is already at its presentation pose;
2. **stimulus**: ``stimulus_duration`` (on the rig the glass clears);
3. **delay**: ``delay_duration`` (on the rig the glass turns opaque if the
   spec says ``occlude``);
4. **response**: the spec's subject arm(s) are released for
   ``response_timeout``.

"Stimulus visible" says whether the rig's glass would be clear; the RL scene
has no glass.

Two trial orders: ``"config"`` runs the task YAML's own generator (the
rig's order: ``foraging_ordered`` cycles objects fastest, then arms, then
its three poses, so a short episode from a reset only ever sees the first
few objects and the first pose); ``"random"`` samples objects, poses,
released arms and occlusion independently each trial (the rig's
``RandomChoiceAlternating`` on the same lists), which RL training wants.
``arms`` overrides which subject arm(s) trials release.
"""

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from virtual_tabletop.task.config import ForagingConfig
from virtual_tabletop.task.generators import (
    TrialFeedback,
    TrialSpec,
    make_generator,
)

PHASES = ("intertrial", "stimulus", "delay", "response")
#: Trial orders: the task YAML's generator, or independent random sampling.
ORDERS = ("config", "random")


@dataclass
class TrialRecord:
    """One trial.

    Attributes:
        spec: The trial spec (object, pose, released arm(s), occlusion).
        start: Sim time the trial began (s).
        touched_at: First time a released hand touched the object during
            the response window, or ``None``.
        touched_by: Which arm touched first (``"left"``/``"right"``).
        pressed_at: First time a released hand pressed the object's button
            during the response window, or ``None``.
        pressed_by: Which arm pressed first.
        rewarded: Whether the response was rewarded.
    """

    spec: TrialSpec
    start: float
    touched_at: float | None = None
    touched_by: str | None = None
    pressed_at: float | None = None
    pressed_by: str | None = None
    rewarded: bool = False


class TrialSchedule:
    """Trials and their phases on simulated time.

    Args:
        config: The task config (timings, generator).
        num_trials: Trials per episode.
        intertrial: Duration of the phase that starts each trial (s).
        response_timeout: Overrides the config's response window (s).
        order: ``"config"`` or ``"random"`` (see the module docstring).
        arms: Overrides the released arm(s) trials cycle through or sample
            from, e.g. ``["right"]``.
    """

    def __init__(
        self,
        config: ForagingConfig,
        num_trials: int,
        intertrial: float = 0.5,
        response_timeout: float | None = None,
        order: str = "config",
        arms: Sequence[str] | None = None,
    ):
        if order not in ORDERS:
            raise ValueError(f"order must be one of {ORDERS}: {order!r}")
        self.config = config
        self.num_trials = num_trials
        self.generator_class, self.generator_kwargs = _generator(
            config, order, arms
        )
        self.durations = {
            "intertrial": intertrial,
            "stimulus": config.stimulus_duration,
            "delay": config.delay_duration,
            "response": response_timeout or config.response_timeout,
        }
        self.trials: list[TrialRecord] = []

    def reset(self, seed: int | None) -> None:
        self._generator = make_generator(
            self.generator_class,
            dict(self.generator_kwargs, num_trials=self.num_trials),
            seed=seed,
            prefer_feedback_group=False,  # no parallel robot arms: alternate
        )
        self.trials = []

    @property
    def trial(self) -> TrialRecord:
        return self.trials[-1]

    @property
    def index(self) -> int:
        return len(self.trials) - 1

    @property
    def last(self) -> bool:
        """Whether the current trial is the episode's last."""
        return len(self.trials) >= self.num_trials

    def start(self, time: float) -> TrialRecord:
        """Begin the next trial at ``time``."""
        self.trials.append(TrialRecord(next(self._generator), time))
        self.update(time)
        return self.trial

    def update(self, time: float) -> None:
        """Set ``phase``, ``phase_time`` and ``stimulus_visible``."""
        elapsed = time - self.trial.start
        for phase in PHASES:
            duration = self.durations[phase]
            if elapsed < duration - 1e-9 or phase == "response":
                self.phase, self.phase_time = phase, elapsed
                break
            elapsed -= duration
        self.stimulus_visible = self.phase == "stimulus" or (
            self.phase in ("delay", "response") and not self.trial.spec.occlude
        )

    @property
    def time_left(self) -> float:
        """Time left in the current phase (s)."""
        return self.durations[self.phase] - self.phase_time

    @property
    def response_over(self) -> bool:
        return self.phase == "response" and self.time_left <= 1e-9

    def released(self, sides) -> list[str]:
        """Which of ``sides`` are free to respond now."""
        if self.phase != "response":
            return []
        arm = self.trial.spec.arm
        return [s for s in sides if arm == "both" or s == arm]

    def finish(self, time: float, responded_at: float | None) -> None:
        """End the current trial; feed its outcome back to the generator."""
        response_start = time - self.phase_time
        self._generator.send(
            self.trial.spec,
            TrialFeedback(
                reaction_time=None
                if responded_at is None
                else responded_at - response_start,
                timeout=not self.trial.rewarded,
            ),
        )


def _generator(config: ForagingConfig, order: str, arms) -> tuple[str, dict]:
    """The generator class and kwargs for a trial order."""
    cls, kwargs = config.generator_class, dict(config.generator_kwargs)
    if arms is not None:
        kwargs["arms"] = list(arms)
    if order == "random" and cls != "RandomChoiceAlternating":
        occlude = kwargs.pop("occlude", [False])
        kwargs["occlude_prob"] = float(np.mean(occlude))
        cls = "RandomChoiceAlternating"
    return cls, kwargs

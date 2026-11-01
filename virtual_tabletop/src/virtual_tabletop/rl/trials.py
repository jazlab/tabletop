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
"""

from dataclasses import dataclass

from virtual_tabletop.task.config import ForagingConfig
from virtual_tabletop.task.generators import (
    TrialFeedback,
    TrialSpec,
    make_generator,
)

PHASES = ("intertrial", "stimulus", "delay", "response")


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
    """

    def __init__(
        self,
        config: ForagingConfig,
        num_trials: int,
        intertrial: float = 0.5,
        response_timeout: float | None = None,
    ):
        self.config = config
        self.num_trials = num_trials
        self.durations = {
            "intertrial": intertrial,
            "stimulus": config.stimulus_duration,
            "delay": config.delay_duration,
            "response": response_timeout or config.response_timeout,
        }
        self.trials: list[TrialRecord] = []

    def reset(self, seed: int | None) -> None:
        self._generator = make_generator(
            self.config.generator_class,
            dict(self.config.generator_kwargs, num_trials=self.num_trials),
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

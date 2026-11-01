"""ROS-free ports of the rig's foraging trial generators.

Ported from ``tabletop_tasks/trial_generators`` (``base.py``,
``ordered_choice_alternating.py``, ``random_choice_alternating.py``) with the
same protocol and quirks, so a given ordered config yields the same trial
sequence as on the rig (the random generator samples the same
distributions from its own seeded RNG):

- Each robot group keeps one *pending* spec until ``send()`` reports it done.
  Asking for a busy group returns that same pending spec again, which is how
  the task's dispatch loop waits without losing trials.
- Groups alternate round-robin, except that the group that last reported
  feedback is offered the next trial (``prefer_feedback_group=False``
  gives plain round-robin, for sims without parallel robot arms).
- ``trial_number`` is assigned when a spec is first generated.
- Ordered: nested loops occlude > pose > arm > object_id, object fastest.
- With ``skip_failed=True`` (default) a failed trial (``feedback is None``)
  is not retried.

Poses are plain ``Pose`` values (world frame) instead of ``PoseStamped``.
"""

import itertools
from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

from virtual_tabletop.assets.rig_config import Pose

Arm = Literal["left", "right", "both"]


@dataclass(frozen=True)
class TrialSpec:
    """One trial (``tabletop_tasks`` ``TrialSpec``).

    Attributes:
        trial_number: Sequential trial number.
        object_id: Object to present.
        group_name: Robot group, e.g. ``left_manipulator``.
        object_pose: Presentation pose of the object (world frame).
        arm: Which subject arm(s) are released for the response.
        occlude: Whether the glass is occluded during the delay.
    """

    trial_number: int
    object_id: str = ""
    group_name: str = ""
    object_pose: Pose = Pose()
    arm: Arm = "both"
    occlude: bool = False


@dataclass(frozen=True)
class TrialFeedback:
    """Trial outcome (``tabletop_tasks`` ``TrialFeedback``).

    Attributes:
        reaction_time: Response time from arm release (s), or ``None``.
        timeout: True if no response arrived in the window.
    """

    reaction_time: float | None = None
    timeout: bool | None = None


def _check_reachable(
    grouped_object_ids: Mapping[str, list[str]],
    reachable: Mapping[str, set[str]] | None,
) -> None:
    if reachable is None:
        return
    for group, object_ids in grouped_object_ids.items():
        missing = set(object_ids) - set(reachable.get(group, ()))
        if missing:
            raise ValueError(
                "group_object_ids contains objects not reachable by robot "
                f"{group}: {missing}"
            )


class _AlternatingGenerator:
    """Shared pending-spec and group-alternation logic."""

    def __init__(
        self,
        grouped_object_ids: Mapping[str, list[str]],
        num_trials: int,
        skip_failed: bool,
        reachable: Mapping[str, set[str]] | None,
        prefer_feedback_group: bool = True,
    ):
        _check_reachable(grouped_object_ids, reachable)
        self._prefer_feedback_group = prefer_feedback_group
        if num_trials < 1:
            raise ValueError("num_trials must be at least 1")
        if any(len(ids) == 0 for ids in grouped_object_ids.values()):
            raise ValueError("Every robot group needs at least one object")
        self._grouped_object_ids = {
            g: list(ids) for g, ids in grouped_object_ids.items()
        }
        self._group_names = list(grouped_object_ids)
        self._num_trials = num_trials
        self._skip_failed = skip_failed
        self._trial_counter = 0
        self._last_trial_spec: dict[str, TrialSpec | None] = {
            g: None for g in self._group_names
        }
        self._last_feedback_group: str | None = None
        self._next_group_idx = 0

    @property
    def group_names(self) -> list[str]:
        return list(self._group_names)

    def __iter__(self) -> "_AlternatingGenerator":
        return self

    def _new_spec(self, group_name: str) -> TrialSpec:
        raise NotImplementedError

    def __next__(self) -> TrialSpec:
        if self._last_feedback_group is not None:
            group_name = self._last_feedback_group
            self._last_feedback_group = None
        else:
            group_name = self._group_names[self._next_group_idx]
            self._next_group_idx = (self._next_group_idx + 1) % len(
                self._group_names
            )
        pending = self._last_trial_spec[group_name]
        if pending is not None:
            return pending
        if self._trial_counter >= self._num_trials:
            raise StopIteration
        spec = self._new_spec(group_name)
        self._last_trial_spec[group_name] = spec
        self._trial_counter += 1
        return spec

    def send(self, trial_spec: TrialSpec, feedback: TrialFeedback | None):
        """Report a finished trial (``None`` feedback means it failed)."""
        pending = self._last_trial_spec[trial_spec.group_name]
        assert (
            pending is not None
            and pending.trial_number == trial_spec.trial_number
        )
        if feedback is not None or self._skip_failed:
            self._last_trial_spec[trial_spec.group_name] = None
        if self._prefer_feedback_group:
            self._last_feedback_group = trial_spec.group_name


class OrderedChoiceAlternating(_AlternatingGenerator):
    """Full-factorial ordered cycling, alternating across robot groups."""

    def __init__(
        self,
        grouped_object_ids: Mapping[str, list[str]],
        poses: list[Mapping[str, Any]],
        arms: list[Arm],
        occlude: list[bool],
        num_trials: int,
        skip_failed: bool = True,
        reachable: Mapping[str, set[str]] | None = None,
        prefer_feedback_group: bool = True,
    ):
        super().__init__(
            grouped_object_ids,
            num_trials,
            skip_failed,
            reachable,
            prefer_feedback_group,
        )
        self._poses = [Pose.from_config(p) for p in poses]
        self._arms = list(arms)
        self._occlude = list(occlude)
        self._iterators = {g: self._cycle(g) for g in self._group_names}

    def _cycle(self, group_name: str) -> Iterator[tuple]:
        return itertools.cycle(
            itertools.product(
                self._occlude,
                self._poses,
                self._arms,
                self._grouped_object_ids[group_name],
            )
        )

    def _new_spec(self, group_name: str) -> TrialSpec:
        occlude, pose, arm, object_id = next(self._iterators[group_name])
        return TrialSpec(
            trial_number=self._trial_counter,
            object_id=object_id,
            group_name=group_name,
            object_pose=pose,
            arm=arm,
            occlude=occlude,
        )


class RandomChoiceAlternating(_AlternatingGenerator):
    """Independent random sampling, alternating across robot groups."""

    def __init__(
        self,
        grouped_object_ids: Mapping[str, list[str]],
        poses: list[Mapping[str, Any]],
        arms: list[Arm],
        occlude_prob: float,
        num_trials: int,
        skip_failed: bool = True,
        reachable: Mapping[str, set[str]] | None = None,
        seed: int | None = None,
        prefer_feedback_group: bool = True,
    ):
        super().__init__(
            grouped_object_ids,
            num_trials,
            skip_failed,
            reachable,
            prefer_feedback_group,
        )
        self._poses = [Pose.from_config(p) for p in poses]
        self._arms = list(arms)
        self._occlude_prob = float(occlude_prob)
        self._rng = np.random.default_rng(seed)

    def _new_spec(self, group_name: str) -> TrialSpec:
        object_ids = self._grouped_object_ids[group_name]
        return TrialSpec(
            trial_number=self._trial_counter,
            object_id=object_ids[self._rng.integers(len(object_ids))],
            group_name=group_name,
            object_pose=self._poses[self._rng.integers(len(self._poses))],
            arm=self._arms[self._rng.integers(len(self._arms))],
            occlude=bool(self._rng.random() < self._occlude_prob),
        )


GENERATORS = {
    "OrderedChoiceAlternating": OrderedChoiceAlternating,
    "RandomChoiceAlternating": RandomChoiceAlternating,
}


def make_generator(
    class_name: str,
    kwargs: Mapping[str, Any],
    *,
    reachable: Mapping[str, set[str]] | None = None,
    seed: int | None = None,
    prefer_feedback_group: bool = True,
) -> _AlternatingGenerator:
    """Instantiate a generator by its rig class name."""
    cls = GENERATORS[class_name]
    extra = {"seed": seed} if cls is RandomChoiceAlternating else {}
    return cls(
        **dict(kwargs),
        reachable=reachable,
        prefer_feedback_group=prefer_feedback_group,
        **extra,
    )

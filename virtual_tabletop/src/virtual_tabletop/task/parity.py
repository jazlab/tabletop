"""Check the MuJoCo foraging task against a recorded session.

Runs :class:`~virtual_tabletop.task.sim.ForagingSim` with the session's task
config and its recorded reaction times, then compares:

1. **per arm:** each arm presents the same objects with the same events, in
   the same order, as in ``/tabletop/task_event``;
1b. **trial numbering:** the dispatcher and generator, replayed with the
   bag's order of trial completions, assign the bag's trial numbers, arms
   and objects. (How the two arms' trials interleave, and so which numbers
   they get, is a race decided by robot and planner timing, so the full sim
   run's interleaving is reported but not checked.)
2. **trial windows:** per-trial ``TRIAL_PRESENTED`` → ``TRIAL_DEPRESENTING``
   durations;
3. **glass:** per-trial time the smartglass was clear (``/teensy/sensor``);
4. **rewards:** count and per-trial reward-solenoid open time.

Robot motion timing (when each trial starts) comes from recorded clips and
emulated planning, so start-time offsets are reported for information but
not checked.

Usage::

    uv run vtt-parity BAG --task CONFIG
"""

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from virtual_tabletop.assets.build import load_model
from virtual_tabletop.assets.robots import ARM_SIDES
from virtual_tabletop.bags import topics
from virtual_tabletop.bags.reader import read_joint_states
from virtual_tabletop.task.config import ForagingConfig, load_foraging_config
from virtual_tabletop.task.motion import (
    ARM_JOINTS,
    ClipOrderError,
    SequenceLibrary,
    build_library,
)
from virtual_tabletop.task.responses import ScheduledResponse
from virtual_tabletop.task.sim import ForagingSim

#: Allowed difference in per-trial durations (s). The rig adds service and
#: executor latencies (~30 ms per trial in mock mode) that the sim omits.
DURATION_TOL = 0.15


@dataclass
class Check:
    name: str
    passed: bool
    detail: str


@dataclass
class ParityReport:
    checks: list[Check] = field(default_factory=list)
    info: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def __str__(self) -> str:
        lines = [
            f"{'PASS' if c.passed else 'FAIL'}  {c.name}: {c.detail}"
            for c in self.checks
        ]
        lines += [f"info  {line}" for line in self.info]
        lines.append("PARITY " + ("PASSED" if self.passed else "FAILED"))
        return "\n".join(lines)


def _intervals(t: np.ndarray, on: np.ndarray) -> list[tuple[float, float]]:
    """``[start, end)`` intervals where a sampled boolean is True."""
    out, start = [], None
    for ti, v in zip(t, on):
        if v and start is None:
            start = ti
        elif not v and start is not None:
            out.append((start, ti))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def _overlap(intervals, a: float, b: float) -> float:
    return sum(max(0.0, min(e, b) - max(s, a)) for s, e in intervals)


def _trial_windows(events) -> dict[int, tuple[float, float]]:
    start, windows = {}, {}
    for e in events:
        if e.type == topics.TaskEventType.TRIAL_PRESENTED:
            start[e.trial_number] = e.t
        elif e.type == topics.TaskEventType.TRIAL_DEPRESENTING:
            windows[e.trial_number] = (start[e.trial_number], e.t)
    return windows


def recorded_reaction_times(
    bag: str | Path, config: ForagingConfig
) -> dict[int, float | None]:
    """Per-trial reaction times implied by a session's button presses.

    The response window opens ``stimulus + delay`` after
    ``TRIAL_PRESENTED``; a press inside a trial's presentation window is its
    response.
    """
    events = topics.read_task_events(bag)
    presses = [p.t for p in topics.read_button_presses(bag)]
    lead = config.stimulus_duration + config.delay_duration
    rts = {}
    for number, (a, b) in _trial_windows(events).items():
        inside = [p for p in presses if a <= p <= b]
        rts[number] = (inside[0] - (a + lead)) if inside else None
    return rts


def _check_numbering(events, config: ForagingConfig) -> Check:
    """Replay dispatch with the recorded completion order."""
    from virtual_tabletop.task.config import reachable_object_ids
    from virtual_tabletop.task.generators import TrialFeedback, make_generator

    recorded = {}
    completions = []
    for e in events:
        if e.type == topics.TaskEventType.TRIAL_PRESENTED:
            recorded[e.trial_number] = (e.group_name, e.object_id)
        elif e.type == topics.TaskEventType.TRIAL_DEPRESENTING:
            # The presentation lock makes this the completion order.
            completions.append(e.group_name)
    generator = make_generator(
        config.generator_class,
        config.generator_kwargs,
        reachable=reachable_object_ids(),
    )
    active: dict[str, object] = {g: None for g in generator.group_names}
    dispatched = {}
    done = iter(completions)
    try:
        for spec in generator:
            if active[spec.group_name] is None:
                active[spec.group_name] = spec
                dispatched[spec.trial_number] = (
                    spec.group_name,
                    spec.object_id,
                )
                continue
            group = next(done)
            generator.send(active[group], TrialFeedback())
            active[group] = None
    except StopIteration:
        pass
    wrong = sorted(
        n
        for n in set(recorded) | set(dispatched)
        if recorded.get(n) != dispatched.get(n)
    )
    if wrong:
        n = wrong[0]
        detail = (
            f"{len(wrong)} differ; trial {n}: bag {recorded.get(n)} vs "
            f"dispatcher {dispatched.get(n)}"
        )
    else:
        detail = f"{len(recorded)} trials numbered as in the bag"
    return Check("trial numbering (dispatcher replay)", not wrong, detail)


def check_parity(bag: str | Path, task: str | Path) -> ParityReport:
    """Run the sim against ``bag`` and compare.

    The robots replay this session's own clips in recorded order
    (:class:`~virtual_tabletop.task.motion.SequenceLibrary`), so motion
    timing matches and differences point at the task logic.
    """
    report = ParityReport()
    config = load_foraging_config(task)
    model = load_model()
    library = SequenceLibrary(build_library([bag], model).clips())
    joints = read_joint_states(bag)
    initial_q = {
        side: np.array(
            [
                joints.position[0, joints.names.index(f"{side}_{j}")]
                for j in ARM_JOINTS
            ]
        )
        for side in ARM_SIDES
    }

    events = topics.read_task_events(bag)
    t0 = next(
        e.t for e in events if e.type == topics.TaskEventType.TASK_STARTED
    )
    events = [
        topics.TaskEvent(
            e.t - t0,
            e.type,
            e.session_id,
            e.trial_number,
            e.group_name,
            e.object_id,
        )
        for e in events
    ]
    rts = recorded_reaction_times(bag, config)
    sim = ForagingSim(
        model, config, library, ScheduledResponse(rts), initial_q=initial_q
    )
    try:
        sim.run()
    except ClipOrderError as error:
        report.checks.append(
            Check(
                "robot clip order", False, f"at sim {sim.time:.1f} s: {error}"
            )
        )
    else:
        report.checks.append(
            Check(
                "robot clip order",
                True,
                "sim requested the session's clips in order",
            )
        )

    # 1. per arm: same objects and events, in the same order
    def per_arm(evs):
        arms: dict[str, list[tuple]] = {}
        for e in evs:
            if e.group_name:
                arms.setdefault(e.group_name, []).append(
                    (e.type.name, e.object_id)
                )
        return arms

    rec_arms, sim_arms = per_arm(events), per_arm(sim.events)
    bad = [
        g
        for g in set(rec_arms) | set(sim_arms)
        if rec_arms.get(g) != sim_arms.get(g)
    ]
    if bad:
        g = sorted(bad)[0]
        r, m_ = rec_arms.get(g, []), sim_arms.get(g, [])
        i = next(
            (i for i, (x, y) in enumerate(zip(r, m_)) if x != y),
            min(len(r), len(m_)),
        )
        detail = (
            f"{g} differs at #{i}: bag {r[i : i + 1]} vs sim {m_[i : i + 1]}"
        )
    else:
        detail = (
            ", ".join(
                f"{g.split('_')[0]} {sum(1 for t, _ in v if t == 'TRIAL_PRESENTED')} trials"
                for g, v in sorted(rec_arms.items())
            )
            + " identical"
        )
    report.checks.append(
        Check("per-arm object/event sequence", not bad, detail)
    )

    # 2. trial numbering: the rig's dispatcher + generator, fed the bag's
    # order of trial completions, must assign the bag's trial numbers.
    report.checks.append(_check_numbering(events, config))

    # Cross-arm interleaving is a race between the arms (shared planner,
    # planning stalls); reported, not checked.
    def order(evs):
        return [(e.type.name, e.group_name) for e in evs]

    same = sum(a == b for a, b in zip(order(events), order(sim.events)))
    report.info.append(
        f"cross-arm event order: {same}/{len(events)} in the same position "
        "(the arms race for the presentation lock; not checked)"
    )

    rec_win, sim_win = _trial_windows(events), _trial_windows(sim.events)
    common = sorted(set(rec_win) & set(sim_win))

    # 2. trial windows
    diffs = np.array(
        [
            (rec_win[n][1] - rec_win[n][0]) - (sim_win[n][1] - sim_win[n][0])
            for n in common
        ]
    )
    worst = float(np.abs(diffs).max()) if len(diffs) else 0.0
    report.checks.append(
        Check(
            "trial window durations",
            worst <= DURATION_TOL,
            f"{len(common)} trials, max |bag - sim| {worst * 1000:.0f} ms "
            f"(mean {diffs.mean() * 1000:+.0f} ms)",
        )
    )

    # 3./4. glass and reward per trial
    teensy = topics.read_teensy(bag)
    rec_glass = _intervals(teensy.t - t0, teensy.smartglass_revealed)
    rec_reward = _intervals(teensy.t - t0, teensy.reward_active)
    sig = np.array(sim.signals + [(sim.time, False, False)], dtype=float)
    sim_glass = _intervals(sig[:, 0], sig[:, 1].astype(bool))
    sim_reward = _intervals(sig[:, 0], sig[:, 2].astype(bool))
    margin = 0.5
    for name, rec_iv, sim_iv in (
        ("glass clear time", rec_glass, sim_glass),
        ("reward time", rec_reward, sim_reward),
    ):
        d = np.array(
            [
                _overlap(
                    rec_iv, rec_win[n][0] - margin, rec_win[n][1] + margin
                )
                - _overlap(
                    sim_iv, sim_win[n][0] - margin, sim_win[n][1] + margin
                )
                for n in common
            ]
        )
        worst = float(np.abs(d).max()) if len(d) else 0.0
        report.checks.append(
            Check(
                f"per-trial {name}",
                worst <= DURATION_TOL,
                f"max |bag - sim| {worst * 1000:.0f} ms",
            )
        )
    n_rec = sum(1 for n in common if rts.get(n) is not None)
    n_sim = sum(
        1
        for n in common
        if sim.results[n].feedback and not sim.results[n].feedback.timeout
    )
    report.checks.append(
        Check("rewarded trials", n_rec == n_sim, f"bag {n_rec}, sim {n_sim}")
    )

    offsets = np.array([sim_win[n][0] - rec_win[n][0] for n in common])
    if len(offsets):
        report.info.append(
            "presentation start sim - bag: "
            f"first {offsets[0]:+.2f} s, last {offsets[-1]:+.2f} s "
            "(robot motion timing; not checked)"
        )
    report.info.append(
        f"task duration: bag {events[-1].t:.1f} s, sim {sim.time:.1f} s"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("bag", type=Path)
    parser.add_argument("--task", required=True, help="task config used")
    args = parser.parse_args()
    report = check_parity(args.bag, args.task)
    print(report)
    raise SystemExit(0 if report.passed else 1)

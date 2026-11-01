"""Tests for the MuJoCo foraging task: generators, config, sim, env."""

import tempfile
from pathlib import Path

import pytest
import yaml
from virtual_tabletop.task import generators
from virtual_tabletop.task.config import (
    load_foraging_config,
    reachable_object_ids,
    task_config_path,
)
from virtual_tabletop.task.generators import (
    OrderedChoiceAlternating,
    RandomChoiceAlternating,
    TrialFeedback,
)
from virtual_tabletop.task.motion import DEFAULT_LIBRARY, bridge

POSES = [{"position": [0.9525, 0.4, 0.6], "rpy": [0.0, 0.0, -1.5707]}]
GROUPED = {
    "left_manipulator": ["small_object_0", "small_object_1"],
    "right_manipulator": ["small_object_15"],
}


def _ordered(**kwargs):
    defaults = dict(
        grouped_object_ids=GROUPED,
        poses=POSES,
        arms=["left", "both"],
        occlude=[False],
        num_trials=6,
    )
    defaults.update(kwargs)
    return OrderedChoiceAlternating(**defaults)


def test_ordered_alternates_and_cycles_objects_fastest():
    gen = _ordered()
    a, b = next(gen), next(gen)
    assert (a.trial_number, a.group_name, a.object_id, a.arm) == (
        0,
        "left_manipulator",
        "small_object_0",
        "left",
    )
    assert (b.group_name, b.object_id) == (
        "right_manipulator",
        "small_object_15",
    )
    # Left is busy: asking again returns its pending spec, not a new one.
    assert next(gen) is a
    gen.send(a, TrialFeedback(timeout=True))
    c = next(gen)  # the group that reported feedback goes next
    assert (c.trial_number, c.group_name, c.object_id, c.arm) == (
        2,
        "left_manipulator",
        "small_object_1",
        "left",
    )
    gen.send(c, TrialFeedback())
    d = next(gen)  # objects exhausted: arm advances, objects restart
    assert (d.object_id, d.arm) == ("small_object_0", "both")


def test_generator_stops_after_num_trials():
    gen = _ordered(num_trials=2)
    specs = [next(gen), next(gen)]
    for spec in specs:
        gen.send(spec, TrialFeedback())
    with pytest.raises(StopIteration):
        next(gen)


def test_unreachable_objects_rejected_like_the_rig():
    with pytest.raises(ValueError, match="not reachable"):
        _ordered(
            grouped_object_ids={"left_manipulator": ["small_object_15"]},
            reachable=reachable_object_ids(),
        )


def test_random_generator_is_seeded():
    def specs(seed):
        gen = RandomChoiceAlternating(
            GROUPED, POSES, ["left", "right"], 0.5, 4, seed=seed
        )
        out = []
        for _ in range(4):
            spec = next(gen)
            out.append((spec.object_id, spec.arm, spec.occlude))
            gen.send(spec, TrialFeedback())
        return out

    assert specs(1) == specs(1)


def test_reachable_cells_split_the_grid():
    reach = reachable_object_ids()
    assert len(reach["left_manipulator"]) == 15
    assert len(reach["right_manipulator"]) == 15
    assert not reach["left_manipulator"] & reach["right_manipulator"]


def test_config_loads_and_applies_availability():
    config = load_foraging_config(
        "foraging_ordered", available={"small_object_0", "small_object_15"}
    )
    grouped = config.generator_kwargs["grouped_object_ids"]
    assert grouped == {
        "left_manipulator": ["small_object_0"],
        "right_manipulator": ["small_object_15"],
    }
    assert "small_object_1" in config.excluded["left_manipulator"]
    assert config.generator_class in generators.GENERATORS


def test_bridge_is_smooth_and_hits_endpoints():
    clip = bridge([0.0] * 6, [1.0] * 6)
    assert clip.q[0] == pytest.approx([0.0] * 6)
    assert clip.q[-1] == pytest.approx([1.0] * 6)
    assert clip.duration == pytest.approx(1.0)


needs_library = pytest.mark.skipif(
    not DEFAULT_LIBRARY.exists(),
    reason="no motion library (record sim bags, then `vtt-clips`)",
)


def _test_config(tmp: Path, **generator) -> Path:
    cfg = yaml.safe_load(task_config_path("foraging_ordered").read_text())
    kwargs = cfg["tasks"][0]["kwargs"]
    kwargs["response_timeout"] = 3.0
    kwargs["trial_generator"]["kwargs"].update(num_trials=4, **generator)
    path = tmp / "task.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return path


@needs_library
def test_sim_runs_the_task_and_rewards_flic_presses():
    from virtual_tabletop.assets.build import load_model
    from virtual_tabletop.bags.topics import TaskEventType as E
    from virtual_tabletop.task.motion import load_library
    from virtual_tabletop.task.responses import FlicSimResponse
    from virtual_tabletop.task.sim import run_task

    config = load_foraging_config(_test_config(Path(tempfile.mkdtemp())))
    sim = run_task(
        load_model(), config, load_library(), FlicSimResponse(1.0, 2.0, seed=0)
    )
    types = [e.type for e in sim.events]
    assert types[0] == E.TASK_STARTED and types[-1] == E.TASK_ENDED
    assert types.count(E.TRIAL_PRESENTED) == 4
    assert types.count(E.TRIAL_DEPRESENTING) == 4
    assert all(not r.feedback.timeout for r in sim.results.values())
    # Presentations never overlap (one presentation lock).
    windows = sorted(
        (r.presented_at, r.depresented_at) for r in sim.results.values()
    )
    assert all(a[1] <= b[0] for a, b in zip(windows, windows[1:]))
    # Objects are back on their mounts at the end.
    assert all(sim.tracker.holding(s) is None for s in ("left", "right"))


@needs_library
def test_scripted_monkey_touches_presented_objects():
    from virtual_tabletop.task.env import ForagingEnv, run_scripted_episode

    path = _test_config(Path(tempfile.mkdtemp()), arms=["both"])
    outcome = run_scripted_episode(ForagingEnv(path, max_time=300), seed=0)
    assert outcome["trials"] == 4
    assert outcome["rewarded"] == 4

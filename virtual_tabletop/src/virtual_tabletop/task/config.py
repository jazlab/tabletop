"""Load rig task configs for the MuJoCo foraging task.

Reads the same files the rig does:

- the task YAML (``tabletop_tasks/config/foraging_*.yaml``, or a copy such
  as the ones ``scripts/record_sim_trials.sh`` writes): task kwargs and the
  trial generator;
- ``commander.yaml``: which grid cells each arm can reach;
- the manipulation preflight report
  (``.cache/tabletop/manipulation_preflight.yaml``): objects whose status is
  not ``PASS`` are dropped from the generator's object lists, as
  ``tabletop_tasks/run.py`` does with ``exclude_unvalidated``.
"""

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from virtual_tabletop import paths
from virtual_tabletop.assets.rig_config import find_key, load_rig_scene

PREFLIGHT_REPORT = (
    paths.REPO_DIR / ".cache" / "tabletop" / "manipulation_preflight.yaml"
)


@dataclass(frozen=True)
class ForagingConfig:
    """``ForagingTask`` kwargs plus its trial generator.

    Attributes:
        stimulus_duration: Glass revealed before the delay (s).
        delay_duration: Delay before the response window (s).
        response_timeout: Response window (s).
        reward_duration: Reward solenoid open time (s).
        reward_sound: Whether a sound accompanies reward.
        reveal_duration: Glass reveal after a rewarded response (s).
        generator_class: ``OrderedChoiceAlternating`` or
            ``RandomChoiceAlternating``.
        generator_kwargs: The generator's kwargs (object lists already
            filtered by preflight availability).
        excluded: Objects dropped per arm group by the preflight report.
    """

    stimulus_duration: float
    delay_duration: float
    response_timeout: float
    reward_duration: float
    reward_sound: bool
    reveal_duration: float
    generator_class: str
    generator_kwargs: dict[str, Any]
    excluded: dict[str, list[str]] = field(default_factory=dict)


def task_config_path(task: str | Path) -> Path:
    """Resolve a task name (as for ``tt-launch tasks task:=``) to a file."""
    path = Path(task)
    if path.suffix != ".yaml":
        path = path.with_suffix(".yaml")
    if path.exists():
        return path
    candidate = paths.TABLETOP_TASKS_DIR / "config" / path.name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"No task config {task}")


def reachable_object_ids(
    config_file: Path = paths.COMMANDER_CONFIG,
) -> dict[str, set[str]]:
    """Map each robot group to the grid objects it can reach."""
    config = yaml.safe_load(config_file.read_text())
    params = find_key(config, "robot_interface_names")
    scene = load_rig_scene(config_file)
    by_idx = {
        f"{o.grid_idx[0]},{o.grid_idx[1]}": o.object_id
        for o in scene.grid_objects
    }
    reachable = {}
    for group, names in params.items():
        interface = find_key(config, names["manipulation_interface_name"])
        cells = interface.get("reachable_grid_indices") or []
        reachable[group] = {by_idx[c] for c in cells if c in by_idx}
    return reachable


def available_object_ids(report: Path = PREFLIGHT_REPORT) -> set[str] | None:
    """Objects the preflight report marks ``PASS`` (``None`` if no report)."""
    if not report.exists():
        return None
    data = yaml.safe_load(report.read_text())
    return {
        object_id
        for object_id, result in (data.get("objects") or {}).items()
        if result.get("status") == "PASS"
    }


def load_foraging_config(
    task: str | Path = "foraging_ordered",
    *,
    available: set[str] | None = None,
    report: Path = PREFLIGHT_REPORT,
) -> ForagingConfig:
    """Load a foraging task config.

    Args:
        task: Task name or YAML path.
        available: Objects allowed in trials. Defaults to the preflight
            report's PASS objects when the config asks for preflight
            exclusion (and the report exists).
        report: Preflight report location.
    """
    config = yaml.safe_load(task_config_path(task).read_text())
    tasks = [t for t in config["tasks"] if t["class"] == "ForagingTask"]
    if len(tasks) != 1:
        raise ValueError("Expected exactly one ForagingTask in the config")
    kwargs = copy.deepcopy(tasks[0]["kwargs"])
    generator = kwargs.pop("trial_generator")
    generator_kwargs = generator["kwargs"]

    preflight = config.get("manipulation_preflight") or {}
    if available is None and preflight.get("exclude_unvalidated", True):
        if preflight:
            available = available_object_ids(report)
    excluded = {}
    if available is not None:
        grouped = generator_kwargs["grouped_object_ids"]
        for group, object_ids in grouped.items():
            excluded[group] = [o for o in object_ids if o not in available]
            grouped[group] = [o for o in object_ids if o in available]

    return ForagingConfig(
        stimulus_duration=float(kwargs["stimulus_duration"]),
        delay_duration=float(kwargs["delay_duration"]),
        response_timeout=float(kwargs["response_timeout"]),
        reward_duration=float(kwargs["reward_duration"]),
        reward_sound=bool(kwargs.get("reward_sound", False)),
        reveal_duration=float(kwargs["reveal_duration"]),
        generator_class=generator["class"],
        generator_kwargs=generator_kwargs,
        excluded=excluded,
    )

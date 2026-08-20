"""Explicit mock-only execution checks for cached manipulation branches."""

from __future__ import annotations

import os
from copy import deepcopy
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any

import yaml

from tabletop_rig.exceptions import PlanningError
from tabletop_rig.interfaces.moveit.object_manipulation import (
    ManipulationState,
)
from tabletop_rig.manipulation_preflight import (
    _fingerprint,
    _fingerprint_payload,
    _load_cache,
    _validate_branch,
)

if TYPE_CHECKING:
    from tabletop_rig.nodes.commander import Commander


def _cache_file(filename: str) -> str:
    cache_dir = os.environ.get("TABLETOP_CACHE_DIR")
    if not cache_dir:
        raise RuntimeError("TABLETOP_CACHE_DIR is required")
    if os.path.basename(filename) != filename or not filename.endswith(
        ".yaml"
    ):
        raise ValueError("Cache filename must be a YAML basename")
    return os.path.join(cache_dir, filename)


def _branch_state(manipulator, result: dict[str, Any]):
    positions = result.get("selected_branch", {}).get("joint_positions")
    if not isinstance(positions, dict):
        raise ValueError("Cached result has no selected branch")
    state = manipulator._moveit.get_current_state()
    state.joint_positions = positions
    state.update()
    return state


def _assert_cache_compatible(
    manipulator, object_id: str, result: dict[str, Any]
) -> None:
    current_fingerprint = _fingerprint(
        _fingerprint_payload(manipulator, object_id)
    )
    if result.get("fingerprint") != current_fingerprint:
        raise ValueError("Object pose or manipulation configuration changed")


async def run(commander: Commander, config: str | None = None) -> None:
    """Execute complete cached-branch fetch/return cycles in mock mode only."""
    if not commander.param("simulate"):
        raise RuntimeError("Cached-branch execution check is MOCK ONLY")
    if config is None:
        raise ValueError("A mock-cycle config file is required")
    with open(config) as f:
        options = yaml.safe_load(f) or {}

    object_ids = list(options.get("object_ids", []))
    if not object_ids:
        raise ValueError("At least one object_id is required")
    source_name = options.get("source_report", "manipulation_preflight.yaml")
    output_name = options.get(
        "output_report", "manipulation_preflight_mock_cycles.yaml"
    )
    cycles_per_object = int(options.get("cycles_per_object", 1))
    exercise_recovery = bool(options.get("exercise_recovery", False))
    if cycles_per_object < 1:
        raise ValueError("cycles_per_object must be at least 1")
    if exercise_recovery and cycles_per_object < 2:
        raise ValueError(
            "exercise_recovery requires a normal warm-up cycle and at least "
            "one recovery cycle"
        )
    source = _load_cache(_cache_file(source_name))

    results: dict[str, Any] = {}
    for object_id in object_ids:
        cached_result = source.get("objects", {}).get(object_id)
        if (
            not isinstance(cached_result, dict)
            or cached_result.get("status") != "PASS"
        ):
            results[object_id] = {
                "status": "SKIPPED",
                "reason": "object has no passing cached branch",
            }
            continue

        arm = cached_result["arm"]
        context = commander._manipulation_contexts[arm]
        manipulator = context._manipulator
        if not manipulator._simulate:
            raise RuntimeError("Cached-branch execution check is MOCK ONLY")

        try:
            _assert_cache_compatible(manipulator, object_id, cached_result)
            branch = _branch_state(manipulator, cached_result)
            await _validate_branch(manipulator, object_id, branch)

            original_get_state_goal = manipulator._get_state_goal
            original_transition = manipulator._fetch_or_return_transition
            target_object_id = object_id
            injected_states: set[ManipulationState] = set()
            inject_this_cycle = False

            def cached_get_state_goal(
                state: ManipulationState,
                object_id: str | None,
                *,
                use_object_override: bool = True,
            ):
                if (
                    state == ManipulationState.PRE_FETCH
                    and object_id == target_object_id
                ):
                    return deepcopy(branch)
                return original_get_state_goal(
                    state,
                    object_id,
                    use_object_override=use_object_override,
                )

            async def recovery_injected_transition(
                active_object_id: str,
                next_state: ManipulationState,
            ):
                if (
                    inject_this_cycle
                    and active_object_id == target_object_id
                    and next_state
                    in (ManipulationState.PRE_FETCH, ManipulationState.FETCHED)
                    and next_state not in injected_states
                ):
                    injected_states.add(next_state)
                    raise PlanningError(
                        "Mock-only injected failure to exercise fetch "
                        f"recovery at {next_state.name}",
                        manipulator.group_name,
                    )
                return await original_transition(active_object_id, next_state)

            manipulator._get_state_goal = cached_get_state_goal
            manipulator._fetch_or_return_transition = (
                recovery_injected_transition
            )
            completed_cycles = 0
            exercised_states: set[ManipulationState] = set()
            try:
                async with commander.manipulation_context(arm) as mock_arm:
                    for cycle_index in range(cycles_per_object):
                        injected_states = set()
                        inject_this_cycle = (
                            exercise_recovery and cycle_index > 0
                        )
                        await mock_arm.reset_manipulation(reset_to_idle=True)
                        await mock_arm.fetch_object(target_object_id)
                        await mock_arm.return_object(target_object_id)
                        completed_cycles += 1
                        exercised_states.update(injected_states)
            finally:
                manipulator._fetch_or_return_transition = original_transition
                manipulator._get_state_goal = original_get_state_goal

            expected_states = {
                ManipulationState.PRE_FETCH,
                ManipulationState.FETCHED,
            }
            if exercise_recovery and exercised_states != expected_states:
                missing = sorted(
                    state.name for state in expected_states - exercised_states
                )
                raise RuntimeError(
                    f"Recovery branches were not exercised: {missing}"
                )
        except Exception as exc:
            results[object_id] = {
                "status": "FAIL",
                "arm": arm,
                "reason": f"{type(exc).__name__}: {exc}",
            }
        else:
            results[object_id] = {
                "status": "PASS",
                "arm": arm,
                "cycles_completed": completed_cycles,
                "recovery_states_exercised": sorted(
                    state.name for state in exercised_states
                ),
                "reason": (
                    f"completed {completed_cycles} full mock fetch/return "
                    "cycles"
                ),
            }

    output = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mode": "mock_execution",
        "source_report": source_name,
        "cycles_per_object": cycles_per_object,
        "exercise_recovery": exercise_recovery,
        "objects": results,
    }
    output_path = _cache_file(output_name)
    temporary_path = f"{output_path}.tmp"
    with open(temporary_path, "w") as f:
        yaml.safe_dump(output, f, sort_keys=False)
    os.replace(temporary_path, output_path)

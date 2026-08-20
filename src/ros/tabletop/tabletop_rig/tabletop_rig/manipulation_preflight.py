"""Planning-only IK branch preflight for grid-object manipulation."""

from __future__ import annotations

import hashlib
import json
import math
import os
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any

import numpy as np
import yaml
from moveit.core.robot_state import (
    RobotState,  # type: ignore[reportMissingModuleSource]
)

from tabletop_rig.interfaces.moveit.object_manipulation import (
    ManipulationState,
    ObjectManipulationInterface,
)
from tabletop_rig.interfaces.moveit.requests import PlanRequest
from tabletop_rig.manipulation_preflight_report import (
    format_report_summary,
)
from tabletop_rig.utils.ros import pose_stamped_msg

if TYPE_CHECKING:
    from tabletop_rig.nodes.commander import Commander


_CACHE_VERSION = 6
_DEFAULT_IK_SEEDS = 32
_BRANCH_VALIDATION_ATTEMPTS = 3
_CANONICAL_TOLERANCE = 1e-5
_MAX_WAYPOINT_JUMP = math.pi
_RANDOM_SEED_LIMIT = math.pi - 0.2
_CARTESIAN_GOAL_STATES = (
    ManipulationState.PRE_FETCH,
    ManipulationState.PRE_ATTACH,
    ManipulationState.ATTACH,
    ManipulationState.POST_ATTACH,
    ManipulationState.POST_FETCH,
)
_VALIDATED_STATES = (
    ManipulationState.PRE_FETCH,
    ManipulationState.PRE_ATTACH,
    ManipulationState.ATTACH,
    ManipulationState.POST_ATTACH,
    ManipulationState.POST_FETCH,
    ManipulationState.FETCHED,
)


def _pose_dict(pose_stamped) -> dict[str, Any]:
    pose = pose_stamped.pose
    return {
        "frame_id": pose_stamped.header.frame_id,
        "position": [pose.position.x, pose.position.y, pose.position.z],
        "orientation_xyzw": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }


def _canonical_joint_key(values: np.ndarray) -> tuple[int, ...]:
    """Map equivalent revolute solutions differing by 2*pi to one key."""
    wrapped = (values + math.pi) % (2.0 * math.pi) - math.pi
    return tuple(np.rint(wrapped / _CANONICAL_TOLERANCE).astype(int))


def _joint_positions(state: RobotState, group_name: str) -> np.ndarray:
    return np.asarray(state.get_joint_group_positions(group_name), dtype=float)


def _idle_state(manipulator: ObjectManipulationInterface) -> RobotState:
    idle_goal = manipulator._get_state_goal(ManipulationState.IDLE, None)
    if not isinstance(idle_goal, str):
        raise ValueError("Configured IDLE goal must be a named target state")
    return manipulator._moveit.get_target_state(
        idle_goal, manipulator.group_name
    )


def _fetched_state(manipulator: ObjectManipulationInterface) -> RobotState:
    fetched_goal = manipulator._get_state_goal(
        ManipulationState.FETCHED, None, use_object_override=False
    )
    if isinstance(fetched_goal, str):
        return manipulator._moveit.get_target_state(
            fetched_goal, manipulator.group_name
        )
    if isinstance(fetched_goal, RobotState):
        return fetched_goal
    raise ValueError("Configured FETCHED goal must be a joint-state target")


def _canonical_representative(
    manipulator: ObjectManipulationInterface, state: RobotState
) -> RobotState:
    """Store the equivalent revolute state in the principal interval."""
    group_name = manipulator.group_name
    positions = _joint_positions(state, group_name)
    normalized = (positions + math.pi) % (2.0 * math.pi) - math.pi
    representative = deepcopy(state)
    representative.joint_positions = {
        name: float(value)
        for name, value in zip(
            manipulator._moveit.get_joint_names(group_name), normalized
        )
    }
    representative.update()
    return representative


def _equivalent_representatives(
    manipulator: ObjectManipulationInterface, state: RobotState
) -> list[RobotState]:
    """Try the low-wrap form first, retaining a necessary boundary crossing."""
    canonical = _canonical_representative(manipulator, state)
    group_name = manipulator.group_name
    if np.allclose(
        _joint_positions(canonical, group_name),
        _joint_positions(state, group_name),
        atol=_CANONICAL_TOLERANCE,
        rtol=0.0,
    ):
        return [canonical]
    return [canonical, deepcopy(state)]


def _trajectory_metrics(trajectory, group_name: str) -> tuple[float, float]:
    positions = np.asarray(
        [
            _joint_positions(trajectory[i], group_name)
            for i in range(len(trajectory))
        ]
    )
    if len(positions) < 2:
        return 0.0, 0.0
    deltas = np.abs(np.diff(positions, axis=0))
    return float(np.max(deltas)), float(np.sum(deltas))


def _fingerprint_payload(
    manipulator: ObjectManipulationInterface, object_id: str
) -> dict[str, Any]:
    grid_object = manipulator._moveit.grid_objects_by_id[object_id]
    goal_configs: dict[str, Any] = {}
    resolved_goals: dict[str, Any] = {}
    for state in _CARTESIAN_GOAL_STATES:
        goal_configs[state.name] = manipulator.param(
            f"manipulation_state_goals.{state.name.lower()}"
        )
        goal = manipulator._get_state_goal(
            state, object_id, use_object_override=False
        )
        if isinstance(goal, (RobotState, str)):
            raise ValueError(
                f"Global {state.name} goal must be Cartesian for preflight"
            )
        resolved_goals[state.name] = _pose_dict(goal)

    fetched_config = manipulator.param("manipulation_state_goals.fetched")
    fetched_state = _fetched_state(manipulator)

    return {
        "cache_version": _CACHE_VERSION,
        "object_id": object_id,
        "grid_idx": list(grid_object.grid_idx),
        "object_pose": _pose_dict(grid_object.pose_stamped),
        "group_name": manipulator.group_name,
        "pose_link": manipulator.default_pose_link,
        "joint_names": manipulator._moveit.get_joint_names(
            manipulator.group_name
        ),
        "joint_representative": (
            "canonical key modulo +/-2*pi; low-wrap valid representative preferred"
        ),
        "random_seed_bounds": [-_RANDOM_SEED_LIMIT, _RANDOM_SEED_LIMIT],
        "seed_strategy": (
            "idle + deterministic random; optional cached valid branches"
        ),
        "goal_configs": goal_configs,
        "resolved_cartesian_goals": resolved_goals,
        "fetched_goal": {
            "config": fetched_config,
            "joint_positions": {
                name: float(value)
                for name, value in zip(
                    manipulator._moveit.get_joint_names(
                        manipulator.group_name
                    ),
                    _joint_positions(fetched_state, manipulator.group_name),
                )
            },
        },
        "transition_collision_allowances": {
            state.name: manipulator._transition_collisions_to_allow(
                object_id, state
            )
            for state in _CARTESIAN_GOAL_STATES
        },
        "planner_chain": [
            "configured PRE_FETCH pipeline",
            "Pilz LIN",
            "Pilz LIN",
            "Pilz LIN with attached object",
            "Pilz LIN with attached object",
            (
                "configured FETCHED pipeline with attached object; "
                f"{manipulator.param('planning.fetched_max_attempts')} "
                "fallback attempts"
            ),
        ],
        "max_waypoint_jump": _MAX_WAYPOINT_JUMP,
    }


def _fingerprint(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":")
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def _cache_path(filename: str = "manipulation_preflight.yaml") -> str:
    cache_dir = os.environ.get("TABLETOP_CACHE_DIR")
    if not cache_dir:
        raise RuntimeError(
            "TABLETOP_CACHE_DIR is required for manipulation preflight"
        )
    if os.path.basename(filename) != filename or not filename.endswith(
        ".yaml"
    ):
        raise ValueError("Preflight report filename must be a YAML basename")
    return os.path.join(
        os.path.abspath(os.path.expanduser(os.path.expandvars(cache_dir))),
        filename,
    )


def _load_cache(path: str) -> dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        loaded = yaml.safe_load(f)
    return loaded if isinstance(loaded, dict) else {}


def _write_cache(path: str, report: dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    temporary_path = f"{path}.tmp"
    with open(temporary_path, "w") as f:
        yaml.safe_dump(report, f, sort_keys=False)
    os.replace(temporary_path, path)


def load_compatible_results(
    commander: Commander,
    *,
    report_name: str = "manipulation_preflight.yaml",
    allow_real_use: bool = False,
) -> dict[str, Any]:
    """Load fingerprint-compatible branches from a mock-planning report.

    Branch generation remains mock-only. Real mode installs a branch only when
    explicitly authorized and its current mount/configuration fingerprint
    exactly matches the report.
    """
    simulate = bool(commander.param("simulate"))
    if not simulate and allow_real_use:
        commander.log(
            "Installing explicitly authorized, fingerprint-compatible "
            "preflight branches for real use"
        )

    report = _load_cache(_cache_path(report_name))
    if report.get("cache_version") != _CACHE_VERSION:
        raise RuntimeError(
            f"No compatible manipulation preflight report: {report_name}"
        )
    if report.get("mode") != "mock_planning_only":
        raise RuntimeError(
            "Manipulation preflight report was not generated by mock planning"
        )

    object_to_manipulator: dict[str, ObjectManipulationInterface] = {}
    for context in commander._manipulation_contexts.values():
        manipulator = context._manipulator
        for object_id in manipulator.reachable_object_ids:
            object_to_manipulator[object_id] = manipulator

    branches_by_group: dict[str, dict[str, dict[str, float]]] = {
        manipulator.group_name: {}
        for manipulator in object_to_manipulator.values()
    }
    available: set[str] = set()
    unavailable: set[str] = set()
    stale: set[str] = set()
    reasons: dict[str, str] = {}
    report_objects = report.get("objects", {})

    for object_id, manipulator in object_to_manipulator.items():
        result = report_objects.get(object_id)
        if not isinstance(result, dict):
            stale.add(object_id)
            reasons[object_id] = "missing from preflight report"
            continue
        try:
            current_fingerprint = _fingerprint(
                _fingerprint_payload(manipulator, object_id)
            )
        except Exception as exc:
            stale.add(object_id)
            reasons[object_id] = (
                f"cannot fingerprint: {type(exc).__name__}: {exc}"
            )
            continue
        if result.get("fingerprint") != current_fingerprint:
            stale.add(object_id)
            reasons[object_id] = "mount pose or manipulation config changed"
            continue

        status = result.get("status")
        if status == "UNAVAILABLE":
            unavailable.add(object_id)
            reasons[object_id] = str(result.get("reason", "unavailable"))
            continue
        positions = result.get("selected_branch", {}).get("joint_positions")
        if status != "PASS" or not isinstance(positions, dict):
            stale.add(object_id)
            reasons[object_id] = "preflight result has no usable PASS branch"
            continue
        expected_joints = set(
            manipulator._moveit.get_joint_names(manipulator.group_name)
        )
        if set(positions) != expected_joints:
            stale.add(object_id)
            reasons[object_id] = "cached branch joint names changed"
            continue
        available.add(object_id)
        # Real installation remains explicit and fingerprint-gated.
        if simulate or allow_real_use:
            branches_by_group[manipulator.group_name][object_id] = {
                name: float(value) for name, value in positions.items()
            }

    return {
        "report_name": report_name,
        "available_object_ids": available,
        "unavailable_object_ids": unavailable,
        "stale_object_ids": stale,
        "reasons": reasons,
        "branches_by_group": branches_by_group,
    }


def _cached_candidate(
    cached: dict[str, Any],
    fingerprint: str,
    manipulator: ObjectManipulationInterface,
) -> RobotState | None:
    """Return a cached branch only when its pose/config fingerprint matches."""
    if cached.get("fingerprint") != fingerprint:
        return None
    positions = cached.get("selected_branch", {}).get("joint_positions")
    if not isinstance(positions, dict):
        return None
    if set(positions) != set(
        manipulator._moveit.get_joint_names(manipulator.group_name)
    ):
        return None
    state = manipulator._moveit.get_current_state()
    state.joint_positions = positions
    state.update()
    return state


def _compatible_migration_candidate(
    result: dict[str, Any],
    payload: dict[str, Any],
    manipulator: ObjectManipulationInterface,
) -> RobotState | None:
    """Reuse a same-pose branch only as input to current full validation."""
    old_payload = result.get("fingerprint_payload")
    if not isinstance(old_payload, dict) or result.get("status") != "PASS":
        return None
    compatibility_fields = (
        "object_id",
        "grid_idx",
        "object_pose",
        "group_name",
        "pose_link",
        "joint_names",
        "goal_configs",
        "resolved_cartesian_goals",
        "transition_collision_allowances",
    )
    if any(
        old_payload.get(key) != payload.get(key)
        for key in compatibility_fields
    ):
        return None
    positions = result.get("selected_branch", {}).get("joint_positions")
    expected_joints = set(
        manipulator._moveit.get_joint_names(manipulator.group_name)
    )
    if not isinstance(positions, dict) or set(positions) != expected_joints:
        return None
    state = _idle_state(manipulator)
    state.joint_positions = positions
    state.update()
    return state


async def _enumerate_ik_branches(
    manipulator: ObjectManipulationInterface,
    object_id: str,
    goal,
    num_seeds: int,
    cached_state: RobotState | None,
) -> tuple[list[RobotState], Counter[str]]:
    moveit = manipulator._moveit
    group_name = manipulator.group_name
    branches: dict[tuple[int, ...], list[RobotState]] = {}
    rejected: Counter[str] = Counter()
    joint_names = moveit.get_joint_names(group_name)
    random_seed = int.from_bytes(
        hashlib.sha256(f"{group_name}:{object_id}".encode()).digest()[:8],
        byteorder="big",
    )
    rng = np.random.default_rng(random_seed)

    def add_representatives(state: RobotState) -> None:
        key = _canonical_joint_key(_joint_positions(state, group_name))
        representatives = branches.setdefault(key, [])
        for representative in _equivalent_representatives(manipulator, state):
            if not moveit.is_state_valid(
                representative, group_name, verbose=False
            ):
                continue
            positions = _joint_positions(representative, group_name)
            if any(
                np.allclose(
                    positions,
                    _joint_positions(existing, group_name),
                    atol=_CANONICAL_TOLERANCE,
                    rtol=0.0,
                )
                for existing in representatives
            ):
                continue
            representatives.append(deepcopy(representative))

    seed_states: list[RobotState] = []
    if cached_state is not None:
        add_representatives(cached_state)
    base_state = _idle_state(manipulator)
    seed_states.append(base_state)
    for _ in range(num_seeds):
        seed = deepcopy(base_state)
        random_positions = rng.uniform(
            -_RANDOM_SEED_LIMIT,
            _RANDOM_SEED_LIMIT,
            size=len(joint_names),
        )
        seed.joint_positions = {
            name: float(value)
            for name, value in zip(joint_names, random_positions)
        }
        seed.update()
        seed_states.append(seed)

    for seed in seed_states:
        if not moveit.is_state_valid(seed, group_name, verbose=False):
            rejected["random seed violates collision/limits"] += 1
            continue
        try:
            trajectory, _ = await manipulator.plan(
                request=PlanRequest(
                    goal=goal,
                    start_state=seed,
                    use_cache=False,
                )
            )
        except Exception:
            rejected["PRE_FETCH IK/planning failed"] += 1
            continue
        state = trajectory[len(trajectory) - 1]
        if not moveit.is_state_valid(state, group_name, verbose=False):
            rejected["PRE_FETCH violates collision/limits"] += 1
            continue
        add_representatives(state)

    return [
        representative
        for key in sorted(branches)
        for representative in branches[key]
    ], rejected


async def _plan_segment(
    manipulator: ObjectManipulationInterface,
    object_id: str,
    state: ManipulationState,
    start_state: RobotState,
    goal,
):
    request = PlanRequest(
        goal=goal,
        start_state=start_state,
        use_cache=False,
        planning_pipeline=(
            "linear"
            if state in _CARTESIAN_GOAL_STATES
            and state != ManipulationState.PRE_FETCH
            else None
        ),
        max_attempts=(
            manipulator.param("planning.fetched_max_attempts")
            if state == ManipulationState.FETCHED
            else None
        ),
    )
    collisions = manipulator._transition_collisions_to_allow(object_id, state)
    modified: list[tuple[str, str]] = []
    if collisions:
        modified = manipulator._moveit.allow_collision(*zip(*collisions))
    try:
        trajectory, _ = await manipulator.plan(request=request)
    finally:
        if modified:
            manipulator._moveit.disallow_collision(*zip(*modified))
    return trajectory


async def _validate_branch(
    manipulator: ObjectManipulationInterface,
    object_id: str,
    branch: RobotState,
) -> dict[str, Any]:
    group_name = manipulator.group_name
    start_state = _idle_state(manipulator)
    total_travel = 0.0
    max_jump = 0.0
    end_states: dict[str, dict[str, float]] = {}
    goals = {
        ManipulationState.PRE_FETCH: branch,
        ManipulationState.PRE_ATTACH: manipulator._get_state_goal(
            ManipulationState.PRE_ATTACH,
            object_id,
            use_object_override=False,
        ),
        ManipulationState.ATTACH: manipulator._get_state_goal(
            ManipulationState.ATTACH,
            object_id,
            use_object_override=False,
        ),
        ManipulationState.POST_ATTACH: manipulator._get_state_goal(
            ManipulationState.POST_ATTACH,
            object_id,
            use_object_override=False,
        ),
        ManipulationState.POST_FETCH: manipulator._get_state_goal(
            ManipulationState.POST_FETCH,
            object_id,
            use_object_override=False,
        ),
        ManipulationState.FETCHED: manipulator._get_state_goal(
            ManipulationState.FETCHED,
            object_id,
            use_object_override=False,
        ),
    }

    joint_names = manipulator._moveit.get_joint_names(group_name)
    moveit = manipulator._moveit
    original_scene_state = deepcopy(moveit.get_current_state())
    original_object_pose = deepcopy(
        moveit.grid_objects_by_id[object_id].pose_stamped
    )
    try:
        for state in _VALIDATED_STATES:
            try:
                trajectory = await _plan_segment(
                    manipulator, object_id, state, start_state, goals[state]
                )
            except Exception as exc:
                raise RuntimeError(
                    f"{state.name}: {type(exc).__name__}: {exc}"
                ) from exc
            segment_jump, segment_travel = _trajectory_metrics(
                trajectory, group_name
            )
            if segment_jump > _MAX_WAYPOINT_JUMP + 1e-9:
                raise ValueError(
                    f"{state.name} excessive joint wrap "
                    f"({segment_jump:.6f} rad)"
                )
            max_jump = max(max_jump, segment_jump)
            total_travel += segment_travel
            start_state = trajectory[len(trajectory) - 1]
            end_positions = _joint_positions(start_state, group_name)
            end_states[state.name] = {
                name: float(value)
                for name, value in zip(joint_names, end_positions)
            }

            if state == ManipulationState.ATTACH:
                # The executing state machine attaches only after it has
                # reached ATTACH. Temporarily make that explicit planned end
                # state the scene's current state, then use the exact same
                # move/attach helpers. This preserves the correct link-relative
                # object transform without executing the robot trajectory.
                with moveit.psm.read_write() as scene:
                    scene.current_state.joint_positions = (
                        start_state.joint_positions
                    )
                    scene.current_state.update()
                moveit.move_collision_object(
                    object_id,
                    pose_stamped_msg(
                        pose=start_state.get_pose(manipulator.attach_link),
                        frame_id=start_state.robot_model.model_frame,
                    ),
                )
                moveit.attach_collision_object(
                    object_id,
                    manipulator.attach_link,
                    touch_links=manipulator.touch_links,
                )
                if object_id not in moveit.attached_collision_object_ids:
                    raise RuntimeError(
                        "ATTACH: planning-scene attachment failed"
                    )
    finally:
        if object_id in moveit.attached_collision_object_ids:
            moveit.detach_collision_object(object_id)
        moveit.move_collision_object(object_id, original_object_pose)
        with moveit.psm.read_write() as scene:
            scene.current_state.joint_positions = (
                original_scene_state.joint_positions
            )
            scene.current_state.update()

    return {
        "score": {
            "total_joint_travel": total_travel,
            "max_waypoint_jump": max_jump,
        },
        "chain_end_states": end_states,
    }


async def _validate_branch_repeated(
    manipulator: ObjectManipulationInterface,
    object_id: str,
    branch: RobotState,
) -> dict[str, Any]:
    """Require a branch to survive repeated full-chain replanning."""
    validation: dict[str, Any] | None = None
    for _ in range(_BRANCH_VALIDATION_ATTEMPTS):
        validation = await _validate_branch(manipulator, object_id, branch)
    assert validation is not None
    return validation


async def preflight_object(
    manipulator: ObjectManipulationInterface,
    object_id: str,
    *,
    num_seeds: int,
    cached: dict[str, Any],
    seed_cache: dict[str, Any],
) -> dict[str, Any]:
    payload = _fingerprint_payload(manipulator, object_id)
    fingerprint = _fingerprint(payload)
    base_metadata = {
        "grid_idx": payload["grid_idx"],
        "arm": manipulator.group_name,
        "fingerprint": fingerprint,
        "fingerprint_payload": payload,
    }
    cached_result = cached.get("objects", {}).get(object_id, {})
    seed_result = seed_cache.get("objects", {}).get(object_id, {})
    cached_state = _cached_candidate(
        cached_result,
        fingerprint,
        manipulator,
    )
    pre_fetch_goal = manipulator._get_state_goal(
        ManipulationState.PRE_FETCH,
        object_id,
        use_object_override=False,
    )
    if isinstance(pre_fetch_goal, RobotState):
        raise ValueError(
            "Global PRE_FETCH must remain Cartesian for branch discovery"
        )

    migration_candidates = [
        _compatible_migration_candidate(result, payload, manipulator)
        for result in (cached_result, seed_result)
    ]
    candidate_states: list[RobotState] = []
    candidate_keys: set[tuple[int, ...]] = set()
    for candidate in (cached_state, *migration_candidates):
        if candidate is None:
            continue
        for representative in _equivalent_representatives(
            manipulator, candidate
        ):
            positions = _joint_positions(
                representative, manipulator.group_name
            )
            exact_key = tuple(
                np.rint(positions / _CANONICAL_TOLERANCE).astype(int)
            )
            if exact_key in candidate_keys:
                continue
            candidate_keys.add(exact_key)
            candidate_states.append(representative)

    for candidate_state in candidate_states:
        try:
            validation = await _validate_branch_repeated(
                manipulator, object_id, candidate_state
            )
        except Exception:
            continue
        else:
            joint_names = manipulator._moveit.get_joint_names(
                manipulator.group_name
            )
            selected_positions = _joint_positions(
                candidate_state, manipulator.group_name
            )
            return {
                **base_metadata,
                "ik_seed_count": 0,
                "distinct_ik_branches": 1,
                "valid_branches": 1,
                "cache_reused": True,
                "status": "PASS",
                "reason": (
                    "cached branch passed 3 consecutive full-chain replans"
                ),
                "selected_branch": {
                    "joint_positions": {
                        name: float(value)
                        for name, value in zip(joint_names, selected_positions)
                    },
                    **validation,
                },
            }

    branches, rejected = await _enumerate_ik_branches(
        manipulator,
        object_id,
        pre_fetch_goal,
        num_seeds,
        cached_state,
    )
    valid: dict[
        tuple[int, ...],
        tuple[tuple[Any, ...], RobotState, dict[str, Any]],
    ] = {}
    failures: Counter[str] = Counter()
    distinct_branch_keys = {
        _canonical_joint_key(_joint_positions(branch, manipulator.group_name))
        for branch in branches
    }
    for branch in branches:
        try:
            validation = await _validate_branch_repeated(
                manipulator, object_id, branch
            )
        except Exception as exc:
            message = str(exc).strip().splitlines()[0]
            failures[f"{type(exc).__name__}: {message}"] += 1
            continue
        positions = _joint_positions(branch, manipulator.group_name)
        branch_key = _canonical_joint_key(positions)
        score = validation["score"]
        rank = (
            score["total_joint_travel"],
            score["max_waypoint_jump"],
            tuple(np.rint(positions / _CANONICAL_TOLERANCE).astype(int)),
        )
        current = valid.get(branch_key)
        if current is None or rank < current[0]:
            valid[branch_key] = (rank, branch, validation)

    base = {
        **base_metadata,
        "ik_seed_count": num_seeds,
        "distinct_ik_branches": len(distinct_branch_keys),
        "valid_branches": len(valid),
        "cache_reused": False,
    }
    if not valid:
        reasons = failures or rejected
        reason = "; ".join(
            f"{name} ({count})" for name, count in reasons.most_common(3)
        )
        return {
            **base,
            "status": "UNAVAILABLE",
            "reason": reason or "no IK branches",
        }

    _, selected, validation = min(valid.values(), key=lambda item: item[0])
    joint_names = manipulator._moveit.get_joint_names(manipulator.group_name)
    selected_positions = _joint_positions(selected, manipulator.group_name)
    return {
        **base,
        "status": "PASS",
        "reason": (
            "validated PRE_FETCH -> PRE_ATTACH -> ATTACH -> "
            "POST_ATTACH -> POST_FETCH -> FETCHED"
        ),
        "selected_branch": {
            "joint_positions": {
                name: float(value)
                for name, value in zip(joint_names, selected_positions)
            },
            **validation,
        },
    }


async def run_preflight(
    commander: Commander,
    *,
    num_seeds: int = _DEFAULT_IK_SEEDS,
    object_ids: set[str] | None = None,
    report_name: str = "manipulation_preflight.yaml",
    seed_report_name: str | None = None,
) -> dict[str, Any]:
    """Run the dual-arm grid preflight without executing any trajectory."""
    if not commander.param("simulate"):
        raise RuntimeError(
            "Manipulation preflight is MOCK ONLY; refusing simulate=false"
        )
    if num_seeds < 1:
        raise ValueError("num_seeds must be at least 1")

    path = _cache_path(report_name)
    old_cache = _load_cache(path)
    checkpoint_path = f"{path}.partial"
    resume_cache = deepcopy(old_cache)
    checkpoint_cache = _load_cache(checkpoint_path)
    if checkpoint_cache.get("cache_version") == _CACHE_VERSION:
        resume_cache["cache_version"] = _CACHE_VERSION
        resume_cache.setdefault("objects", {}).update(
            checkpoint_cache.get("objects", {})
        )
    seed_cache = (
        _load_cache(_cache_path(seed_report_name))
        if seed_report_name is not None
        else {}
    )
    object_to_manipulator: dict[str, ObjectManipulationInterface] = {}
    for context in commander._manipulation_contexts.values():
        manipulator = context._manipulator
        if not manipulator._simulate:
            raise RuntimeError("Manipulation preflight is MOCK ONLY")
        for object_id in manipulator.reachable_object_ids:
            if object_id in object_to_manipulator:
                raise RuntimeError(
                    f"Object {object_id} is assigned to more than one arm"
                )
            object_to_manipulator[object_id] = manipulator

    report: dict[str, Any] = {
        "cache_version": _CACHE_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mode": "mock_planning_only",
        "requested_objects": (
            "all" if object_ids is None else sorted(object_ids)
        ),
        "validated_chain": [state.name for state in _VALIDATED_STATES],
        "objects": {},
    }
    for grid_idx, grid_object in sorted(
        commander._moveit.grid_objects_by_idx.items()
    ):
        object_id = grid_object.object_id
        if object_ids is not None and object_id not in object_ids:
            continue
        manipulator = object_to_manipulator.get(object_id)
        if manipulator is None:
            result = {
                "grid_idx": list(grid_idx),
                "status": "UNAVAILABLE",
                "reason": "no arm is configured to reach this mount",
            }
        else:
            commander.log(
                f"Preflighting grid_idx={grid_idx} object={object_id} "
                f"arm={manipulator.group_name}"
            )
            try:
                cached_result = resume_cache.get("objects", {}).get(
                    object_id, {}
                )
                current_fingerprint = _fingerprint(
                    _fingerprint_payload(manipulator, object_id)
                )
                if (
                    resume_cache.get("cache_version") == _CACHE_VERSION
                    and cached_result.get("fingerprint") == current_fingerprint
                    and cached_result.get("status") in {"PASS", "UNAVAILABLE"}
                ):
                    result = deepcopy(cached_result)
                    result["cache_reused"] = True
                else:
                    result = await preflight_object(
                        manipulator,
                        object_id,
                        num_seeds=num_seeds,
                        cached=resume_cache,
                        seed_cache=seed_cache,
                    )
            except Exception as exc:
                message = str(exc).strip().splitlines()[0]
                result = {
                    "grid_idx": list(grid_idx),
                    "arm": manipulator.group_name,
                    "status": "UNAVAILABLE",
                    "reason": f"{type(exc).__name__}: {message}",
                }
        report["objects"][object_id] = result
        commander.log(
            f"PREFLIGHT grid_idx={grid_idx} object={object_id}: "
            f"{result['status']} - {result['reason']}"
        )
        _write_cache(checkpoint_path, report)

    if object_ids is not None:
        missing = object_ids - set(report["objects"])
        if missing:
            raise ValueError(
                f"Unknown preflight object IDs: {sorted(missing)}"
            )

    os.replace(checkpoint_path, path)
    commander.log(f"Manipulation preflight report saved to {path}")
    print(f"\n{format_report_summary(report)}\n", flush=True)
    return report


async def run(commander: Commander, config: str | None = None) -> None:
    """Commander coroutine entry point used by ``commander.launch.py``."""
    num_seeds = _DEFAULT_IK_SEEDS
    object_ids = None
    report_name = "manipulation_preflight.yaml"
    seed_report_name = None
    if config is not None:
        with open(config) as f:
            options = yaml.safe_load(f) or {}
        num_seeds = int(options.get("num_seeds", num_seeds))
        configured_object_ids = options.get("object_ids")
        if configured_object_ids is not None:
            object_ids = set(configured_object_ids)
        report_name = options.get("report_name", report_name)
        seed_report_name = options.get("seed_report_name")
    await run_preflight(
        commander,
        num_seeds=num_seeds,
        object_ids=object_ids,
        report_name=report_name,
        seed_report_name=seed_report_name,
    )

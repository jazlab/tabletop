"""LMDB-backed fuzzy trajectory cache.

Concrete backend that persists the cache to a single LMDB file via
memory-mapped reads. Each call into the backend runs its own
transaction; the base class serializes the read-modify-write in
`__setitem__` with `self._lock`, so concurrent threads inside a single
Python process see consistent reads.

See `trajectory_cache.TrajectoryCache` for the abstract base class
that owns lifecycle (open/close), request validation, metadata-drift
detection, and the high-level `cache_trajectory` API.
"""

import bisect
import json
import os
import pickle
from collections.abc import Iterable
from typing import Any, Literal, Optional

import lmdb
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.impl.rcutils_logger import RcutilsLogger

from tabletop_rig.interfaces.moveit.requests import PlanRequest
from tabletop_rig.interfaces.moveit.trajectory_cache import (
    OrientationToleranceT,
    PositionToleranceT,
    RobotStateToleranceT,
    TrajectoryCache,
    TrajectoryCacheValue,
)
from tabletop_rig.utils.ros import (
    arrays_from_pose_msg,
    get_joint_group_positions,
)

_PICKLE_PROTOCOL = pickle.HIGHEST_PROTOCOL
_DEFAULT_MAP_SIZE: int = 2 * 1024**3  # 2 GiB of virtual address space

# Fuzzy trajectory keys are JSON-serialized dicts (always start with
# '{'), so a plain-word metadata key cannot collide with them.
_META_KEY = b"_metadata"


class LMDBTrajectoryCache(TrajectoryCache):
    """Persistent fuzzy trajectory cache backed by a single LMDB file.

    Values are pickled and stored under their fuzzy-key bytes. Each
    backend primitive opens its own LMDB transaction; the base class's
    `_lock` guards the read-modify-write in `__setitem__`.

    On metadata mismatch (e.g. `scene_hash` or tolerances changed
    across runs), the LMDB file is removed from disk and recreated in
    place by the base class's `_open_and_validate` flow.
    """

    def __init__(
        self,
        *,
        path: str,
        scene_hash: str,
        planning_frame: str,
        group_name: str,
        pose_link: Optional[str] = None,
        robot_state_tolerance: RobotStateToleranceT,
        position_tolerance: PositionToleranceT,
        orientation_tolerance: OrientationToleranceT,
        sort_by: Literal["path_length", "path_duration"] = "path_duration",
        max_trajectories: int = 1,
        map_size: int = _DEFAULT_MAP_SIZE,
        parent_logger: Optional[RcutilsLogger] = None,
    ):
        """
        Args:
            path: Absolute path to the cache file. The parent directory
                is created if it does not exist.
            map_size: Maximum virtual address space, in bytes, reserved
                for the LMDB environment. Cheap on Linux (it's only
                virtual); pick a value larger than the cache will ever
                grow.
            (Other args: see `TrajectoryCache`.)
        """
        super().__init__(
            path=path,
            scene_hash=scene_hash,
            planning_frame=planning_frame,
            group_name=group_name,
            pose_link=pose_link,
            robot_state_tolerance=robot_state_tolerance,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            sort_by=sort_by,
            max_trajectories=max_trajectories,
            parent_logger=parent_logger,
        )

        self._map_size = int(map_size)
        self._env: lmdb.Environment | None = None

        self._open_and_validate()

    # ---------------------------------------------------------------
    # Backend primitives
    # ---------------------------------------------------------------

    def _require_env(self) -> lmdb.Environment:
        self._require_open()
        env = self._env
        if env is None:
            raise RuntimeError("Trajectory cache database is not open")
        return env

    def _get_raw(self, key: bytes) -> Any:
        env = self._require_env()
        with env.begin(buffers=True) as txn:
            raw = txn.get(key)
            if raw is None:
                raise KeyError(key)
            return pickle.loads(raw)

    def _put_raw(self, key: bytes, value: Any) -> None:
        env = self._require_env()
        data = pickle.dumps(value, protocol=_PICKLE_PROTOCOL)
        with env.begin(write=True) as txn:
            txn.put(key, data)

    def _delete_raw(self, key: bytes) -> None:
        env = self._require_env()
        with env.begin(write=True) as txn:
            if not txn.delete(key):
                raise KeyError(key)

    def _contains_raw(self, key: bytes) -> bool:
        env = self._require_env()
        with env.begin(buffers=True) as txn:
            return txn.get(key) is not None

    def __len__(self) -> int:
        return self._require_env().stat()["entries"]

    # ---------------------------------------------------------------
    # Backend storage hooks (called by the base class)
    # ---------------------------------------------------------------

    def _open_impl(self) -> None:
        self._env = lmdb.open(
            self._path,
            map_size=self._map_size,
            subdir=False,
            readahead=False,
            writemap=True,
            metasync=True,
            sync=True,
            max_readers=126,
            max_dbs=0,
            create=True,
        )

    def _close_impl(self) -> None:
        try:
            if self._env is not None:
                self._env.close()
        finally:
            self._env = None

    def _clear_storage(self) -> None:
        """Close the env, remove the files on disk, and reopen empty."""
        self.close()
        for filepath in (self._path, self._path + "-lock"):
            if os.path.exists(filepath):
                os.remove(filepath)
        self.open()

    def _read_metadata(self) -> Optional[dict[str, Any]]:
        try:
            return self._get_raw(_META_KEY)
        except KeyError:
            return None

    def _write_metadata(self, metadata: dict[str, Any]) -> None:
        self._put_raw(_META_KEY, metadata)

    # ---------------------------------------------------------------
    # Mapping API (concrete; uses fuzzy keys + storage primitives)
    # ---------------------------------------------------------------

    def __setitem__(
        self, request: PlanRequest, trajectory: RobotTrajectory
    ) -> None:
        """Insert `trajectory` under the fuzzy key for `request`.

        Maintains at most `max_trajectories` per fuzzy key, evicting
        the most expensive trajectory when over the cap. The full
        read-modify-write is held under `self._lock` so concurrent
        Python threads cannot interleave.
        """
        fuzzy_key = self._fuzzy_key_bytes(request)
        value = TrajectoryCacheValue(trajectory, self._sort_by)
        self.log(f"Setting item for key: {fuzzy_key!r}", severity="DEBUG")

        with self._lock:
            try:
                values = self._get_raw(fuzzy_key)
            except KeyError:
                values: list[TrajectoryCacheValue] = []
            else:
                self._validate_db_values(values)

            bisect.insort_left(values, value)
            if len(values) > self._max_trajectories:
                values.pop(-1)

            self._put_raw(fuzzy_key, values)

    def __getitem__(self, request: PlanRequest) -> list[RobotTrajectory]:
        """Return matching trajectories for `request`, ranked best-first."""
        fuzzy_key = self._fuzzy_key_bytes(request)
        self.log(f"Getting values for key: {fuzzy_key!r}", severity="DEBUG")

        values = self._get_raw(fuzzy_key)
        self._validate_db_values(values)
        return [v.get_trajectory(request.start_state) for v in values]

    def __contains__(self, request: PlanRequest) -> bool:
        """Check if `request`'s fuzzy bin has any cached trajectories."""
        return self._contains_raw(self._fuzzy_key_bytes(request))

    def __delitem__(self, request: PlanRequest) -> None:
        """Delete every trajectory in `request`'s fuzzy bin."""
        self._delete_raw(self._fuzzy_key_bytes(request))

    def _validate_db_values(self, values: list[TrajectoryCacheValue]) -> None:
        """Sanity-check a list of values pulled from the backend."""
        if not __debug__:
            return

        assert isinstance(values, list)
        assert all(isinstance(v, TrajectoryCacheValue) for v in values)
        assert 1 <= len(values) <= self._max_trajectories

    # ---------------------------------------------------------------
    # Fuzzy-key construction
    # ---------------------------------------------------------------

    def _fuzzy_key_bytes(self, request: PlanRequest) -> bytes:
        """Compute the fuzzy bytes-key for `request`."""
        return json.dumps(
            self._fuzzy_key_dict(request), sort_keys=True
        ).encode("utf-8")

    def _fuzzy_key_dict(self, request: PlanRequest) -> dict[str, Any]:
        """Compute the fuzzy key as a dict.

        Joint angles and Cartesian coordinates are each quantized into
        integer bins via the configured tolerances. JSON-serializing
        this dict yields the bytes-key used by the storage layer.

        `group_name`, `pose_link`, and the Cartesian goal's `frame_id`
        are intentionally absent — they are stored once as cache-level
        metadata and validated against on every request, so they need
        not bloat every fuzzy bin's key.
        """
        start_state = request.start_state
        goal = request.goal

        assert start_state is not None

        positions = get_joint_group_positions(start_state, self._group_name)
        fuzzy: dict[str, Any] = {
            "start_state": self._fuzz_dict(
                positions, self._robot_state_tolerance
            ),
        }

        if isinstance(goal, RobotState):
            goal_positions = get_joint_group_positions(goal, self._group_name)
            fuzzy["goal_joints"] = self._fuzz_dict(
                goal_positions, self._robot_state_tolerance
            )
        else:
            assert isinstance(goal, PoseStamped)
            goal_position, goal_orientation = arrays_from_pose_msg(
                goal.pose, euler=False
            )
            fuzzy["goal_pose"] = {
                "position": self._fuzz_iterable(
                    goal_position, self._position_tolerance
                ),
                "orientation": self._fuzz_iterable(
                    goal_orientation, self._orientation_tolerance
                ),
            }

        return fuzzy

    @staticmethod
    def _fuzz_float(value: float, tolerance: float) -> int:
        return int(value // tolerance)

    @classmethod
    def _fuzz_iterable(
        cls,
        value: Iterable[float],
        tolerance: float | Iterable[float],
    ) -> tuple[int, ...]:
        if isinstance(tolerance, (float, int)):
            return tuple(cls._fuzz_float(v, tolerance) for v in value)
        return tuple(cls._fuzz_float(v, t) for v, t in zip(value, tolerance))

    @classmethod
    def _fuzz_dict(
        cls,
        value: dict[str, float],
        tolerance: float | dict[str, float],
    ) -> dict[str, int]:
        if isinstance(tolerance, (float, int)):
            return {k: cls._fuzz_float(v, tolerance) for k, v in value.items()}
        return {k: cls._fuzz_float(value[k], tolerance[k]) for k in value}

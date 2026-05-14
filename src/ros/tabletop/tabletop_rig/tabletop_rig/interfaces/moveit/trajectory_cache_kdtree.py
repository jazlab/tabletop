"""K-d tree trajectory cache (in-memory, feature-vector indexed).

Each `PlanRequest` is reduced to a 12- or 13-dimensional feature
vector:

- **Joint-space goal (12D)**: 6 start-state joint angles ++ 6 goal
  joint angles.
- **Cartesian goal (13D)**: 6 start-state joint angles ++ 3 goal
  position components ++ 4 goal orientation quaternion components.

Because these two spaces have different dimensions they cannot share
a single tree, so this backend holds two scipy `KDTree`s side-by-side
and dispatches on goal type at every insert and query.

Per-coordinate tolerances are folded into a fixed scale vector at
init time. Feature vectors are divided by that scale before insertion
into the tree; queries scale the query point the same way and use
`KDTree.query_ball_point(..., r=1.0, p=np.inf)`, which finds every
stored point inside the L∞ hypercube of half-side 1.0 around the
query — exactly the per-coordinate tolerance equivalence the LMDB
fuzzy backend uses.

`group_name`, `pose_link`, and Cartesian `frame_id` are dropped from
the feature vector entirely (they live in the base class as cache-
level metadata and are validated on every request).
"""

import os
import pickle
from typing import Any, Literal, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.impl.rcutils_logger import RcutilsLogger
from scipy.spatial import KDTree

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


class KDTreeTrajectoryCache(TrajectoryCache):
    """In-memory trajectory cache indexed by k-d trees over feature vectors.

    Each insert appends a single (feature, value) pair to the
    appropriate store (joint-space or Cartesian); queries do an L∞
    ball search on the scaled feature space and return the cheapest
    `max_trajectories` matches.

    Tree builds are lazy and size-invalidated: a burst of inserts
    followed by one query pays exactly one tree build, but
    interleaved insert/query workloads rebuild on every query. This
    is the natural cadence for a static k-d tree backend used as a
    benchmark target.

    On-disk format is a single pickled dict::

        {
            "metadata": {...base config keys + "joint_names"...},
            "joint_names": [...],
            "state_tree": {"features": [...], "values": [...]},
            "pose_tree":  {"features": [...], "values": [...]},
        }

    The trees themselves are not persisted; they're rebuilt lazily
    against the loaded features on the next query.

    Args:
        path: Absolute path to a pickle file. Loaded on `open()` and
            saved on `close()`.
        sample_state: Any `RobotState` from the same MoveIt setup the
            cache will be queried against. Used once at construction
            to snapshot the canonical joint ordering (from the joint
            model group's `active_joint_model_names`) so that feature
            vectors built later are consistent. The joint ordering is
            persisted as metadata, and a mismatch on reload triggers a
            full wipe via the base class's drift-detection flow.
        (See `TrajectoryCache`. `max_trajectories` caps the number of
        results returned per query — there is no per-point insert
        cap; each insert is its own tree point.)
    """

    def __init__(
        self,
        *,
        path: str,
        scene_hash: str,
        planning_frame: str,
        group_name: str,
        pose_link: Optional[str] = None,
        sample_state: RobotState,
        robot_state_tolerance: RobotStateToleranceT,
        position_tolerance: PositionToleranceT,
        orientation_tolerance: OrientationToleranceT,
        sort_by: Literal["path_length", "path_duration"] = "path_duration",
        max_trajectories: int = 1,
        parent_logger: Optional[RcutilsLogger] = None,
    ):
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

        # Snapshot joint ordering and build per-coordinate scale
        # vectors once, up front. The joint list is taken from the
        # robot model's joint model group and is stable for any
        # RobotState produced by the same MoveIt setup.
        self._joint_names: list[str] = list(
            get_joint_group_positions(sample_state, self._group_name).keys()
        )
        self._state_scale: np.ndarray = self._build_state_scale()
        self._pose_scale: np.ndarray = self._build_pose_scale()

        # Joint-space goal store: 12D features.
        self._state_features: list[np.ndarray] = []
        self._state_values: list[TrajectoryCacheValue] = []
        self._state_tree: Optional[KDTree] = None
        self._state_tree_size: int = 0

        # Cartesian goal store: 13D features.
        self._pose_features: list[np.ndarray] = []
        self._pose_values: list[TrajectoryCacheValue] = []
        self._pose_tree: Optional[KDTree] = None
        self._pose_tree_size: int = 0

        # Mirror of the metadata that was on disk at the last load
        # (None until `_open_impl` has run, or after `_clear_storage`).
        self._loaded_metadata: Optional[dict[str, Any]] = None

        self._open_and_validate()

    # ---------------------------------------------------------------
    # Metadata (extends the base with joint ordering)
    # ---------------------------------------------------------------

    @property
    def _metadata(self) -> dict[str, Any]:
        return {
            **super()._metadata,
            "joint_names": list(self._joint_names),
        }

    # ---------------------------------------------------------------
    # Scale vectors (built once in __init__)
    # ---------------------------------------------------------------

    def _joint_tol_vector(self) -> np.ndarray:
        """Return the (n_joints,) per-joint tolerance vector."""
        tol = self._robot_state_tolerance
        if isinstance(tol, dict):
            return np.array([tol[j] for j in self._joint_names], dtype=float)
        return np.full(len(self._joint_names), float(tol))

    def _build_state_scale(self) -> np.ndarray:
        """Build the 12D scale vector for joint-space goals."""
        joint_tol = self._joint_tol_vector()
        return np.concatenate([joint_tol, joint_tol])

    def _build_pose_scale(self) -> np.ndarray:
        """Build the 13D scale vector for Cartesian goals."""
        joint_tol = self._joint_tol_vector()
        pos_tol = self._position_tolerance
        if isinstance(pos_tol, (int, float)):
            pos_vec = np.full(3, float(pos_tol))
        else:
            pos_vec = np.array(pos_tol, dtype=float)
        ori_tol = self._orientation_tolerance
        if isinstance(ori_tol, (int, float)):
            ori_vec = np.full(4, float(ori_tol))
        else:
            ori_vec = np.array(ori_tol, dtype=float)
        return np.concatenate([joint_tol, pos_vec, ori_vec])

    # ---------------------------------------------------------------
    # Feature construction
    # ---------------------------------------------------------------

    def _joints_to_array(self, state: RobotState) -> np.ndarray:
        """Pull joint positions into a numpy array in canonical order."""
        positions = get_joint_group_positions(state, self._group_name)
        return np.array([positions[j] for j in self._joint_names], dtype=float)

    def _state_feature(self, request: PlanRequest) -> np.ndarray:
        """Compute the 12D feature vector for a joint-space goal request."""
        assert isinstance(request.start_state, RobotState)
        assert isinstance(request.goal, RobotState)
        start = self._joints_to_array(request.start_state)
        goal = self._joints_to_array(request.goal)
        return np.concatenate([start, goal])

    def _pose_feature(self, request: PlanRequest) -> np.ndarray:
        """Compute the 13D feature vector for a Cartesian goal request."""
        assert isinstance(request.start_state, RobotState)
        assert isinstance(request.goal, PoseStamped)
        start = self._joints_to_array(request.start_state)
        pos, ori = arrays_from_pose_msg(request.goal.pose, euler=False)
        return np.concatenate(
            [start, np.asarray(pos, dtype=float), np.asarray(ori, dtype=float)]
        )

    # ---------------------------------------------------------------
    # Lazy tree build
    # ---------------------------------------------------------------

    def _ensure_state_tree(self) -> None:
        if not self._state_features:
            self._state_tree = None
            self._state_tree_size = 0
            return
        if self._state_tree is None or self._state_tree_size != len(
            self._state_features
        ):
            scaled = np.stack(self._state_features) / self._state_scale
            self._state_tree = KDTree(scaled)
            self._state_tree_size = len(self._state_features)

    def _ensure_pose_tree(self) -> None:
        if not self._pose_features:
            self._pose_tree = None
            self._pose_tree_size = 0
            return
        if self._pose_tree is None or self._pose_tree_size != len(
            self._pose_features
        ):
            scaled = np.stack(self._pose_features) / self._pose_scale
            self._pose_tree = KDTree(scaled)
            self._pose_tree_size = len(self._pose_features)

    # ---------------------------------------------------------------
    # Backend storage hooks (called by the base class)
    # ---------------------------------------------------------------

    def _open_impl(self) -> None:
        """Load the pickle dict, if any, into the in-memory stores."""
        self._reset_in_memory_state()
        if not os.path.exists(self._path):
            return
        try:
            with open(self._path, "rb") as f:
                payload = pickle.load(f)
        except Exception as e:
            self.log(
                f"Failed to load cache from {self._path}: {e}. "
                f"Starting fresh.",
                severity="WARN",
            )
            return

        if not isinstance(payload, dict):
            self.log(
                f"Cache file {self._path} is not in the expected dict "
                f"format; starting fresh.",
                severity="WARN",
            )
            return

        state_tree = payload.get("state_tree", {}) or {}
        pose_tree = payload.get("pose_tree", {}) or {}
        self._state_features = list(state_tree.get("features", []))
        self._state_values = list(state_tree.get("values", []))
        self._pose_features = list(pose_tree.get("features", []))
        self._pose_values = list(pose_tree.get("values", []))
        self._loaded_metadata = payload.get("metadata")
        self.log(
            f"Loaded {len(self._state_features)} state + "
            f"{len(self._pose_features)} pose entries from {self._path}",
            severity="INFO",
        )

    def _close_impl(self) -> None:
        """Persist the current in-memory stores back to disk."""
        payload = {
            "metadata": self._metadata,
            "joint_names": list(self._joint_names),
            "state_tree": {
                "features": self._state_features,
                "values": self._state_values,
            },
            "pose_tree": {
                "features": self._pose_features,
                "values": self._pose_values,
            },
        }
        try:
            with open(self._path, "wb") as f:
                pickle.dump(payload, f, protocol=_PICKLE_PROTOCOL)
            self.log(
                f"Saved {len(self._state_features)} state + "
                f"{len(self._pose_features)} pose entries to {self._path}",
                severity="INFO",
            )
        except Exception as e:
            self.log(
                f"Failed to save cache to {self._path}: {e}",
                severity="ERROR",
            )

    def _clear_storage(self) -> None:
        """Wipe both the in-memory state and the pickle on disk."""
        self._reset_in_memory_state()
        if os.path.exists(self._path):
            os.remove(self._path)

    def _read_metadata(self) -> Optional[dict[str, Any]]:
        return self._loaded_metadata

    def _write_metadata(self, metadata: dict[str, Any]) -> None:
        # Persistence happens in `_close_impl`; recording the snapshot
        # here keeps `_read_metadata` in sync if it's queried again
        # before the next save.
        self._loaded_metadata = metadata

    def _reset_in_memory_state(self) -> None:
        self._state_features = []
        self._state_values = []
        self._pose_features = []
        self._pose_values = []
        self._state_tree = None
        self._state_tree_size = 0
        self._pose_tree = None
        self._pose_tree_size = 0
        self._loaded_metadata = None

    # ---------------------------------------------------------------
    # Mapping API
    # ---------------------------------------------------------------

    def __setitem__(
        self, request: PlanRequest, trajectory: RobotTrajectory
    ) -> None:
        """Append a single (feature, value) point to the appropriate store.

        The tree is invalidated implicitly via the size-check in
        `_ensure_*_tree` — the next query rebuilds it.
        """
        self._validate_request(request)
        self._require_open()
        assert isinstance(request.start_state, RobotState)

        value = TrajectoryCacheValue(trajectory, self._sort_by)

        with self._lock:
            if isinstance(request.goal, RobotState):
                feature = self._state_feature(request)
                self._state_features.append(feature)
                self._state_values.append(value)
            else:
                feature = self._pose_feature(request)
                self._pose_features.append(feature)
                self._pose_values.append(value)

    def __getitem__(self, request: PlanRequest) -> list[RobotTrajectory]:
        """Return cheapest-`max_trajectories` matches via L∞ ball query."""
        self._validate_request(request)
        self._require_open()
        assert isinstance(request.start_state, RobotState)

        if isinstance(request.goal, RobotState):
            self._ensure_state_tree()
            tree = self._state_tree
            values = self._state_values
            query_feature = self._state_feature(request)
            scale = self._state_scale
        else:
            self._ensure_pose_tree()
            tree = self._pose_tree
            values = self._pose_values
            query_feature = self._pose_feature(request)
            scale = self._pose_scale

        if tree is None:
            raise KeyError(request)

        scaled_query = query_feature / scale
        indices = tree.query_ball_point(scaled_query, r=1.0, p=np.inf)

        if not indices:
            raise KeyError(request)

        candidates = sorted(values[i] for i in indices)
        capped = candidates[: self._max_trajectories]
        return [v.get_trajectory(request.start_state) for v in capped]

    def __contains__(self, request: PlanRequest) -> bool:
        """Return True iff at least one stored point is within tolerance."""
        self._validate_request(request)
        self._require_open()
        assert isinstance(request.start_state, RobotState)

        if isinstance(request.goal, RobotState):
            self._ensure_state_tree()
            tree = self._state_tree
            query_feature = self._state_feature(request)
            scale = self._state_scale
        else:
            self._ensure_pose_tree()
            tree = self._pose_tree
            query_feature = self._pose_feature(request)
            scale = self._pose_scale

        if tree is None:
            return False
        scaled_query = query_feature / scale
        indices = tree.query_ball_point(scaled_query, r=1.0, p=np.inf)
        return bool(indices)

    def __delitem__(self, request: PlanRequest) -> None:
        """Delete every stored point within tolerance of `request`.

        Bypasses the k-d tree (which has no native delete) and rebuilds
        the affected store by filtering. Invalidates the tree.

        Raises:
            KeyError: If no stored point matches.
        """
        self._validate_request(request)
        self._require_open()
        assert isinstance(request.start_state, RobotState)

        with self._lock:
            if isinstance(request.goal, RobotState):
                features = self._state_features
                values = self._state_values
                scale = self._state_scale
                query_feature = self._state_feature(request)
            else:
                features = self._pose_features
                values = self._pose_values
                scale = self._pose_scale
                query_feature = self._pose_feature(request)

            keep_features: list[np.ndarray] = []
            keep_values: list[TrajectoryCacheValue] = []
            matched = False
            for feat, val in zip(features, values):
                if np.all(np.abs(feat - query_feature) <= scale):
                    matched = True
                    continue
                keep_features.append(feat)
                keep_values.append(val)

            if not matched:
                raise KeyError(request)

            if isinstance(request.goal, RobotState):
                self._state_features = keep_features
                self._state_values = keep_values
                self._state_tree = None
                self._state_tree_size = 0
            else:
                self._pose_features = keep_features
                self._pose_values = keep_values
                self._pose_tree = None
                self._pose_tree_size = 0

    def __len__(self) -> int:
        """Total number of stored points across both trees."""
        self._require_open()
        return len(self._state_features) + len(self._pose_features)

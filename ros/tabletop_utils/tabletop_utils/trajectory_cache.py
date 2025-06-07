import json
import logging
import os
import threading
from collections.abc import Iterable
from copy import deepcopy
from shelve import DbfilenameShelf
from typing import Any, Callable, NamedTuple, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from tabletop_utils.common import is_iterable
from tabletop_utils.ros import (
    arrays_from_pose_msg,
    pose_stamped_msg,
    robot_trajectory_from_msg,
)

RobotStateToleranceT = tuple[float, float, float, float, float, float]
PositionToleranceT = tuple[float, float, float]
OrientationToleranceT = tuple[float, float, float, float]

logger = logging.getLogger(__name__)


class FuzzyTrajectoryCacheKey(NamedTuple):
    start_state: RobotState
    goal: RobotState | PoseStamped
    pose_link: str | None


class FuzzyTrajectoryCache(DbfilenameShelf):
    """A persistent cache for fuzzy-matching trajectories.

    This cache is used to store RobotTrajectory objects. It is a subclass of
    `DbfilenameShelf` and is thread-safe.

    The cache is initialized with a metadata dictionary, a joint angle tolerance,
    a position tolerance, and an orientation tolerance.

    The cache is thread-safe.
    """

    reserved_keys = frozenset(
        (
            "metadata",
            "robot_state_tolerance",
            "position_tolerance",
            "orientation_tolerance",
        )
    )

    def __init__(
        self,
        *,
        filename: str,
        metadata: dict[str, Any],
        robot_state_tolerance: float | RobotStateToleranceT,
        position_tolerance: float | PositionToleranceT,
        orientation_tolerance: float | OrientationToleranceT,
        max_trajectories: int = 1,
        clear_cache: bool = False,
        metadata_hash_fn: Optional[Callable[[dict[str, Any]], Any]] = None,
        **kwargs: Any,
    ):
        """
        Args:
            filename: The filename of the cache.
            metadata: The metadata for the cache.
            robot_state_tolerance: The joint angle tolerance for the cache. If
                a single float is provided, it is used for all 6 joints.
            position_tolerance: The position tolerance for the cache. If a
                single float is provided, it is used for all 3 dimensions.
            orientation_tolerance: The orientation tolerance for the cache. If
                a single float is provided, it is used for all 4 dimensions.
            max_trajectories: The maximum number of trajectories to store for
                each key. If the number of trajectories for a key exceeds this
                value, the longest trajectory is removed.
            clear_cache: If True, old database file is deleted and a new one is
                created.
            metadata_hash_fn: A function to hash the metadata. If not provided,
                the metadata is compared using the `==` operator.
            **kwargs: Keyword arguments for the superclass.
        """
        if clear_cache and os.path.exists(filename + ".db"):
            os.remove(filename + ".db")

        super().__init__(filename, **kwargs)

        self._lock = threading.Lock()
        self._max_trajectories = max_trajectories

        try:
            # Initialize the tolerances and save them locally for faster access
            (
                self._robot_state_tolerance,
                self._position_tolerance,
                self._orientation_tolerance,
            ) = self._init_tolerances(
                robot_state_tolerance,
                position_tolerance,
                orientation_tolerance,
            )

            # Validate the database
            self._validate_db(
                metadata,
                self._orientation_tolerance,
                self._position_tolerance,
                self._robot_state_tolerance,
                metadata_hash_fn,
            )

        except Exception as e:
            # Close the cache file if there was an error while initializing
            self.close()
            raise e

    def _validate_db(
        self,
        metadata: dict[str, Any],
        orientation_tolerance: OrientationToleranceT,
        position_tolerance: PositionToleranceT,
        robot_state_tolerance: RobotStateToleranceT,
        metadata_hash_fn: Optional[Callable[[dict[str, Any]], Any]],
    ):
        """Validate the database.

        If the database is empty, set the new values.
        If the database is not empty, check that the values have not changed
        and/or that the hash is the same.
        If the database is not empty and the values have changed, raise an error.
        If the database is not empty and the hash is different, raise an error.
        If the database is not empty and the hash is the same, set the new values.
        """
        # Create a new dictionary with reserved keys and deepcopied values
        # This is done to avoid accidentally modifying these values in the
        # db after they are set.
        new_reserved_dict = {
            "metadata": metadata,
            "robot_state_tolerance": robot_state_tolerance,
            "position_tolerance": position_tolerance,
            "orientation_tolerance": orientation_tolerance,
        }
        new_reserved_dict = {
            key: deepcopy(value) for key, value in new_reserved_dict.items()
        }

        # Check that the keys are the same
        assert new_reserved_dict.keys() == self.reserved_keys

        # If the database is empty, set the new values
        if len(self) == 0:
            for key, value in new_reserved_dict.items():
                super().__setitem__(key, value)
            return

        # Check that the values have not changed and/or that the hash is the same
        for key, value in new_reserved_dict.items():
            try:
                old_value = super().__getitem__(key)
            except KeyError as e:
                raise KeyError(
                    f"Cache file is not empty, but key '{key}' is missing. "
                ) from e

            if key == "metadata" and metadata_hash_fn is not None:
                # If the key is metadata, check if the hash is the same
                if metadata_hash_fn(old_value) != metadata_hash_fn(value):
                    raise ValueError(
                        f"Old {key} hash in db is different from new hash: "
                        f"metadata_hash_fn({old_value}) != "
                        f"metadata_hash_fn({value})."
                    )
                if old_value != value:
                    logger.warning(
                        f"Old {key} value in db is different from new value, "
                        f"but the hash is the same, overriding: {old_value} != {value}."
                    )
            elif old_value != value:
                raise ValueError(
                    f"Old {key} value in db is different from new value: "
                    f"{old_value} != {value}."
                )

        # Set the values in the db anyway
        for key, value in new_reserved_dict.items():
            super().__setitem__(key, value)

    def _init_tolerances(
        self,
        robot_state_tolerance: Any,
        position_tolerance: Any,
        orientation_tolerance: Any,
    ) -> tuple[
        RobotStateToleranceT, PositionToleranceT, OrientationToleranceT
    ]:
        # Convert tolerances to tuples if not already
        if not is_iterable(robot_state_tolerance):
            robot_state_tolerance = [robot_state_tolerance] * 6
        if not is_iterable(position_tolerance):
            position_tolerance = [position_tolerance] * 3
        if not is_iterable(orientation_tolerance):
            orientation_tolerance = [orientation_tolerance] * 4

        # Convert to floats
        robot_state_tolerance = tuple(map(float, robot_state_tolerance))
        position_tolerance = tuple(map(float, position_tolerance))
        orientation_tolerance = tuple(map(float, orientation_tolerance))

        # Check that the lengths are correct
        if len(robot_state_tolerance) != 6:
            raise ValueError("robot_state_tolerance must be a 6-tuple")
        if len(position_tolerance) != 3:
            raise ValueError("position_tolerance must be a 3-tuple")
        if len(orientation_tolerance) != 4:
            raise ValueError("orientation_tolerance must be a 4-tuple")

        # Check that the tolerances are valid
        if any(x <= 0 for x in robot_state_tolerance):
            raise ValueError("robot_state_tolerance must be positive")
        if any(x <= 0 for x in position_tolerance):
            raise ValueError("position_tolerance must be positive")
        if any(x <= 0 for x in orientation_tolerance):
            raise ValueError("orientation_tolerance must be positive")

        return (
            robot_state_tolerance,
            position_tolerance,
            orientation_tolerance,
        )

    @property
    def metadata(self) -> dict[str, Any]:
        with self._lock:
            return super().__getitem__("metadata")

    @property
    def robot_state_tolerance(self) -> RobotStateToleranceT:
        return self._robot_state_tolerance

    @property
    def position_tolerance(self) -> PositionToleranceT:
        return self._position_tolerance

    @property
    def orientation_tolerance(self) -> OrientationToleranceT:
        return self._orientation_tolerance

    def _fuzz_float(self, value: float, tolerance: float) -> int:
        return int(value / tolerance)

    def _fuzz_iterable(
        self, value: Iterable[float], tolerance: Iterable[float]
    ) -> tuple[int, ...]:
        return tuple(
            map(lambda x, t: self._fuzz_float(x, t), value, tolerance)
        )

    def _check_cache_key(
        self,
        key: FuzzyTrajectoryCacheKey,
    ):
        """Check that the key is valid."""
        # Type check
        if not isinstance(key.start_state, RobotState):
            raise ValueError(
                f"Start state is not a RobotState: {key.start_state}"
            )
        if not isinstance(key.goal, (RobotState, PoseStamped)):
            raise ValueError(
                f"Goal is not a RobotState or PoseStamped: {key.goal}"
            )
        if key.pose_link is not None and not isinstance(key.pose_link, str):
            raise ValueError(f"Pose link is not a string: {key.pose_link}")

        # Check that the goal frame is the world frame
        # and that pose link is not provided if goal is RobotState
        if isinstance(key.goal, RobotState):
            if key.goal.robot_model.model_frame != "world":
                raise ValueError(
                    f"Goal robot model frame is not 'world': {key.goal.robot_model.model_frame}"
                )
            if key.pose_link is not None:
                raise ValueError(
                    f"Pose link is provided for a RobotState goal: {key.pose_link}"
                )
        else:
            if key.goal.header.frame_id != "world":
                raise ValueError(
                    f"Goal pose frame id is not 'world': {key.goal.header.frame_id}"
                )
            if key.pose_link is None:
                raise ValueError(
                    "Pose link is not provided for a PoseStamped goal"
                )

        # Check that the start state is in the world frame
        if key.start_state.robot_model.model_frame != "world":
            raise ValueError(
                f"Start state robot model frame is not 'world': {key.start_state.robot_model.model_frame}"
            )

    def _check_db_value(self, value: list[RobotTrajectoryMsg]):
        """Check that the value is valid."""
        assert isinstance(value, list)
        assert all(
            isinstance(trajectory_msg, RobotTrajectoryMsg)
            for trajectory_msg in value
        )
        assert 1 <= len(value) <= self._max_trajectories

    def _get_fuzzy_dict(self, key: FuzzyTrajectoryCacheKey) -> dict[str, Any]:
        """Get the fuzzy key for a given key as a dictionary.

        Applies the fuzzy key algorithm to the key, then returns a dictionary
        with the fuzzy values.

        Args:
            key: The key to get the fuzzy key for.

        Returns:
            The fuzzy key as a dictionary.
        """
        self._check_cache_key(key)

        fuzzy_key_dict = {}

        # Fuzz the start state
        fuzzy_key_dict["start_state"] = self._fuzz_iterable(
            key.start_state.joint_positions.values(),
            self.robot_state_tolerance,
        )

        # Fuzz the goal if it is a PoseStamped or a RobotState
        # If it is a string, it is a named goal and an exact match is required

        if isinstance(key.goal, RobotState):
            fuzzy_key_dict["goal"] = self._fuzz_iterable(
                key.goal.joint_positions.values(), self.robot_state_tolerance
            )
        else:
            if not isinstance(key.goal.header.frame_id, str):
                raise ValueError(
                    f"Goal pose frame id is not a string: {key.goal.header.frame_id}"
                )
            fuzzy_key_dict["goal"] = {}
            goal_position, goal_orientation = arrays_from_pose_msg(
                key.goal.pose
            )

            # Add the frame id
            fuzzy_key_dict["goal"]["frame_id"] = key.goal.header.frame_id

            # Fuzz the goal position
            fuzzy_key_dict["goal"]["position"] = self._fuzz_iterable(
                goal_position, self.position_tolerance
            )

            # Normalize then fuzz the orientation
            goal_orientation = goal_orientation / np.linalg.norm(
                goal_orientation
            )
            fuzzy_key_dict["goal"]["orientation"] = self._fuzz_iterable(
                goal_orientation, self.orientation_tolerance
            )

        # Add the planning link
        fuzzy_key_dict["pose_link"] = key.pose_link

        # Return the fuzzy key
        return fuzzy_key_dict

    def _get_fuzzy_key(self, key: FuzzyTrajectoryCacheKey) -> str:
        """Get the fuzzy key for a given key.

        Applies the fuzzy key algorithm to the key, then converts the result to a
        string.

        Args:
            key: The key to get the fuzzy key for.

        Returns:
            The fuzzy key as a string.
        """
        return json.dumps(self._get_fuzzy_dict(key))

    def __setitem__(
        self, key: FuzzyTrajectoryCacheKey, trajectory: RobotTrajectory
    ):
        fuzzy_key = self._get_fuzzy_key(key)
        trajectory_msg = trajectory.get_robot_trajectory_msg()

        with self._lock:
            try:
                trajectory_msgs = super().__getitem__(fuzzy_key)
            except KeyError:
                super().__setitem__(fuzzy_key, [trajectory_msg])
            else:
                self._check_db_value(trajectory_msgs)

                trajectory_msgs.append(trajectory_msg)
                trajectory_msgs.sort(
                    key=lambda x: robot_trajectory_from_msg(
                        x, key.start_state.robot_model
                    ).path_length
                )
                trajectory_msgs = trajectory_msgs[: self._max_trajectories]
                super().__setitem__(fuzzy_key, trajectory_msgs)

    def cache_trajectory(self, trajectory: RobotTrajectory, *, pose_link: str):
        """Cache a trajectory.

        Args:
            trajectory: The trajectory to cache.
        """
        start_state = trajectory[0]
        end_state = trajectory[len(trajectory) - 1]

        planning_frame = start_state.robot_model.model_frame

        start_pose = pose_stamped_msg(
            pose=start_state.get_pose(pose_link),
            frame_id=planning_frame,
        )
        end_pose = pose_stamped_msg(
            pose=end_state.get_pose(pose_link),
            frame_id=planning_frame,
        )

        reverse_trajectory = trajectory.__reverse__()

        keys = [
            FuzzyTrajectoryCacheKey(start_state, end_pose, pose_link),
            FuzzyTrajectoryCacheKey(start_state, end_state, None),
            FuzzyTrajectoryCacheKey(end_state, start_pose, pose_link),
            FuzzyTrajectoryCacheKey(end_state, start_state, None),
        ]
        values = [
            trajectory,
            trajectory,
            reverse_trajectory,
            reverse_trajectory,
        ]

        for key, value in zip(keys, values):
            self[key] = value

    def __getitem__(
        self, key: FuzzyTrajectoryCacheKey
    ) -> list[RobotTrajectory]:
        with self._lock:
            trajectory_msgs = super().__getitem__(self._get_fuzzy_key(key))

        self._check_db_value(trajectory_msgs)
        return [
            robot_trajectory_from_msg(
                trajectory_msg, key.start_state.robot_model
            )
            for trajectory_msg in trajectory_msgs
        ]

    def get_best_trajectory(
        self,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str | None,
    ) -> RobotTrajectory:
        key = FuzzyTrajectoryCacheKey(start_state, goal, pose_link)

        with self._lock:
            trajectory_msgs = super().__getitem__(self._get_fuzzy_key(key))

        self._check_db_value(trajectory_msgs)
        return robot_trajectory_from_msg(
            trajectory_msgs[0], start_state.robot_model
        )

    def get_trajectories(
        self,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str,
    ) -> list[RobotTrajectory]:
        key = FuzzyTrajectoryCacheKey(start_state, goal, pose_link)
        return self[key]

    def __contains__(self, key: FuzzyTrajectoryCacheKey) -> bool:
        with self._lock:
            return super().__contains__(self._get_fuzzy_key(key))

    def contains(
        self,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str,
    ) -> bool:
        key = FuzzyTrajectoryCacheKey(start_state, goal, pose_link)
        return key in self

    def __delitem__(self, key: FuzzyTrajectoryCacheKey):
        with self._lock:
            super().__delitem__(self._get_fuzzy_key(key))

    def delete_trajectory(
        self,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str,
    ):
        key = FuzzyTrajectoryCacheKey(start_state, goal, pose_link)
        super().__delitem__(self._get_fuzzy_key(key))

    def close(self):
        with self._lock:
            super().close()

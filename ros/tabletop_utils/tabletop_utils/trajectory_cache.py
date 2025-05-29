import json
import logging
import threading
from collections.abc import Iterable
from copy import deepcopy
from shelve import DbfilenameShelf
from typing import Any, Callable, Optional, cast

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore

from tabletop_utils.common import is_iterable
from tabletop_utils.ros import PlanningGoalT, arrays_from_pose_msg

FuzzyTrajectoryCacheKeyT = tuple[PlanningGoalT, RobotState]
RobotStateToleranceT = tuple[float, float, float, float, float, float]
PositionToleranceT = tuple[float, float, float]
OrientationToleranceT = tuple[float, float, float, float]

logger = logging.getLogger(__name__)


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
        *args: Any,
        metadata: dict[str, Any],
        robot_state_tolerance: float | RobotStateToleranceT,
        position_tolerance: float | PositionToleranceT,
        orientation_tolerance: float | OrientationToleranceT,
        override_db: bool = False,
        metadata_hash_fn: Optional[Callable[[dict[str, Any]], Any]] = None,
        **kwargs: Any,
    ):
        """
        Args:
            *args: Arguments for the superclass.
            metadata: The metadata for the cache.
            robot_state_tolerance: The joint angle tolerance for the cache. If
                a single float is provided, it is used for all 6 joints.
            position_tolerance: The position tolerance for the cache. If a
                single float is provided, it is used for all 3 dimensions.
            orientation_tolerance: The orientation tolerance for the cache. If
                a single float is provided, it is used for all 4 dimensions.
            override_db: If True and the metadata is different from the database,
                the database is overridden with the new values.
            metadata_hash_fn: A function to hash the metadata. If not provided,
                the metadata is compared using the `==` operator.
            **kwargs: Keyword arguments for the superclass.
        """
        super().__init__(*args, **kwargs)

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
                self._robot_state_tolerance,
                self._position_tolerance,
                self._orientation_tolerance,
                metadata_hash_fn,
            )

            # Locks for thread-safe operations
            self._lock = threading.Lock()

        except Exception as e:
            # Close the cache file if there was an error while initializing
            self.close()
            raise e

    def _validate_db(
        self,
        metadata: dict[str, Any],
        robot_state_tolerance: float | RobotStateToleranceT,
        position_tolerance: float | PositionToleranceT,
        orientation_tolerance: float | OrientationToleranceT,
        metadata_hash_fn: Optional[Callable[[dict[str, Any]], Any]],
    ):
        # Create a new dictionary with reserved keys and deepcopied values
        # This is done to avoid accidentally modifying these values in the
        # db after they are set.
        new_reserved_dict = {
            "metadata": deepcopy(metadata),
            "robot_state_tolerance": deepcopy(robot_state_tolerance),
            "position_tolerance": deepcopy(position_tolerance),
            "orientation_tolerance": deepcopy(orientation_tolerance),
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
                    "Delete the cache file and try again."
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
                        f"but the hash is the same: {old_value} != {value}."
                    )
                    logger.warning(
                        f"Overriding {key} value in db with new value: {value}."
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

    def get_fuzzy_key_dict(
        self, key: FuzzyTrajectoryCacheKeyT
    ) -> dict[str, Any]:
        """Get the fuzzy key for a given key as a dictionary.

        Applies the fuzzy key algorithm to the key, then returns a dictionary
        with the fuzzy values.

        Args:
            key: The key to get the fuzzy key for.

        Returns:
            The fuzzy key as a dictionary.
        """
        goal, start_state = key

        fuzzy_key_dict = {}

        # Fuzz the goal if it is a PoseStamped or a RobotState
        # If it is a string, it is a named goal and an exact match is required
        if isinstance(goal, str):
            fuzzy_key_dict["goal"] = goal
        elif isinstance(goal, PoseStamped):
            if not isinstance(goal.header.frame_id, str):
                raise ValueError(
                    f"Goal pose frame id is not a string: {goal.header.frame_id}"
                )
            fuzzy_key_dict["goal"] = {}
            goal_position, goal_orientation = arrays_from_pose_msg(goal.pose)

            # Add the frame id
            fuzzy_key_dict["goal"]["frame_id"] = goal.header.frame_id

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
        elif isinstance(goal, RobotState):
            fuzzy_key_dict["goal"] = self._fuzz_iterable(
                goal.joint_positions.values(), self.robot_state_tolerance
            )
        else:
            raise TypeError(
                f"Expected goal to be a string, RobotState, or PoseStamped, got {type(goal)}"
            )

        # Fuzz the start state
        fuzzy_key_dict["start_state"] = self._fuzz_iterable(
            start_state.joint_positions.values(), self.robot_state_tolerance
        )

        # Return the fuzzy key
        return fuzzy_key_dict

    def get_fuzzy_key(self, key: FuzzyTrajectoryCacheKeyT | str) -> str:
        """Get the fuzzy key for a given key.

        Applies the fuzzy key algorithm to the key, then converts the result to a
        string.

        Args:
            key: The key to get the fuzzy key for.

        Returns:
            The fuzzy key as a string.
        """
        if isinstance(key, str):
            if key in self.reserved_keys:
                raise KeyError(
                    f"'{key}' is a reserved key. Use the {key} property "
                    "to access the value."
                )
            return key
        return json.dumps(self.get_fuzzy_key_dict(key))

    def __setitem__(
        self, key: FuzzyTrajectoryCacheKeyT | str, value: RobotTrajectory
    ):
        with self._lock:
            assert isinstance(value, RobotTrajectory)
            super().__setitem__(self.get_fuzzy_key(key), value)

    def __delitem__(self, key: FuzzyTrajectoryCacheKeyT | str):
        with self._lock:
            super().__delitem__(self.get_fuzzy_key(key))

    def __contains__(self, key: FuzzyTrajectoryCacheKeyT | str) -> bool:
        with self._lock:
            return super().__contains__(self.get_fuzzy_key(key))

    def __getitem__(
        self, key: FuzzyTrajectoryCacheKeyT | str
    ) -> RobotTrajectory:
        with self._lock:
            try:
                return super().__getitem__(self.get_fuzzy_key(key))
            except KeyError as e:
                if isinstance(key[0], RobotState):
                    try:
                        trajectory = cast(
                            RobotTrajectory,
                            super().__getitem__(
                                self.get_fuzzy_key((key[1], key[0]))
                            ),
                        )
                        return cast(RobotTrajectory, reversed(trajectory))
                    except KeyError:
                        raise e
                raise e

    def close(self):
        with self._lock:
            super().close()

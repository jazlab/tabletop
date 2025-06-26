import bisect
import datetime
import json
import os
import threading
from collections.abc import Iterable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from shelve import Shelf
from typing import Any, Optional, cast

import rclpy.logging
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from tabletop_utils import dbm_sqlite3
from tabletop_utils.common import is_iterable
from tabletop_utils.ros import (
    all_close_poses_stamped,
    all_close_robot_states,
    arrays_from_pose_msg,
    pose_stamped_msg,
    robot_trajectory_from_msg,
)

RobotStateToleranceT = float | dict[str, float]
PositionToleranceT = float | tuple[float, float, float]
OrientationToleranceT = (
    float | tuple[float, float, float] | tuple[float, float, float, float]
)


logger = rclpy.logging.get_logger("trajectory_cache")


@dataclass(slots=True, frozen=True)
class FuzzyTrajectoryCacheKey:
    start_state: RobotState
    goal: RobotState | PoseStamped
    pose_link: str | None
    group_name: str

    def __post_init__(self):
        # Type check
        if not isinstance(self.start_state, RobotState):
            raise ValueError(
                f"Start state is not a RobotState: {self.start_state}"
            )
        if not isinstance(self.goal, (RobotState, PoseStamped)):
            raise ValueError(
                f"Goal is not a RobotState or PoseStamped: {self.goal}"
            )
        if self.pose_link is not None and not isinstance(self.pose_link, str):
            raise ValueError(f"Pose link is not a string: {self.pose_link}")
        if not isinstance(self.group_name, str):
            raise ValueError(f"Group name is not a string: {self.group_name}")

        # Check that the start state is in the world frame
        if self.start_state.robot_model.model_frame != "world":
            raise ValueError(
                f"Start state robot model frame is not 'world': "
                f"{self.start_state.robot_model.model_frame}"
            )

        # Check that the start state has the joint model group
        if not self.start_state.robot_model.has_joint_model_group(
            self.group_name
        ):
            raise ValueError(
                f"Start state robot model does not have joint model group: "
                f"{self.group_name}"
            )

        if isinstance(self.goal, RobotState):
            # Check that the goal state is in the world frame
            if self.goal.robot_model.model_frame != "world":
                raise ValueError(
                    f"Goal robot model frame is not 'world': "
                    f"{self.goal.robot_model.model_frame}"
                )

            # Check that the pose link is not provided if the goal is a RobotState
            if self.pose_link is not None:
                raise ValueError(
                    f"Pose link is provided for a RobotState goal: "
                    f"{self.pose_link}"
                )

            # Check that the goal state has the joint model group
            if not self.goal.robot_model.has_joint_model_group(
                self.group_name
            ):
                raise ValueError(
                    f"Goal robot model does not have joint model group: "
                    f"{self.group_name}"
                )
        else:
            # Check that the goal pose is in the world frame
            if self.goal.header.frame_id != "world":
                raise ValueError(
                    f"Goal pose frame id is not 'world': "
                    f"{self.goal.header.frame_id}"
                )

            # Check that the pose link is provided if the goal is a PoseStamped
            if self.pose_link is None:
                raise ValueError(
                    "Pose link is not provided for a PoseStamped goal"
                )

    def _fuzz_float(self, value: float, tolerance: float) -> int:
        return int(value / tolerance)

    def _fuzz_iterable(
        self, value: Iterable[float], tolerance: float | Iterable[float]
    ) -> tuple[int, ...]:
        if isinstance(tolerance, (float, int)):
            return tuple(self._fuzz_float(v, tolerance) for v in value)
        else:
            return tuple(
                self._fuzz_float(v, t) for v, t in zip(value, tolerance)
            )

    def _fuzz_dict(
        self,
        value: dict[str, float],
        tolerance: float | dict[str, float],
    ) -> dict[str, Any]:
        if isinstance(tolerance, (float, int)):
            return {
                k: self._fuzz_float(v, tolerance) for k, v in value.items()
            }
        else:
            return {k: self._fuzz_float(value[k], tolerance[k]) for k in value}

    def get_fuzzy_dict(
        self,
        robot_state_tolerance: RobotStateToleranceT,
        position_tolerance: PositionToleranceT,
        orientation_tolerance: OrientationToleranceT,
        use_euler_tolerance: bool,
    ) -> dict[str, Any]:
        """Get the fuzzy key for a given key as a dictionary.

        Applies the fuzzy key algorithm to the key, then returns a dictionary
        with the fuzzy values.

        Args:
            key: The key to get the fuzzy key for.

        Returns:
            The fuzzy key as a dictionary.
        """
        fuzzy_key_dict = {}

        # Fuzz the start state
        fuzzy_key_dict["start_state"] = self._fuzz_dict(
            self.start_state.joint_positions,
            robot_state_tolerance,
        )

        # Fuzz the goal if it is a PoseStamped or a RobotState
        # If it is a string, it is a named goal and an exact match is required

        if isinstance(self.goal, RobotState):
            fuzzy_key_dict["goal"] = self._fuzz_dict(
                self.goal.joint_positions, robot_state_tolerance
            )
        else:
            if not isinstance(self.goal.header.frame_id, str):
                raise ValueError(
                    f"Goal pose frame id is not a string: {self.goal.header.frame_id}"
                )
            fuzzy_key_dict["goal"] = {}

            # Add the frame id
            fuzzy_key_dict["goal"]["frame_id"] = self.goal.header.frame_id

            # Fuzz the goal position
            goal_position, goal_orientation = arrays_from_pose_msg(
                self.goal.pose, euler=use_euler_tolerance
            )
            fuzzy_key_dict["goal"]["position"] = self._fuzz_iterable(
                goal_position, position_tolerance
            )

            # Fuzz the orientation
            fuzzy_key_dict["goal"]["orientation"] = self._fuzz_iterable(
                goal_orientation, orientation_tolerance
            )

        # Add the planning link
        fuzzy_key_dict["pose_link"] = self.pose_link

        # Add the group name
        fuzzy_key_dict["group_name"] = self.group_name

        # Return the fuzzy key
        return fuzzy_key_dict

    def get_fuzzy_string(self, *args: Any, **kwargs: Any) -> str:
        """Get the fuzzy key for a given key.

        Applies the fuzzy key algorithm to the key, then converts the result to a
        string.

        Args:
            *args: Arguments to pass to get_fuzzy_dict.
            **kwargs: Keyword arguments to pass to get_fuzzy_dict.

        Returns:
            The fuzzy key as a string.
        """
        return json.dumps(self.get_fuzzy_dict(*args, **kwargs), sort_keys=True)


@dataclass(slots=True, frozen=True, eq=False)
class FuzzyTrajectoryCacheValue:
    trajectory_msg: RobotTrajectoryMsg
    group_name: str
    path_length: float

    def __init__(self, trajectory: RobotTrajectory):
        if not isinstance(trajectory, RobotTrajectory):
            raise ValueError(
                f"Trajectory is not a RobotTrajectory: {trajectory}"
            )
        object.__setattr__(
            self, "trajectory_msg", trajectory.get_robot_trajectory_msg()
        )
        object.__setattr__(
            self, "group_name", trajectory.joint_model_group_name
        )
        object.__setattr__(self, "path_length", trajectory.path_length)

    def get_trajectory(self, state: RobotState) -> RobotTrajectory:
        return robot_trajectory_from_msg(
            self.trajectory_msg, state, self.group_name
        )

    def __lt__(self, other: "FuzzyTrajectoryCacheValue") -> bool:
        return self.path_length < other.path_length


class FuzzyTrajectoryCache(Shelf):
    """A persistent cache for fuzzy-matching trajectories.

    This cache is used to store RobotTrajectory objects. It is a subclass of
    Sqlite3Shelf, which uses the shelve module to interface with the sqlite3
    database.

    The cache is initialized with a rig hash, a joint angle tolerance, a position
    tolerance, and an orientation tolerance.

    The cache is made thread-safe by using a lock to synchronize access to the
    database.
    """

    _symlink_filename: str = "cache.db"

    _robot_state_tolerance: RobotStateToleranceT
    _position_tolerance: PositionToleranceT
    _orientation_tolerance: OrientationToleranceT
    _use_euler_tolerance: bool
    _max_trajectories: int

    def __init__(
        self,
        *,
        path: str,
        rig_hash: str,
        robot_state_tolerance: RobotStateToleranceT,
        position_tolerance: PositionToleranceT,
        orientation_tolerance: OrientationToleranceT,
        use_euler_tolerance: bool = True,
        max_trajectories: int = 1,
        new_cache: bool = False,
    ):
        """
        Args:
            path: The path of the cache.
            rig_hash: The hash of the rig.
            robot_state_tolerance: The joint angle tolerance for the cache. If
                a single float is provided, it is used for all 6 joints.
            position_tolerance: The position tolerance for the cache. If a
                single float is provided, it is used for all 3 dimensions.
            orientation_tolerance: The orientation tolerance for the cache. If
                a single float is provided, it is used for all 4 dimensions.
            use_euler_tolerance: Whether to use euler tolerance instead of
                quaternion tolerance.
            max_trajectories: The maximum number of trajectories to store for
                each key. If the number of trajectories for a key exceeds this
                value, the longest trajectory is removed.
            new_cache: If True, a new, empty cache file is created and the
                symlink is updated. If False, the old cache file (either the
                provided filename or the symlinked cache file) is used.
        """
        # Initialize the path
        path = os.path.abspath(os.path.expanduser(path))
        if not os.path.exists(path):
            new_cache = True
            _, ext = os.path.splitext(path)
            if ext == "":
                os.makedirs(path)
            elif ext != ".db":
                raise ValueError(f"Invalid cache file extension: {ext}")
        elif not os.path.isdir(path) and new_cache:
            raise ValueError(
                "Cannot create a new cache file if path is not a directory"
            )

        if os.path.isdir(path):
            symlink_path = os.path.join(path, self._symlink_filename)

        new_path: str | None = None
        old_symlink_target: str | None = None

        try:
            if new_cache:
                assert os.path.isdir(path)

                # Create a new, empty cache file with a timestamp
                timestamp = datetime.datetime.now().strftime(
                    "%Y-%m-%d-%H-%M-%S"
                )
                filename = f"{timestamp}.db"
                new_path = os.path.join(path, filename)
                if os.path.exists(new_path):
                    raise FileExistsError(
                        f"Cache file already exists: {new_path}"
                    )

                # Update the symlink
                if os.path.islink(symlink_path):
                    old_symlink_target = os.readlink(symlink_path)
                    os.remove(symlink_path)
                elif os.path.exists(symlink_path):
                    raise RuntimeError(
                        f"Symlink path exists but is not a symlink: {symlink_path}"
                    )
                os.symlink(filename, symlink_path)
                self._db_path = symlink_path
            elif os.path.isdir(path):
                # Check that current symlink is valid
                if not os.path.islink(symlink_path):
                    raise RuntimeError(
                        f"Symlink path does not exist or is not a symlink: "
                        f"{symlink_path}"
                    )
                if not os.path.exists(symlink_path):
                    raise FileNotFoundError(
                        f"Symlinked database file does not exist: {symlink_path}"
                    )
                self._db_path = symlink_path
            else:
                self._db_path = path

            # Initialize the lock and the closed flag
            self._lock = threading.Lock()
            self._closed = True

            # Initialize the database
            self.open(flag="c")
            with self:
                # Initialize the instance variables
                self._max_trajectories = max_trajectories

                # Initialize the tolerances and save them locally for faster access
                (
                    self._robot_state_tolerance,
                    self._position_tolerance,
                    self._orientation_tolerance,
                ) = self._init_tolerances(
                    robot_state_tolerance,
                    position_tolerance,
                    orientation_tolerance,
                    use_euler_tolerance,
                )
                self._use_euler_tolerance = use_euler_tolerance

                # Validate the database
                self._validate_db(rig_hash)

                logger.info(
                    f"Initialized trajectory cache with orientation tolerance "
                    f"{self._orientation_tolerance}, position tolerance "
                    f"{self._position_tolerance}, robot state tolerance "
                    f"{self._robot_state_tolerance}, and max trajectories "
                    f"{max_trajectories}."
                )
        except Exception:
            # Clean up the cache file if there was an error while initializing
            if new_cache:
                if new_path is not None and os.path.exists(new_path):
                    os.remove(new_path)
                if os.path.islink(symlink_path):
                    os.remove(symlink_path)
                if old_symlink_target is not None:
                    os.symlink(old_symlink_target, symlink_path)
            raise

    def _init_tolerances(
        self,
        robot_state_tolerance: Any,
        position_tolerance: Any,
        orientation_tolerance: Any,
        use_euler_tolerance: bool,
    ) -> tuple[
        RobotStateToleranceT, PositionToleranceT, OrientationToleranceT
    ]:
        """Validate and initialize the tolerances."""
        if isinstance(robot_state_tolerance, Mapping):
            robot_state_tolerance = {
                k: float(v) for k, v in robot_state_tolerance.items()
            }
            if len(robot_state_tolerance) != 6:
                raise ValueError("robot_state_tolerance must be a 6-tuple")
            if any(x <= 0 for x in robot_state_tolerance.values()):
                raise ValueError("robot_state_tolerance must be positive")
        elif robot_state_tolerance <= 0:
            raise ValueError("robot_state_tolerance must be positive")

        if is_iterable(position_tolerance):
            position_tolerance = tuple(map(float, position_tolerance))
            if len(position_tolerance) != 3:
                raise ValueError("position_tolerance must be a 3-tuple")
            if any(x <= 0 for x in position_tolerance):
                raise ValueError("position_tolerance must be positive")
        elif position_tolerance <= 0:
            raise ValueError("position_tolerance must be positive")

        if is_iterable(orientation_tolerance):
            orientation_tolerance = tuple(map(float, orientation_tolerance))
            if (use_euler_tolerance and len(orientation_tolerance) != 3) or (
                not use_euler_tolerance and len(orientation_tolerance) != 4
            ):
                raise ValueError(
                    "orientation_tolerance must be a 3-tuple if use_euler_tolerance "
                    "is True, and a 4-tuple if use_euler_tolerance is False, "
                    f"but got {len(orientation_tolerance)}-tuple"
                )
            if any(x <= 0 for x in orientation_tolerance):
                raise ValueError("orientation_tolerance must be positive")
        elif orientation_tolerance <= 0:
            raise ValueError("orientation_tolerance must be positive")

        return (
            robot_state_tolerance,
            position_tolerance,
            orientation_tolerance,
        )

    def _validate_db(self, rig_hash: str):
        """Validate the database against the new values."""
        # Create a new dictionary with reserved keys and deepcopied values
        # This is done to avoid accidentally modifying these values in the
        # db after they are set.
        metadata = {
            "rig_hash": rig_hash,
            "robot_state_tolerance": deepcopy(self._robot_state_tolerance),
            "position_tolerance": self._position_tolerance,
            "orientation_tolerance": self._orientation_tolerance,
            "use_euler_tolerance": self._use_euler_tolerance,
            "max_trajectories": self._max_trajectories,
        }

        # If the database is empty, set the new values
        if len(self) == 0:
            for key, value in metadata.items():
                super().__setitem__(key, value)
            return

        # Check that the values have not changed and/or that the hash is the same
        for key, value in metadata.items():
            try:
                old_value = super().__getitem__(key)
            except KeyError as e:
                raise KeyError(
                    f"Cache file is not empty, but key '{key}' is missing. "
                ) from e

            if old_value != value:
                raise ValueError(
                    f"Old {key} value in db is different from new value: "
                    f"{old_value} != {value}."
                )

        # Set the values in the db anyway
        for key, value in metadata.items():
            super().__setitem__(key, value)

    @property
    def rig_hash(self) -> str:
        """Rig hash stored in the underlying database."""
        with self._lock:
            return super().__getitem__("rig_hash")

    @property
    def robot_state_tolerance(self) -> RobotStateToleranceT:
        """Robot state tolerance (stored in memory for faster access)"""
        return self._robot_state_tolerance

    @property
    def position_tolerance(self) -> PositionToleranceT:
        """Position tolerance (stored in memory for faster access)"""
        return self._position_tolerance

    @property
    def orientation_tolerance(self) -> OrientationToleranceT:
        """Orientation tolerance (stored in memory for faster access)"""
        return self._orientation_tolerance

    @property
    def use_euler_tolerance(self) -> bool:
        """Whether to use euler tolerance instead of quaternion tolerance (stored in memory for faster access)"""
        return self._use_euler_tolerance

    @property
    def db_path(self) -> str:
        """The path to the database file."""
        return self._db_path

    def get_fuzzy_key(self, key: FuzzyTrajectoryCacheKey) -> str:
        """Get the fuzzy key for a given key and tolerances."""
        return key.get_fuzzy_string(
            self.robot_state_tolerance,
            self.position_tolerance,
            self.orientation_tolerance,
            self.use_euler_tolerance,
        )

    def _validate_db_values(self, values: list[FuzzyTrajectoryCacheValue]):
        """Validate that the values stored in the database are valid."""
        if not __debug__:
            return

        assert isinstance(values, list)
        assert all(isinstance(v, FuzzyTrajectoryCacheValue) for v in values)
        assert 1 <= len(values) <= self._max_trajectories

    def _validate_trajectory_quality(
        self,
        trajectory: RobotTrajectory,
        *,
        pose_link: str,
        true_start_state: Optional[RobotState] = None,
        true_end_state: Optional[RobotState] = None,
        true_goal: Optional[RobotState | PoseStamped] = None,
    ):
        """Validate that the trajectory is valid for the given ground truth states and pose."""
        trajectory_start_state: RobotState = trajectory[0]
        trajectory_end_state: RobotState = trajectory[len(trajectory) - 1]
        trajectory_start_pose = pose_stamped_msg(
            pose=trajectory_start_state.get_pose(pose_link),
            frame_id=trajectory_start_state.robot_model.model_frame,
        )
        trajectory_end_pose = pose_stamped_msg(
            pose=trajectory_end_state.get_pose(pose_link),
            frame_id=trajectory_end_state.robot_model.model_frame,
        )

        # Check that the trajectory start state and pose
        # are close to the true start state and pose
        if true_start_state is not None:
            if not all_close_robot_states(
                trajectory_start_state,
                true_start_state,
                position_tolerance=self.robot_state_tolerance,
            ):
                raise ValueError(
                    "True start state is not close to the trajectory start state. "
                    f"True start state joint positions: {true_start_state.joint_positions}, "
                    f"Trajectory start state joint positions: {trajectory_start_state.joint_positions}"
                )

            true_start_pose = pose_stamped_msg(
                pose=true_start_state.get_pose(pose_link),
                frame_id=true_start_state.robot_model.model_frame,
            )
            if not all_close_poses_stamped(
                trajectory_start_pose,
                true_start_pose,
                position_tolerance=self.position_tolerance,
                orientation_tolerance=self.orientation_tolerance,
                use_euler_tolerance=self.use_euler_tolerance,
            ):
                raise ValueError(
                    "True start pose is not close to the trajectory start pose. "
                    f"True start pose: {true_start_pose}, "
                    f"Trajectory start pose: {trajectory_start_pose}"
                )

        # Check that the trajectory end state and pose
        # are close to the true end state and pose
        if true_end_state is not None:
            if not all_close_robot_states(
                trajectory_end_state,
                true_end_state,
                position_tolerance=self.robot_state_tolerance,
            ):
                raise ValueError(
                    "True end state is not close to the trajectory end state. "
                    f"True end state joint positions: {true_end_state.joint_positions}, "
                    f"Trajectory end state joint positions: {trajectory_end_state.joint_positions}"
                )

            true_end_pose = pose_stamped_msg(
                pose=true_end_state.get_pose(pose_link),
                frame_id=true_end_state.robot_model.model_frame,
            )
            if not all_close_poses_stamped(
                trajectory_end_pose,
                true_end_pose,
                position_tolerance=self.position_tolerance,
                orientation_tolerance=self.orientation_tolerance,
                use_euler_tolerance=self.use_euler_tolerance,
            ):
                raise ValueError(
                    "True end state pose is not close to the trajectory end pose. "
                    f"True end state pose: {true_end_pose}, "
                    f"Trajectory end pose: {trajectory_end_pose}"
                )

        # Check that the trajectory end state and pose
        # are close to the true goal
        if true_goal is not None:
            if isinstance(true_goal, RobotState):
                if not all_close_robot_states(
                    trajectory_end_state,
                    true_goal,
                    position_tolerance=self.robot_state_tolerance,
                ):
                    raise ValueError(
                        "True goal state is not close to the trajectory end state. "
                        f"True goal state joint positions: {true_goal.joint_positions}, "
                        f"Trajectory end state joint positions: {trajectory_end_state.joint_positions}"
                    )
                true_goal_pose = pose_stamped_msg(
                    pose=true_goal.get_pose(pose_link),
                    frame_id=true_goal.robot_model.model_frame,
                )
            else:
                true_goal_pose = true_goal

            if not all_close_poses_stamped(
                trajectory_end_pose,
                true_goal_pose,
                position_tolerance=self.position_tolerance,
                orientation_tolerance=self.orientation_tolerance,
                use_euler_tolerance=self.use_euler_tolerance,
            ):
                raise ValueError(
                    "True goal pose is not close to the trajectory end pose. "
                    f"True goal pose: {true_goal_pose}, "
                    f"Trajectory end pose: {trajectory_end_pose}"
                )

    def __setitem__(
        self, key: FuzzyTrajectoryCacheKey, value: FuzzyTrajectoryCacheValue
    ):
        """Set an item in the database.

        If the key is not in the database, a new list is created.
        If the key is in the database, the value is inserted in the list.
        If the list has more than `max_trajectories` elements, the trajectory
        with the longest path length is removed.

        Args:
            key: The key to set.
            value: The value to set.
        """
        fuzzy_key = self.get_fuzzy_key(key)
        logger.debug(f"Setting item for key: {fuzzy_key}")

        with self._lock:
            try:
                values = cast(
                    list[FuzzyTrajectoryCacheValue],
                    super().__getitem__(fuzzy_key),
                )
            except KeyError:
                values = []
            else:
                self._validate_db_values(values)

            bisect.insort_left(values, value)
            if len(values) > self._max_trajectories:
                values.pop()

            super().__setitem__(fuzzy_key, values)

    def cache_trajectory(
        self,
        trajectory: RobotTrajectory,
        *,
        pose_link: str,
        true_start_state: Optional[RobotState] = None,
        true_end_state: Optional[RobotState] = None,
        true_goal: Optional[RobotState | PoseStamped] = None,
        _cache_reverse: bool = True,
    ):
        """Cache a trajectory.

        Args:
            trajectory: The trajectory to cache.
            true_start_state: The true start state of the robot, to check the
                quality of the trajectory. If not provided, the trajectory is
                cached without checking the quality.
            true_end_state: The true final state of the robot, to check the
                quality of the trajectory. If not provided, the trajectory is
                cached without checking the quality.
            true_goal_pose: The true goal pose of the robot, to check the quality
                of the trajectory. If not provided, the trajectory is cached
                without checking the quality.
            pose_link: The link to use for the pose if the goal is a PoseStamped.
                If the goal is a RobotState, the pose link is not used to cache
                the trajectory, but it is used to validate the trajectory
                quality and to cache additional entries with the same start
                state and the PoseStamped goal associated with the pose link
                for the given goal state.
            _cache_reverse: Whether to cache the reverse trajectory
                (used internally).
        """
        # Check that the trajectory is valid
        try:
            self._validate_trajectory_quality(
                trajectory,
                pose_link=pose_link,
                true_start_state=true_start_state,
                true_end_state=true_end_state,
                true_goal=true_goal,
            )
        except ValueError as e:
            logger.warning(f"Trajectory is not valid: {e}. Skipping cache.")
            return

        # Extract the start and end states, the end pose, and the group name
        # from the trajectory
        start_state: RobotState = trajectory[0]
        end_state: RobotState = trajectory[len(trajectory) - 1]
        end_pose = pose_stamped_msg(
            pose=end_state.get_pose(pose_link),
            frame_id=end_state.robot_model.model_frame,
        )
        group_name = trajectory.joint_model_group_name

        # Start state to end state key
        state_key = FuzzyTrajectoryCacheKey(
            start_state, end_state, None, group_name
        )
        # Start state to end pose key
        pose_key = FuzzyTrajectoryCacheKey(
            start_state, end_pose, pose_link, group_name
        )

        # Cache the trajectory
        value = FuzzyTrajectoryCacheValue(trajectory)
        self.__setitem__(pose_key, value)
        self.__setitem__(state_key, value)

        if _cache_reverse:
            self.cache_trajectory(
                trajectory.reverse(),
                pose_link=pose_link,
                true_start_state=true_end_state,
                true_end_state=true_start_state,
                _cache_reverse=False,
            )

    def __getitem__(
        self, key: FuzzyTrajectoryCacheKey
    ) -> list[FuzzyTrajectoryCacheValue]:
        """Get the values for a given key.

        Args:
            key: The key to get the values for.

        Returns:
            The sorted list of values for the given key.
        """
        fuzzy_key = self.get_fuzzy_key(key)
        logger.debug(f"Getting values for key: {fuzzy_key}")
        with self._lock:
            values = super().__getitem__(fuzzy_key)

        self._validate_db_values(values)
        return values

    def get_best_trajectory(
        self,
        *,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str,
        group_name: str,
    ) -> RobotTrajectory:
        """Get the best trajectory for a given key.

        Args:
            start_state: The start state of the trajectory.
            goal: The goal of the trajectory.
            pose_link: The link to use for the pose if the goal is a PoseStamped.
                If the goal is a RobotState, the pose link is not used to
                retrieve the trajectory, but it is used to validate the
                trajectory quality.
            group_name: The group name of the trajectory.

        Returns:
            The best trajectory for the given key.
        """
        if isinstance(goal, RobotState):
            cache_pose_link = None
        else:
            cache_pose_link = pose_link

        key = FuzzyTrajectoryCacheKey(
            start_state, goal, cache_pose_link, group_name
        )
        trajectory = self.__getitem__(key)[0].get_trajectory(start_state)
        self._validate_trajectory_quality(
            trajectory,
            pose_link=pose_link,
            true_start_state=start_state,
            true_goal=goal,
        )
        return trajectory

    def get_trajectories(
        self,
        *,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str,
        group_name: str,
    ) -> list[RobotTrajectory]:
        """Get all trajectories for a given key.

        Args:
            start_state: The start state of the trajectory.
            goal: The goal of the trajectory.
            pose_link: The link to use for the pose if the goal is a PoseStamped.
                If the goal is a RobotState, the pose link is not used to
                retrieve the trajectory, but it is used to validate the
                trajectory quality.
            group_name: The group name of the trajectory.

        Returns:
            The sorted list of trajectories for the given key.
        """
        if isinstance(goal, RobotState):
            cache_pose_link = None
        else:
            cache_pose_link = pose_link

        key = FuzzyTrajectoryCacheKey(
            start_state, goal, cache_pose_link, group_name
        )
        trajectories = [
            v.get_trajectory(start_state) for v in self.__getitem__(key)
        ]
        for trajectory in trajectories:
            self._validate_trajectory_quality(
                trajectory,
                pose_link=pose_link,
                true_start_state=start_state,
                true_goal=goal,
            )
        return trajectories

    def __contains__(self, key: FuzzyTrajectoryCacheKey) -> bool:
        """Check if a key is in the database.

        Args:
            key: The key to check.

        Returns:
            True if the key is in the database, False otherwise.
        """
        with self._lock:
            return super().__contains__(self.get_fuzzy_key(key))

    def has_trajectory(
        self,
        *,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str | None,
        group_name: str,
    ) -> bool:
        """Check if a trajectory exists for a given key.

        Args:
            start_state: The start state of the trajectory.
            goal: The goal of the trajectory.
            pose_link: The link to use for the pose.
            group_name: The group name of the trajectory.

        Returns:
            True if a trajectory exists for the given key, False otherwise.
        """
        key = FuzzyTrajectoryCacheKey(start_state, goal, pose_link, group_name)
        return self.__contains__(key)

    def __delitem__(self, key: FuzzyTrajectoryCacheKey):
        """Delete a key from the database.

        Args:
            key: The key to delete.
        """
        with self._lock:
            super().__delitem__(self.get_fuzzy_key(key))

    def delete_trajectory(
        self,
        *,
        start_state: RobotState,
        goal: RobotState | PoseStamped,
        pose_link: str | None,
        group_name: str,
    ):
        """Delete all trajectories for a given key.

        Args:
            start_state: The start state of the trajectory.
            goal: The goal of the trajectory.
            pose_link: The link to use for the pose.
            group_name: The group name of the trajectory.
        """
        key = FuzzyTrajectoryCacheKey(start_state, goal, pose_link, group_name)
        self.__delitem__(key)

    def open(self, flag: str = "w"):
        """Open the database."""
        if not self._closed:
            raise RuntimeError("Database is already open")
        with self._lock:
            super().__init__(dbm_sqlite3.open(self._db_path, flag=flag))
            self._closed = False

    def close(self):
        """Close the database and backup the database file."""
        if not self._closed:
            with self._lock:
                try:
                    super().close()
                finally:
                    self._closed = True

    def __enter__(self):
        if self._closed:
            self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

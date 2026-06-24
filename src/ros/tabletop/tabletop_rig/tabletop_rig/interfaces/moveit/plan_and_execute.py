"""Motion planning and trajectory execution interface.

This module defines PlanAndExecuteInterface (a direct subclass of
BaseInterface) with capabilities for planning robot trajectories and
executing them on the real robot. It integrates with MoveIt's planning
pipeline and trajectory execution manager, and is the base class of
ObjectManipulationInterface.

Key Capabilities:
- Single and multi-waypoint trajectory planning
- Trajectory caching for faster re-planning
- Time-optimal trajectory generation (TOTG) and smoothing
- Safe trajectory execution with pre-flight checks
- Trajectory concatenation for complex motions
- Error recovery and retry logic

The interface supports both joint space and Cartesian space goals,
with configurable planning pipelines (OMPL, Pilz, etc.).

Planning Flow:
1. Create PlanRequest/ConcatPlanRequest with goal and parameters
2. Call plan() or plan_concat() to compute trajectory
3. Optionally post-process with TOTG/smoothing
4. Execute with plan_and_execute() or execute_trajectory()

Safety:
    Before execution, the interface checks that the current robot state
    matches the trajectory start state and that external safety conditions
    are met via a callback function.
"""

import asyncio
import os
import threading
from collections.abc import Callable
from copy import copy, deepcopy
from typing import Any, Optional

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit.core.planning_interface import (  # type: ignore[reportMissingModuleSource]
    MotionPlanResponse,
)
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from moveit.planning import (
    MultiPipelinePlanRequestParameters,
    PlanningComponent,
    PlanRequestParameters,
)
from moveit_msgs.msg import (
    Constraints,
)
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from trajectory_msgs.msg import JointTrajectory

from tabletop_rig.exceptions import (
    ActionGoalNotAcceptedError,
    ActionResultUnsuccessfulError,
    ExecutionInterruptedError,
    ExecutionPreventedError,
    ExecutionRejectedError,
    ExecutionStoppedError,
    MaxPlanningAttemptsReachedError,
    NotSafeToExecuteError,
    PlanningError,
    PlanningPipelineError,
    TrajectoryError,
    TrajectoryErrorCodes,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.interfaces.moveit.moveit import MoveItInterface
from tabletop_rig.interfaces.moveit.requests import (
    ConcatPlanRequest,
    JointStateDeltaDict,
    JointStateDict,
    PlanGoalT,
    PlanRequest,
    PlanResponseT,
    TrajectoryCacheKwargs,
)
from tabletop_rig.interfaces.moveit.trajectory_cache import TrajectoryCache
from tabletop_rig.interfaces.moveit.trajectory_cache_kdtree import (
    KDTreeTrajectoryCache,
)
from tabletop_rig.interfaces.moveit.trajectory_cache_lmdb import (
    LMDBTrajectoryCache,
)
from tabletop_rig.nodes.base import AIOActionClient, BaseNode
from tabletop_rig.utils.ros import (
    all_close_robot_states,
    robot_trajectory_copy,
)


def _is_constraints_goal(goal: Any) -> bool:
    """True iff `goal` is a `list[Constraints]` motion-plan goal.

    Matches the shape accepted by MoveIt's `set_goal_state(
    motion_plan_constraints=...)` overload. An empty list returns False
    (treated as a malformed goal, caught downstream by MoveIt itself).
    """
    return (
        isinstance(goal, list)
        and len(goal) > 0
        and all(isinstance(c, Constraints) for c in goal)
    )


class PlanAndExecuteInterface(BaseInterface):
    """Motion planning and trajectory execution interface.

    Extends BaseInterface with:
    - Motion planning via MoveIt's PlanningComponent (with retry logic)
    - Multi-waypoint trajectory concatenation (ConcatPlanRequest)
    - Trajectory caching (fuzzy matching via LMDB or KD-tree)
    - Post-processing: Time-optimal trajectory generation (TOTG),
        ruckig smoothing, velocity/acceleration scaling
    - Safe trajectory execution with pre-flight safety checks
    - Execution cancellation via stop_execution()

    Supports joint-space (RobotState), Cartesian (PoseStamped), named
    targets (str), and raw constraints (list[Constraints]) goals.

    Parameters from config with '<name>_manipulation_interface.' fallback:
        trajectory_cache.{backend,base_dir,kwargs,freeze_cache}
        execution.{joint_trajectory_controller,allowed_start_tolerance,...}
        planning.{default_pose_link,group_name,fast_pipeline,...}

    Attributes:
        _moveit: MoveItInterface for planning scene access.
        _trajectory_cache: Backend-specific cache (LMDB or KD-tree).
        _execution_client: AIOActionClient for FollowJointTrajectory.
        _executing: Flag indicating active execution.
        _execution_stopped: Flag indicating cancellation requested.
        _execution_goal_handle: Current action goal handle (or None).
    """

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        moveit_interface: MoveItInterface,
        safe_to_execute_condition: Callable[[], bool],
        parameter_fallback_prefix: Optional[str] = None,
    ):
        """Initializes the MoveItPlanInterface

        Args:
            safe_to_execute_callback: Function to evaluate before executing to
                determine if it is safe to execute
        """
        super().__init__(
            node, name, parameter_fallback_prefix=parameter_fallback_prefix
        )

        self._moveit = moveit_interface

        available: list[str] = self._moveit.robot_model.joint_model_group_names
        if self.group_name not in available:
            raise ValueError(
                f"group_name '{self.group_name}' not in present in MoveIt "
                f"RobotModel (defined in the SRDF in your MoveIt config). "
                f"Available joint model group names: {available}"
            )

        # REQUIRED user callback that is checked before executing
        self._safe_to_execute_condition = safe_to_execute_condition

        # Trajectory cache to store previously executed trajectories
        base_dir: str = self.param("trajectory_cache.base_dir")
        backend: str = self.param("trajectory_cache.backend")
        cache_kwargs: dict[str, Any] = self.param("trajectory_cache.kwargs")
        common_kwargs = {
            "path": os.path.join(
                base_dir, self.group_name, f"cache_{backend}"
            ),
            "scene_hash": self._moveit.scene_hash(include_robot=True),
            "planning_frame": self._moveit.planning_frame,
            "group_name": self.group_name,
            "pose_link": self.default_pose_link,
            "parent_logger": self.get_logger(),
            **cache_kwargs,
        }
        self._trajectory_cache: TrajectoryCache
        match backend:
            case "lmdb":
                self._trajectory_cache = LMDBTrajectoryCache(**common_kwargs)
            case "kdtree":
                self._trajectory_cache = KDTreeTrajectoryCache(
                    sample_state=self._moveit.get_current_state(),
                    **common_kwargs,
                )
            case _:
                raise ValueError(
                    f"Unknown trajectory_cache.backend: {backend!r}. "
                    f"Expected one of: 'lmdb', 'kdtree'."
                )
        self._trajectory_cache.open()

        # Execution lock to ensure only one execution command is run at a time

        controller = self.param("execution.joint_trajectory_controller")
        action_name = f"{controller}/follow_joint_trajectory"
        self._execution_client = AIOActionClient(
            self.node,
            FollowJointTrajectory,
            action_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.log(f"Waiting for {action_name} to become available")
        self._execution_client.wait_for_server()

        self._execution_status_lock: threading.Lock = threading.Lock()
        self._executing: bool = False
        self._execution_stopped: bool = False
        self._execution_stopped_future: asyncio.Future | None = None
        self._execution_goal_handle: ClientGoalHandle | None = None
        self._goal_handle_lock: threading.Lock = threading.Lock()

        self.log("MoveIt plan and execute interface initialized")

    ###########################################################################
    ########## Properties #####################################################
    ###########################################################################

    @property
    def executing(self) -> bool:
        """Get the execution status of this robot"""
        with self._execution_status_lock:
            return self._executing

    ###########################################################################
    ########## Parameter Convenience Properties ###############################
    ###########################################################################

    @property
    def default_pose_link(self) -> str:
        """Get the planning link from the parameter server."""
        return self.param("planning.default_pose_link")

    @property
    def group_name(self) -> str:
        """Get the group name of this interface"""
        return self.param("group_name")

    ###########################################################################
    ########## Trajectory processing and validation ###########################
    ###########################################################################

    def _apply_totg(
        self,
        trajectory: RobotTrajectory,
        *,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        path_tolerance: float,
        resample_dt: float,
        min_angle_change: float,
    ) -> RobotTrajectory:
        """Apply time parameterization to the given robot trajectory.

        Args:
            trajectory: The robot trajectory to apply time parameterization to.
            velocity_scaling_factor: The velocity scaling factor to apply to the trajectory.
            acceleration_scaling_factor: The acceleration scaling factor to apply to the trajectory.
            path_tolerance: The path tolerance to apply to the trajectory.
            resample_dt: The resample time step to apply to the trajectory.
            min_angle_change: The minimum angle change to apply to the trajectory.

        Returns:
            The robot trajectory with time parameterization applied.

        Raises:
            TrajectoryError: If time parameterization fails.
        """
        self.log(
            "Applying time parameterization to trajectory with "
            f"velocity_scaling_factor {velocity_scaling_factor}, "
            f"acceleration_scaling_factor {acceleration_scaling_factor}, "
            f"path_tolerance {path_tolerance}, "
            f"resample_dt {resample_dt}, "
            f"min_angle_change {min_angle_change}",
            severity="DEBUG",
        )

        trajectory = robot_trajectory_copy(trajectory)

        old_num_waypoints = len(trajectory)
        old_duration = trajectory.duration
        old_path_length = trajectory.path_length

        if not trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            path_tolerance=path_tolerance,
            resample_dt=resample_dt,
            min_angle_change=min_angle_change,
        ):
            raise TrajectoryError(
                TrajectoryErrorCodes.TOTG_FAILED,
                group_name=trajectory.joint_model_group_name,
            )

        # assert len(trajectory) > 1

        self.log(
            "Time parameterization applied successfully with: "
            f"number of waypoints {old_num_waypoints} -> {len(trajectory)}, "
            f"duration {old_duration} -> {trajectory.duration}, "
            f"path length {old_path_length} -> {trajectory.path_length}",
            severity="DEBUG",
        )

        return trajectory

    def _apply_smoothing(
        self,
        trajectory: RobotTrajectory,
        *,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        mitigate_overshoot: bool,
        overshoot_threshold: float,
    ) -> RobotTrajectory:
        """Apply ruckig smoothing to the given robot trajectory.

        Args:
            trajectory: The robot trajectory to apply smoothing to.
            velocity_scaling_factor: The velocity scaling factor to apply to the trajectory.
            acceleration_scaling_factor: The acceleration scaling factor to apply to the trajectory.
            mitigate_overshoot: Whether to mitigate overshoot.
            overshoot_threshold: The overshoot threshold to apply to the trajectory.

        Returns:
            The robot trajectory with smoothing applied.

        Raises:
            TrajectoryError: If smoothing fails.
        """
        trajectory = robot_trajectory_copy(trajectory)

        old_num_waypoints = len(trajectory)
        old_duration = trajectory.duration
        old_path_length = trajectory.path_length

        self.log(
            f"Applying smoothing to trajectory with "
            f"velocity_scaling_factor {velocity_scaling_factor}, "
            f"acceleration_scaling_factor {acceleration_scaling_factor}, "
            f"mitigate_overshoot {mitigate_overshoot}, "
            f"overshoot_threshold {overshoot_threshold}",
            severity="DEBUG",
        )

        if not trajectory.apply_ruckig_smoothing(
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            mitigate_overshoot=mitigate_overshoot,
            overshoot_threshold=overshoot_threshold,
        ):
            raise TrajectoryError(
                TrajectoryErrorCodes.SMOOTHING_FAILED,
                group_name=trajectory.joint_model_group_name,
            )

        self.log(
            "Smoothing applied successfully with: "
            f"number of waypoints {old_num_waypoints} -> {len(trajectory)}, "
            f"duration {old_duration} -> {trajectory.duration}, "
            f"path length {old_path_length} -> {trajectory.path_length}",
            severity="DEBUG",
        )

        return trajectory

    def _post_process_trajectory(
        self,
        trajectory: RobotTrajectory,
        request: PlanRequest | ConcatPlanRequest,
    ) -> RobotTrajectory:
        """Apply TOTG and/or smoothing to trajectory per request flags.

        Args:
            trajectory: The trajectory to post-process.
            request: Request with apply_totg, apply_smoothing, and scaling
                parameters.

        Returns:
            The post-processed trajectory (copy; original unchanged).
        """
        if request.apply_totg:
            trajectory = self._apply_totg(
                trajectory,
                velocity_scaling_factor=request.velocity_scaling_factor,
                acceleration_scaling_factor=request.acceleration_scaling_factor,
                path_tolerance=request.path_tolerance,
                resample_dt=request.resample_dt,
                min_angle_change=request.min_angle_change,
            )

        if request.apply_smoothing:
            trajectory = self._apply_smoothing(
                trajectory,
                velocity_scaling_factor=request.velocity_scaling_factor,
                acceleration_scaling_factor=request.acceleration_scaling_factor,
                mitigate_overshoot=request.mitigate_overshoot,
                overshoot_threshold=request.overshoot_threshold,
            )

        return trajectory

    def _validate_trajectory(self, trajectory: RobotTrajectory):
        """Validate trajectory against current planning scene.

        Checks for self-collisions and collisions with world objects.

        Args:
            trajectory: The trajectory to validate.

        Raises:
            TrajectoryError: If collision detected or validation failed.
        """
        self.log("Validating trajectory", severity="DEBUG")

        if not self._moveit.is_path_valid(trajectory):
            raise TrajectoryError(
                TrajectoryErrorCodes.INVALID_TRAJECTORY,
                group_name=trajectory.joint_model_group_name,
            )

    ###########################################################################
    ########## Planning and execution #########################################
    ###########################################################################

    def _get_cached_trajectory(
        self,
        request: PlanRequest,
        cancel_event: Optional[threading.Event] = None,
    ) -> RobotTrajectory | None:
        """Attempt to retrieve and validate all cached trajectories"""
        self.log(
            "Attempting to retrieve cached trajectories", severity="DEBUG"
        )
        try:
            trajectories = self._trajectory_cache.get_trajectories(request)
        except KeyError:
            self.log("No cached trajectory found", severity="INFO")
            return None

        for trajectory in trajectories:
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Plan cancelled")

            try:
                trajectory = self._post_process_trajectory(trajectory, request)
                self._validate_trajectory(trajectory)
                return trajectory
            except TrajectoryError as e:
                self.log(
                    f"{type(e).__name__} attempting to use cached trajectory: {e}",
                    severity="WARN",
                )

        self.log("All cached trajectories invalid", severity="WARN")
        # self._trajectory_cache.delete_trajectory(request)
        return None

    def cache_trajectories(self, cache_kwargs: list[TrajectoryCacheKwargs]):
        """Cache the given trajectory.

        Args:
            trajectory: The trajectory to cache.
            **kwargs: Keyword arguments to pass to `TrajectoryCache.cache_trajectory()`.
        """
        if not self.param("trajectory_cache.freeze_cache"):
            for kwargs in cache_kwargs:
                self._trajectory_cache.cache_trajectory(**kwargs)
            self.log(
                f"Cached {len(cache_kwargs)} trajectories successfully",
                severity="DEBUG",
            )
        else:
            self.log("Cache is frozen, skipping cache", severity="DEBUG")

    def _prepare_planning_component(
        self, request: PlanRequest
    ) -> tuple[
        PlanningComponent,
        PlanRequestParameters | MultiPipelinePlanRequestParameters,
    ]:
        """Get the planning component and request parameters for the given kwargs.

        Args:
            request: The request to plan for.

        Returns:
            A tuple of the planning component and request parameters.
        """
        self.log(
            f"Preparing planning component and request parameters with request {request}",
            severity="DEBUG",
        )

        assert request.group_name is not None
        planning_component = self._moveit.get_planning_component(
            request.group_name
        )

        # Set workspace
        planning_component.set_workspace(
            min_x=0.0, min_y=0.0, min_z=0.0, max_x=2.0, max_y=2.0, max_z=2.0
        )

        # Set start state
        if not planning_component.set_start_state(
            robot_state=request.start_state
        ):
            raise ValueError(
                f"Error setting PlanningComponent start state: {request.start_state}"
            )

        # Set goal state
        goal_kwargs = {}
        if isinstance(request.goal, PoseStamped):
            goal_kwargs["pose_stamped_msg"] = request.goal
            goal_kwargs["pose_link"] = request.pose_link
        elif isinstance(request.goal, RobotState):
            goal_kwargs["robot_state"] = request.goal
        else:
            goal_kwargs["motion_plan_constraints"] = request.goal

        if not planning_component.set_goal_state(**goal_kwargs):
            raise ValueError(
                f"Error setting PlanningComponent goal: {request.goal}"
            )

        # Set path constraints
        if request.path_constraints is not None:
            if not planning_component.set_path_constraints(
                request.path_constraints
            ):
                raise ValueError(
                    f"Error setting PlanningComponent path constraints: {request.path_constraints}"
                )

        # Create request parameters
        if isinstance(request.planning_pipeline, str):
            params = PlanRequestParameters(
                self._moveit.moveit_py, request.planning_pipeline
            )
            if request.planning_time is not None:
                params.planning_time = request.planning_time
        else:
            assert isinstance(request.planning_pipeline, (list, tuple))
            params = MultiPipelinePlanRequestParameters(
                self._moveit.moveit_py, request.planning_pipeline
            )
            if request.planning_time is not None:
                for params in params.multi_plan_request_parameters:
                    params.planning_time = request.planning_time

        return planning_component, params

    def _plan_pipeline(
        self,
        request: PlanRequest,
        cancel_event: Optional[threading.Event] = None,
    ) -> RobotTrajectory:
        assert request.group_name is not None
        assert request.max_attempts is not None
        assert request.exp_backoff_factor is not None

        # Get the planning component and request parameters
        planning_component, params = self._prepare_planning_component(request)

        if isinstance(params, PlanRequestParameters):
            max_backoff_time = max(
                params.planning_time,
                self.param("planning.max_backoff_time"),
            )
        else:
            max_backoff_time = None

        # Plan until successful or max attempts reached
        excs: list[PlanningError] = []
        for i in range(request.max_attempts):
            try:
                if cancel_event is not None and cancel_event.is_set():
                    raise asyncio.CancelledError("Plan cancelled")

                if isinstance(params, PlanRequestParameters):
                    plan_response: MotionPlanResponse = (
                        planning_component.plan(
                            self._moveit.moveit_py,
                            single_plan_parameters=params,
                            planning_scene=request.planning_scene,
                        )
                    )
                else:
                    plan_response: MotionPlanResponse = (
                        planning_component.plan(
                            self._moveit.moveit_py,
                            multi_plan_parameters=params,
                            planning_scene=request.planning_scene,
                        )
                    )

                if not plan_response:
                    raise PlanningPipelineError(
                        plan_response.error_code, group_name=request.group_name
                    )

                if cancel_event is not None and cancel_event.is_set():
                    raise asyncio.CancelledError("Plan cancelled")

                trajectory = self._post_process_trajectory(
                    plan_response.trajectory, request
                )
                # TODO: Maybe revalidate
                # self._validate_trajectory(trajectory)
                return trajectory
            except PlanningError as e:
                self.log(
                    f"Planning attempt {i + 1}/{request.max_attempts} failed: {e!r}",
                    severity="WARN",
                )
                excs.append(e)

                # Apply exponential backoff to planning time in case of
                # planning pipeline failure
                if isinstance(e, PlanningPipelineError) and isinstance(
                    params, PlanRequestParameters
                ):
                    assert max_backoff_time is not None
                    backoff_time = (
                        params.planning_time * request.exp_backoff_factor
                    )
                    params.planning_time = min(backoff_time, max_backoff_time)

        raise MaxPlanningAttemptsReachedError(
            excs, group_name=request.group_name
        )

    def _plan_impl(
        self,
        request: PlanRequest,
        *,
        cancel_event: Optional[threading.Event] = None,
    ) -> PlanResponseT:
        """Plan or retrieve cached single trajectory to a goal.

        Normalizes goal (resolves named targets, partial joint goals,
        frame conversions), attempts cache lookup (if cacheable), falls
        back to fast then standard pipeline with exponential backoff
        retry logic, post-processes (TOTG/smoothing), and returns
        trajectory plus cache metadata.

        Cache is skipped for: Constraints goals, non-default pose_link,
        use_cache=False, or custom planning_scene.

        Args:
            request: PlanRequest with goal and parameters.
            cancel_event: threading.Event to signal cancellation.

        Returns:
            (trajectory, cache_kwargs_list or None)

        Raises:
            MaxPlanningAttemptsReachedError: All retry attempts failed.
            TrajectoryError: TOTG/smoothing failed; post-processing error.
        """
        self.log("Planning single trajectory", severity="DEBUG")

        # Flag to disbale if the plan request contains parameters unsupported
        # by the trajectory cache
        cache_supported = True

        # Set to default group name if not provided
        if request.group_name is None:
            request.group_name = self.group_name
        elif request.group_name != self.group_name:
            raise ValueError(
                f"Provided group_name does not match self.group_name. "
                f"Expected '{self.group_name}', got '{request.group_name}'"
            )

        # Set start state to current state if None
        if request.start_state is None:
            request.start_state = self._moveit.get_current_state()

        # Transform goal to world frame or valid robot state
        if isinstance(request.goal, PoseStamped):
            if not request.goal.header.frame_id:
                request.goal.header.frame_id = self._moveit.planning_frame
            elif request.goal.header.frame_id != self._moveit.planning_frame:
                request.goal = self._moveit.change_reference_frame(
                    request.goal, self._moveit.planning_frame
                )
        elif isinstance(request.goal, str):
            request.goal = self._moveit.get_target_state(
                request.goal, request.group_name
            )
        elif isinstance(request.goal, (JointStateDict, JointStateDeltaDict)):
            # Resolve partial joint goals to a concrete RobotState: unprovided
            # joints are filled from the start state, and JointStateDeltaDict
            # values are added to (rather than replacing) the start positions.
            # Downstream this behaves exactly like a RobotState goal, so it
            # plans, caches, and validates the same way.
            request.goal = self._moveit.get_joint_state_target(
                request.goal,
                request.group_name,
                relative=isinstance(request.goal, JointStateDeltaDict),
                base_state=request.start_state,
            )
        elif isinstance(request.goal, RobotState):
            pass
        else:
            assert _is_constraints_goal(request.goal)
            # Constraints goals are not currently supported by cache
            cache_supported = False
            for constraints in request.goal:
                for pc in constraints.position_constraints:
                    if not pc.header.frame_id:
                        pc.header.frame_id = self._moveit.planning_frame

                for oc in constraints.orientation_constraints:
                    if not oc.header.frame_id:
                        oc.header.frame_id = self._moveit.planning_frame

        # Set pose link to default if not provided, but only for Cartesian
        # goals — RobotState and Constraints goals must have pose_link=None
        # per the cache's request-validation contract.
        if request.pose_link is None:
            if isinstance(request.goal, PoseStamped):
                request.pose_link = self.default_pose_link
        else:
            if not isinstance(request.goal, PoseStamped):
                raise ValueError(
                    "'pose_link' cannot be provided if 'goal' is not PoseStamped"
                )
            if request.pose_link != self.default_pose_link:
                # Plan requests for pose links other than the default are
                # not yet supported by the cache
                cache_supported = False

        # Attempt to retrieve cached trajectories if use_cache is True,
        # if the goal is not a Constraints goal, and if the pose link
        # is the same as the default pose link
        if (
            self.param("trajectory_cache.use_cached_trajectories")
            and request.use_cache
            and cache_supported
        ):
            trajectory = self._get_cached_trajectory(request, cancel_event)
            if trajectory is not None:
                return trajectory, None
        else:
            self.log(
                "Not using cached trajectories, planning normally",
                severity="DEBUG",
            )

        if request.max_attempts is None:
            if request.planning_pipeline in ("linear", "ptp"):
                request.max_attempts = 1
            else:
                request.max_attempts = self.param(
                    "planning.default_max_attempts"
                )

        if request.exp_backoff_factor is None:
            request.exp_backoff_factor = self.param(
                "planning.default_exp_backoff_factor"
            )

        fast_request: PlanRequest | None = None
        if request.planning_pipeline is None:
            fast_pipeline: str = self.param("planning.fast_pipeline")
            fallback_pipeline: str = self.param("planning.fallback_pipeline")

            # The fallback pipeline should not be 'linear' or 'ptp'
            if fallback_pipeline in ("linear", "ptp"):
                raise ValueError(
                    "'planning.fallback_pipeline' must not be 'linear' or 'ptp'"
                )
            request.planning_pipeline = fallback_pipeline

            # The linear and ptp pipelines do not support Constraints goals
            if fast_pipeline not in ("linear", "ptp") or isinstance(
                request.goal, (PoseStamped, RobotState)
            ):
                fast_request = copy(request)
                fast_request.planning_pipeline = fast_pipeline
                fast_request.max_attempts = 1

        elif request.planning_pipeline in ("linear", "ptp") and not isinstance(
            request.goal, (PoseStamped, RobotState)
        ):
            raise ValueError(
                f"The '{request.planning_pipeline}' planning pipeline "
                f"cannot be used with Constraints goals"
            )

        trajectory: RobotTrajectory | None = None
        successful_request: PlanRequest | None = None
        if fast_request is not None:
            try:
                trajectory = self._plan_pipeline(fast_request, cancel_event)
                successful_request = fast_request
            except PlanningError as e:
                self.log(
                    f"Could not plan with fast pipeline "
                    f"'{fast_request.planning_pipeline}': {e!r}. "
                    f"Falling back to {request.planning_pipeline}"
                )

        if successful_request is None:
            trajectory = self._plan_pipeline(request, cancel_event)
            successful_request = request

        assert trajectory is not None

        if not cache_supported:
            return trajectory, None

        cache_kwargs: TrajectoryCacheKwargs = {
            "trajectory": trajectory,
            "request": successful_request,
        }

        return trajectory, [cache_kwargs]

    def _plan_concat_impl(
        self,
        request: ConcatPlanRequest,
        *,
        cancel_event: Optional[threading.Event] = None,
    ) -> PlanResponseT:
        """Plan multiple waypoint goals and concatenate trajectories.

        Plans each goal sequentially (start_state for first only),
        concatenates with dwell times (dts), optionally adds loop-close
        segment, and post-processes (either per-segment or after concat).

        Args:
            request: ConcatPlanRequest with goals, dts, loop, etc.
            cancel_event: threading.Event to signal cancellation.

        Returns:
            (concatenated_trajectory, cache_kwargs_list or None)

        Raises:
            ValueError: goals empty, dts length mismatch, or post_process_
                after_concat conflicts with dts.
            MaxPlanningAttemptsReachedError: Any segment planning fails.
            TrajectoryError: Post-processing failed.
        """
        self.log("Planning concat trajectory", severity="DEBUG")

        # Validate request
        if len(request.goals) < 1:
            raise ValueError("At least one goal must be provided")
        if request.post_process_after_concat:
            if request.dts is not None:
                raise ValueError(
                    "'dts' cannot be provided if 'post_process_after_concat' is True"
                )

        # Set initial start state if None
        if request.start_state is None:
            start_state = self._moveit.get_current_state()
        else:
            start_state = request.start_state

        # If loop is requested, we add the requested start state as a final
        # goal to plan to, so that subsequent calls to execute for the same
        # request don't fail because the last state is too far from the
        # start state
        if request.loop:
            request.goals.append(start_state)

        # Validate or create list of dts
        if request.dts is None:
            request.dts = [0.0] * len(request.goals)
        elif len(request.dts) != len(request.goals):
            raise ValueError("dts must be the same length as goals")

        # Loop over individual plan requests
        trajectories: list[RobotTrajectory] = []
        cache_kwargs: list[TrajectoryCacheKwargs] = []

        for i, req in enumerate(request.generate_plan_requests()):
            self.log(
                f"Planning trajectory segment {i + 1}/{len(request.goals)}",
            )
            try:
                req.start_state = start_state

                # If completing the loop and the requested planning_pipeline is
                # 'linear', we have to set it to default for this segment
                if (
                    request.loop
                    and i == len(request.goals) - 1
                    and req.planning_pipeline == "linear"
                ):
                    self.log(
                        "Both 'loop' and 'planning_pipeline=linear' were requested, "
                        "but 'loop' uses the initial robot state as the goal for "
                        "the loop closure segment, which is unsupported by the "
                        "'linear' planning pipeline. Reverting to default for the "
                        "last segment",
                        severity="WARN",
                    )
                    req.planning_pipeline = None

                trajectory, single_cache_kwargs = self._plan_impl(
                    request=req, cancel_event=cancel_event
                )

                trajectories.append(trajectory)
                if single_cache_kwargs is not None:
                    cache_kwargs.extend(single_cache_kwargs)

                start_state = trajectory[len(trajectory) - 1]
            except Exception:
                self.log(
                    f"Error generating segment {i + 1}/{len(request.goals)}",
                    severity="ERROR",
                )
                raise

        # Concatenate all trajectories with given dt
        concat_trajectory = RobotTrajectory(self._moveit.robot_model)
        concat_trajectory.joint_model_group_name = trajectories[
            0
        ].joint_model_group_name
        for dt, trajectory in zip(request.dts, trajectories):
            concat_trajectory.append(trajectory, dt=dt + 1e-4, start_index=0)

        # Post process after concatenation if requested
        if request.post_process_after_concat:
            concat_trajectory = self._post_process_trajectory(
                concat_trajectory, request
            )

        if len(cache_kwargs) == 0:
            return concat_trajectory, None

        return concat_trajectory, cache_kwargs

    async def plan(
        self,
        request: Optional[PlanRequest | ConcatPlanRequest] = None,
        *,
        goal: Optional[PlanGoalT] = None,
        goals: Optional[list[PlanGoalT]] = None,
        **kwargs: Any,
    ) -> PlanResponseT:
        """Plan a trajectory to a single goal or series of goals.

        Accepts one of: request, goal, or goals. All other parameters
        (planning_pipeline, planning_time, etc.) come from request or
        kwargs. Runs in thread pool to avoid blocking async executor.

        Args:
            request: PlanRequest or ConcatPlanRequest. Mutually exclusive
                with goal/goals/kwargs.
            goal: Single goal (RobotState, PoseStamped, str, or
                list[Constraints]). Creates PlanRequest.
            goals: List of goals. Creates ConcatPlanRequest.
            **kwargs: PlanRequest/ConcatPlanRequest constructor args
                (group_name, planning_pipeline, planning_time, etc.).

        Returns:
            (trajectory, cache_kwargs_list or None). trajectory is the
            planned RobotTrajectory. cache_kwargs_list is None if cached
            or if trajectory was not cacheable; otherwise list of dicts
            for cache_trajectories().

        Raises:
            ValueError: Multiple goal sources or request+kwargs provided.
            MaxPlanningAttemptsReachedError: All planning attempts failed.
            TrajectoryError: Post-processing (TOTG/smoothing) failed.
        """
        if request is not None:
            if goal is not None or goals is not None or len(kwargs) > 0:
                raise ValueError(
                    "None of 'goal', 'goals', or additional kwargs may be provided if 'request' is provided"
                )
            request = deepcopy(request)
        elif goal is not None:
            if goals is not None:
                raise ValueError("Both 'goal' and 'goals' cannot be provided")

            request = PlanRequest(goal=goal, **kwargs)
        elif goals is not None:
            request = ConcatPlanRequest(goals=goals, **kwargs)
        else:
            raise ValueError(
                "One of 'goal', 'goals', or 'request' must be provided"
            )

        cancel_event = threading.Event()
        try:
            if isinstance(request, PlanRequest):
                return await asyncio.to_thread(
                    self._plan_impl, request, cancel_event=cancel_event
                )
            elif isinstance(request, ConcatPlanRequest):
                return await asyncio.to_thread(
                    self._plan_concat_impl, request, cancel_event=cancel_event
                )
        finally:
            cancel_event.set()

    async def _send_trajectory(self, trajectory: RobotTrajectory) -> None:
        allowed_start_tolerance = self.param(
            "execution.allowed_start_tolerance"
        )
        allowed_end_tolerance = self.param("execution.allowed_end_tolerance")
        allowed_duration_scaling = self.param(
            "execution.allowed_duration_scaling"
        )
        allowed_duration_margin = self.param(
            "execution.allowed_duration_margin"
        )
        if allowed_duration_scaling < 1:
            raise ValueError(
                "'execution.allowed_duration_scaling' parameter must be at least 1"
            )
        if allowed_duration_margin < 0:
            raise ValueError(
                "'execution.allowed_duration_margin' parameter must be at least 0"
            )

        # Check if the robot is safe to execute
        if not self._safe_to_execute_condition():
            raise NotSafeToExecuteError(
                "Not safe to execute before motion started.",
                group_name=trajectory.joint_model_group_name,
            )

        # Check if trajectory start state is within the allowed start tolerance
        if not all_close_robot_states(
            self._moveit.get_current_state(),
            trajectory[0],
            group_name=trajectory.joint_model_group_name,
            position_tolerance=allowed_start_tolerance,
        ):
            raise ExecutionPreventedError(
                f"Trajectory start state deviates from current robot state by more "
                f"than {allowed_start_tolerance}",
                group_name=trajectory.joint_model_group_name,
            )

        allowed_duration = (
            allowed_duration_scaling * trajectory.duration
            + allowed_duration_margin
        )

        traj_msg: JointTrajectory = (
            trajectory.get_robot_trajectory_msg().joint_trajectory
        )
        try:
            goal_handle = await self._execution_client.send_goal_async(
                FollowJointTrajectory.Goal(
                    trajectory=traj_msg,
                )
            )
        except ActionGoalNotAcceptedError:
            raise ExecutionRejectedError(
                "FollowJointTrajectory Action goal not accepted",
                group_name=trajectory.joint_model_group_name,
            )

        with self._goal_handle_lock:
            assert self._execution_goal_handle is None
            self._execution_goal_handle = goal_handle

        try:
            async with asyncio.timeout(allowed_duration):
                result: FollowJointTrajectory.Result = await (
                    self._execution_client.get_result_async(goal_handle)
                )
        except TimeoutError:
            raise ExecutionInterruptedError(
                f"Allowed execution duration ({allowed_duration}) exceeded",
                group_name=trajectory.joint_model_group_name,
            )
        except ActionResultUnsuccessfulError as e:
            result = e.response.result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                raise ExecutionInterruptedError(
                    "FollowJointTrajectory Action result failed",
                    group_name=trajectory.joint_model_group_name,
                ) from e
        finally:
            with self._goal_handle_lock:
                self._execution_goal_handle = None

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise ExecutionInterruptedError(
                result.error_string,
                group_name=trajectory.joint_model_group_name,
            )

        await asyncio.sleep(0.1)

        # Check if final robot state is within the allowed end tolerance of
        # trajectory end state
        if not all_close_robot_states(
            self._moveit.get_current_state(),
            trajectory[len(trajectory) - 1],
            group_name=trajectory.joint_model_group_name,
            position_tolerance=allowed_end_tolerance,
        ):
            raise ExecutionInterruptedError(
                f"Current robot state deviates from trajectory end state "
                f"by more than {allowed_end_tolerance}",
                group_name=trajectory.joint_model_group_name,
            )

    async def execute(
        self, trajectory: RobotTrajectory | list[RobotTrajectory]
    ):
        """Execute one or more pre-planned trajectories on the robot.

        Checks safety conditions, validates start state, sends goal to
        FollowJointTrajectory action, monitors execution, and validates
        end state. Can be cancelled via stop_execution() from another
        thread.

        Args:
            trajectory: Single RobotTrajectory or list of trajectories
                (executed sequentially).

        Raises:
            ValueError: Trajectory group_name doesn't match interface
                group_name.
            RuntimeError: Execution already in progress.
            NotSafeToExecuteError: Safety check failed (before motion).
            ExecutionPreventedError: Start state > allowed_start_tolerance
                from current state.
            ExecutionRejectedError: Action goal not accepted.
            ExecutionInterruptedError: Execution timeout exceeded, error
                code returned, or end state > allowed_end_tolerance from
                goal.
            ExecutionStoppedError: stop_execution() called during execution.
        """

        if isinstance(trajectory, RobotTrajectory):
            trajectories = [trajectory]
        else:
            trajectories = trajectory

        group_names = (x.joint_model_group_name for x in trajectories)

        if any(x != self.group_name for x in group_names):
            raise ValueError(
                f"joint_model_group_name for one or more provided "
                f"trajectories does not match self.group_name. "
                f"Expected '{self.group_name}', got '{group_names}'"
            )

        with self._execution_status_lock:
            if self._executing:
                raise RuntimeError("Execution already in progress")

            assert not self._execution_stopped
            assert self._execution_stopped_future is None

            loop = asyncio.get_running_loop()
            self._execution_stopped_future = loop.create_future()
            self._executing = True

        try:
            for trajectory in trajectories:
                send_task = asyncio.create_task(
                    self._send_trajectory(trajectory)
                )
                try:
                    await asyncio.wait(
                        [self._execution_stopped_future, send_task],
                        return_when="FIRST_COMPLETED",
                    )
                finally:
                    # If send task is not done, cancel it and wait for it to
                    # complete cancellation
                    send_task.cancel()
                    try:
                        await send_task
                    except asyncio.CancelledError:
                        pass

                with self._execution_status_lock:
                    if self._execution_stopped:
                        raise ExecutionStoppedError(
                            "'stop_execution' called",
                            group_name=trajectory.joint_model_group_name,
                        )

        finally:
            with self._execution_status_lock:
                self._execution_stopped_future = None
                self._execution_stopped = False
                self._executing = False

    @staticmethod
    def _set_future(fut: asyncio.Future):
        if not fut.done():
            fut.set_result(None)

    def stop_execution(self):
        """Signal execution to stop; safe to call from any thread.

        Cancels the FollowJointTrajectory goal and wakes the execute()
        coroutine, which raises ExecutionStoppedError. No-op if not
        currently executing.
        """
        with self._execution_status_lock:
            if self._executing:
                self._execution_stopped = True

                with self._goal_handle_lock:
                    if (
                        self._execution_goal_handle is not None
                        and self._execution_goal_handle.status
                        not in (
                            GoalStatus.STATUS_CANCELING,
                            GoalStatus.STATUS_SUCCEEDED,
                            GoalStatus.STATUS_CANCELED,
                            GoalStatus.STATUS_ABORTED,
                        )
                    ):
                        self._execution_goal_handle.cancel_goal_async()

                assert self._execution_stopped_future is not None
                loop = self._execution_stopped_future.get_loop()
                loop.call_soon_threadsafe(
                    self._set_future, self._execution_stopped_future
                )

    async def plan_and_execute(
        self,
        request: Optional[PlanRequest | ConcatPlanRequest] = None,
        *,
        cache_trajectories: bool = True,
        **kwargs: Any,
    ) -> list[TrajectoryCacheKwargs] | None:
        """Plan a trajectory and execute it atomically.

        Convenience method combining plan() and execute(). If
        cache_trajectories=True, trajectory is cached after successful
        execution (deferred so only successful plans are persisted).

        Args:
            request: PlanRequest/ConcatPlanRequest or None.
            cache_trajectories: Cache trajectory after execution succeeds.
            **kwargs: plan() and execute() arguments (goal, goals, etc.);
                start_state is forbidden (always uses current state).

        Returns:
            cache_kwargs_list if cache_trajectories=True and trajectory
            was planned (not cached); None otherwise.

        Raises:
            ValueError: start_state provided in kwargs.
            PlanningError: Planning failed.
            ExecutionError: Execution failed; trajectory still cached if
                cache_trajectories=True.
        """
        self.log("Planning and executing trajectory", severity="DEBUG")

        if (
            request is not None and request.start_state is not None
        ) or "start_state" in kwargs:
            raise ValueError("start_state is not allowed in plan_and_execute")

        # Plan and return immediately if already at goal
        trajectory, cache_kwargs = await self.plan(request=request, **kwargs)

        # Execute desired request
        await self.execute(trajectory)

        # Cache the trajectory if requested
        if cache_trajectories and cache_kwargs is not None:
            self.cache_trajectories(cache_kwargs)
            return None

        return cache_kwargs

    ###########################################################################
    ########## Destroy ########################################################
    ###########################################################################

    def destroy_interface(self):
        """Clean up trajectory cache before shutting down MoveItPy"""
        self.log("Destroying PlanAndExecuteInterface")
        if hasattr(self, "_trajectory_cache"):
            self._trajectory_cache.close()

        # self._tpe.shutdown()
        super().destroy_interface()

"""Motion planning and trajectory execution interface.

This module extends PlanningSceneInterface with capabilities for planning
robot trajectories and executing them on the real robot. It integrates
with MoveIt's planning pipeline and trajectory execution manager.

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
import concurrent
import concurrent.futures
import threading
from collections.abc import Callable
from copy import copy
from types import TracebackType
from typing import Any, Optional, Self

from geometry_msgs.msg import PoseStamped
from moveit.core.controller_manager import (  # type: ignore[reportMissingModuleSource]
    ExecutionStatus,
)
from moveit.core.planning_interface import (  # type: ignore[reportMissingModuleSource]
    MotionPlanResponse,
)
from moveit.core.robot_model import (  # type: ignore[reportMissingModuleSource]
    RobotModel,
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
    TrajectoryExecutionManager,
)

from tabletop_rig.exceptions import (
    ExecutionInterruptedError,
    ExecutionRejectedError,
    MaxPlanningAttemptsReachedError,
    NotSafeToExecuteError,
    PlanningError,
    PlanOnceError,
    TrajectoryError,
    TrajectoryErrorCodes,
)
from tabletop_rig.interfaces.moveit.planning_scene import (
    PlanningSceneInterface,
)
from tabletop_rig.interfaces.moveit.requests import (
    ConcatPlanRequest,
    PlanGoalT,
    PlanRequest,
    PlanResponseT,
    TrajectoryCacheKwargs,
)
from tabletop_rig.interfaces.moveit.trajectory_cache import (
    FuzzyTrajectoryCache,
)
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import (
    all_close_robot_states,
    robot_trajectory_copy,
)


class PlanAndExecuteInterface(PlanningSceneInterface):
    """Interface for motion planning and trajectory execution.

    Extends PlanningSceneInterface with:
    - Motion planning via MoveIt's PlanningComponent
    - Trajectory execution via TrajectoryExecutionManager
    - Trajectory caching via FuzzyTrajectoryCache
    - Post-processing (TOTG, smoothing)
    - Safety-checked execution

    The interface maintains a trajectory pre-cache for deferred cache
    updates after successful execution.

    Attributes:
        _planning_component: MoveIt planning component for motion planning.
        _trajectory_execution_manager: Manager for executing trajectories.
        _safe_to_execute_callback: External safety check function.
        _trajectory_cache: Fuzzy cache for trajectory reuse.
        _trajectory_precache: Pending cache entries awaiting confirmation.
    """

    def __init__(
        self,
        node: BaseNode,
        safe_to_execute_callback: Callable[[], bool],
    ):
        """Initializes the MoveItPlanInterface

        Args:
            safe_to_execute_callback: Function to evaluate before executing to
                determine if it is safe to execute
        """
        super().__init__(node)

        # REQUIRED user callback that is checked before executing
        self._safe_to_execute_callback = safe_to_execute_callback

        # Trajectory cache to store previously executed trajectories
        trajectory_cache_config = self.node.param("trajectory_cache.kwargs")
        allowed_start_tolerance = self.node.param(
            "trajectory_execution.allowed_start_tolerance"
        )
        self._trajectory_cache = FuzzyTrajectoryCache(
            scene_hash=self.scene_hash(include_robot=True),
            planning_frame=self.planning_frame,
            robot_state_tolerance=allowed_start_tolerance,
            parent_logger=self.get_logger(),
            **trajectory_cache_config,
        )

        # Execution lock to ensure only one execution command is run at a time
        self._execution_lock = threading.Lock()
        self._execute_future: concurrent.futures.Future | None = None
        self._stop_execution_future: concurrent.futures.Future | None = None

        self.log("MoveIt plan and execute interface initialized")

    ###########################################################################
    ########## Parameter Convenience Properties ###############################
    ###########################################################################

    @property
    def default_group_name(self) -> str:
        """Get the planning group name from the parameter server."""
        return self.node.param("planning.default_group_name")

    @property
    def default_pose_link(self) -> str:
        """Get the planning link from the parameter server."""
        return self.node.param("planning.default_pose_link")

    ###########################################################################
    ########## MoveIt Convenience Methods and Properties ######################
    ###########################################################################

    @property
    def executing(self) -> bool:
        return self._execution_lock.locked()

    @property
    def robot_model(self) -> RobotModel:
        """Get the robot model."""
        return self.moveit_py.get_robot_model()

    @property
    def trajectory_execution_manager(self) -> TrajectoryExecutionManager:
        """Get the trajectory execution manager."""
        return self.moveit_py.get_trajectory_execution_manager()

    def get_planning_component(
        self, group_name: Optional[str] = None
    ) -> PlanningComponent:
        """Get the planning component for a given planning group name.

        Args:
            group_name: The name of the planning group. If None, the default planning group is used.

        Returns:
            The planning component for the specified group.
        """
        if group_name is None:
            group_name = self.default_group_name
        return self.moveit_py.get_planning_component(group_name)

    def get_named_target_states(
        self, group_name: Optional[str] = None
    ) -> list[str]:
        """Get the named target states from the planning component."""
        return self.get_planning_component(group_name).named_target_states()

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
            raise TrajectoryError(TrajectoryErrorCodes.TOTG_FAILED)

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
            raise TrajectoryError(TrajectoryErrorCodes.SMOOTHING_FAILED)

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
        """Preprocess the trajectory using the given request.

        Args:
            request: The request to preprocess the trajectory with.

        Returns:
            The preprocessed trajectory.
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
        """Validate the given robot trajectory.

        Args:
            trajectory: The robot trajectory to validate.

        Raises:
            TrajectoryError: If the trajectory is invalid.
        """
        self.log("Validating trajectory", severity="DEBUG")

        group_name = trajectory.joint_model_group_name

        with self.planning_scene_ro() as scene:
            if not scene.is_path_valid(
                trajectory,
                joint_model_group_name=group_name,
                verbose=True,
                invalid_index=[],
            ):
                raise TrajectoryError(TrajectoryErrorCodes.INVALID_TRAJECTORY)

    ###########################################################################
    ########## Planning and execution #########################################
    ###########################################################################

    def get_target_state(
        self, target_name: str, group_name: Optional[str] = None
    ) -> RobotState:
        """Get the named target state from the planning component."""
        joint_state_dict = self.get_planning_component(
            group_name
        ).get_named_target_state_values(target_name)
        robot_state = self.current_state
        robot_state.joint_positions = joint_state_dict
        robot_state.update()
        return robot_state

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
            self.log("No cached trajectory found, planning normally")
            return None

        self.log(
            "Cached trajectories found, validating in order of path length"
        )
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

        self.log("All cached trajectories invalid, planning normally")
        return None

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

        planning_component = self.get_planning_component(request.group_name)

        # Set workspace
        planning_component.set_workspace(
            min_x=0.0, min_y=0.0, min_z=0.0, max_x=2.0, max_y=2.0, max_z=2.0
        )

        # Set start state
        if not planning_component.set_start_state(
            robot_state=request.start_state
        ):
            raise ValueError(f"Invalid start state: {request.start_state}")

        # Check that pose_link is the planning link
        # TODO: Implement pose_link functionality
        if (
            request.pose_link is not None
            and request.pose_link != self.default_pose_link
        ):
            raise NotImplementedError(
                "pose_link functionality is not implemented"
            )

        # Verify goal has already been transformed correctly
        if isinstance(request.goal, PoseStamped):
            assert request.goal.header.frame_id == self.planning_frame
        else:
            assert isinstance(request.goal, RobotState)

        # Set goal state
        goal_kwargs = {}
        if isinstance(request.goal, PoseStamped):
            goal_kwargs["pose_stamped_msg"] = request.goal
            goal_kwargs["pose_link"] = request.pose_link
        else:
            goal_kwargs["robot_state"] = request.goal

        if not planning_component.set_goal_state(**goal_kwargs):
            raise ValueError(f"Invalid goal: {request.goal}")

        # Set path constraints
        if request.path_constraints is not None:
            planning_component.set_path_constraints(request.path_constraints)

        # Create request parameters
        if isinstance(request.planning_pipeline, str):
            request_params = PlanRequestParameters(
                self.moveit_py, request.planning_pipeline
            )
        else:
            assert isinstance(request.planning_pipeline, (list, tuple))
            request_params = MultiPipelinePlanRequestParameters(
                self.moveit_py, request.planning_pipeline
            )

        return planning_component, request_params

    def _plan_pipeline(
        self,
        request: PlanRequest,
        cancel_event: Optional[threading.Event] = None,
    ) -> RobotTrajectory:
        # Get the planning component and request parameters
        planning_component, request_params = self._prepare_planning_component(
            request
        )

        # Plan until successful or max attempts reached
        errors: list[PlanOnceError] = []
        for i in range(request.max_attempts):
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Plan cancelled")

            if isinstance(request_params, MultiPipelinePlanRequestParameters):
                plan_response: MotionPlanResponse = planning_component.plan(
                    self.moveit_py,
                    multi_plan_parameters=request_params,
                    planning_scene=request.planning_scene,
                )
            else:
                plan_response: MotionPlanResponse = planning_component.plan(
                    self.moveit_py,
                    single_plan_parameters=request_params,
                    planning_scene=request.planning_scene,
                )

            if plan_response:
                return plan_response.trajectory
            else:
                error = PlanOnceError(plan_response.error_code)
                self.log(
                    f"Planning attempt {i + 1}/{request.max_attempts} failed: {error}",
                    severity="WARN",
                )
                errors.append(error)
        else:
            raise MaxPlanningAttemptsReachedError(errors)

    def _plan_impl(
        self,
        request: PlanRequest,
        *,
        cancel_event: Optional[threading.Event] = None,
    ) -> PlanResponseT:
        """Retrieve trajectory from cache or plan a trajectory"""
        request = copy(request)

        # Transform goal to world frame or valid robot state
        if isinstance(request.goal, PoseStamped):
            if not request.goal.header.frame_id:
                request.goal.header.frame_id = self.planning_frame
            elif request.goal.header.frame_id != self.planning_frame:
                request.goal = self.change_reference_frame(
                    request.goal, self.planning_frame
                )
        elif isinstance(request.goal, str):
            request.goal = self.get_target_state(
                request.goal, request.group_name
            )

        # Set start state to current state if None
        if request.start_state is None:
            request.start_state = self.current_state

        # Set pose link and group name to default if not provided
        if request.pose_link is None:
            request.pose_link = self.default_pose_link
        if request.group_name is None:
            request.group_name = self.default_group_name

        # Check if the goal is already reached (this is wrong)
        # if isinstance(request.goal, PoseStamped):
        #     tolerance_kwargs = self.node.param(
        #         "planning.pose_tolerance"
        #     )
        #     if all_close_poses_stamped(
        #         request.goal, self.eef_pose_stamped(), **tolerance_kwargs
        #     ):
        #         self.log("Already at goal pose, skipping planning")
        #         return None, []
        # else:
        #     tolerance = self.node.param(
        #         "trajectory_execution.allowed_start_tolerance"
        #     )
        #     if all_close_robot_states(
        #         request.goal, self.current_state, position_tolerance=tolerance
        #     ):
        #         self.log("Already at goal state, skipping planning")
        #         return None, []

        # Attempt to retrieve cached trajectories for this request
        if (
            self.node.param("trajectory_cache.use_cached_trajectories")
            and request.use_cache
        ):
            trajectory = self._get_cached_trajectory(request, cancel_event)
            if trajectory is not None:
                return trajectory, None
        else:
            self.log("Not using cached trajectories, planning normally")

        pipelines: list[str]
        attempts: list[int]
        if request.planning_pipeline is None:
            pipelines = [self.node.param("planning.fallback_pipeline")]
            attempts = [request.max_attempts]

            if isinstance(request.goal, PoseStamped):
                pipelines.insert(0, self.node.param("planning.fast_pipeline"))
                attempts.insert(0, 1)
        elif request.planning_pipeline == "linear" and not isinstance(
            request.goal, PoseStamped
        ):
            raise ValueError(
                "The `linear` planning pipeline cannot be used with RobotState goals"
            )
        else:
            pipelines = [request.planning_pipeline]
            attempts = [request.max_attempts]

        trajectory = None
        for i, (pipeline, attempt) in enumerate(zip(pipelines, attempts)):
            request.planning_pipeline = pipeline
            request.max_attempts = attempt
            try:
                trajectory = self._plan_pipeline(request, cancel_event)
                break
            except PlanningError as e:
                if i < len(pipelines) - 1:
                    self.log(
                        f"Could not plan with pipeline {pipeline}, falling back to {pipelines[i + 1]}"
                    )
                else:
                    raise e

        if cancel_event is not None and cancel_event.is_set():
            raise asyncio.CancelledError("Plan cancelled")

        assert trajectory is not None
        trajectory = self._post_process_trajectory(trajectory, request)
        # TODO: Maybe revalidate
        # self._validate_trajectory(trajectory)
        cache_kwargs: TrajectoryCacheKwargs = {
            "trajectory": trajectory,
            "request": request,
        }

        return trajectory, [cache_kwargs]

    def _plan_concat_impl(
        self,
        request: ConcatPlanRequest,
        *,
        cancel_event: Optional[threading.Event] = None,
    ) -> PlanResponseT:
        """Plan a series of trajectories and concatenate them"""
        request = copy(request)

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
            start_state = self.current_state
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
                f"Planning trajectory segment {i}/{len(request.goals)}",
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
                    f"Error generating segment {i}/{len(request.goals)}",
                    severity="ERROR",
                )
                raise

        # Concatenate all trajectories with given dt
        concat_trajectory = RobotTrajectory(self.robot_model)
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
        """Plan a trajectory to a goal or series of goals

        Args:
            goal: The goal to plan towards. If not provided, 'goals' or
                'request' must be provided
            goals: The goals to plan towards. If not provided, 'goals' or
                'request' must be provided
            request: The request to plan for. If not provided, the request is
                created from goal and kwargs.
            **kwargs: Keyword arguments to pass to the 'PlanRequest()' or
                'ConcatPlanRequest()' constructor.

        Returns:
            A tuple containing:
                The planned trajectory, or None if already at the goal
                The parsed plan request for caching, or None if already at goal
                    a cached trajectory was used


        Raises:
            ValueError: If the request or arguments are invalid.
            MaxPlanningAttemptsReachedError: If the maximum number of planning
                attempts is reached.
            asyncio.CancelledError: If the planning is cancelled by the cancel_event.

        See Also:
            `create_plan_request()`: For parameter details

        See Also:
            `_plan_impl()`: For parameter and implementation details.
        """
        if request is not None:
            if goal is not None or goals is not None or len(kwargs) > 0:
                raise ValueError(
                    "None of 'goal', 'goals', or additional kwargs may be provided if 'request' is provided"
                )
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

    def _execute_and_wait(self) -> ExecutionStatus:
        with self._execution_lock:
            return self.trajectory_execution_manager.execute_and_wait()

    def stop_execution(self):
        """Stop execution of the trajectory, if currently executing"""
        if self._execution_lock.locked():
            self.trajectory_execution_manager.stop_execution()

    async def _execute(
        self, trajectory: RobotTrajectory | list[RobotTrajectory]
    ):
        """Execute the given robot trajectory.

        Args:
            trajectory: Trajectory to execute

        Raises:
            NotSafeToExecuteError: If the robot is not safe to execute.
            ExecutionInterruptedError: If the robot moved but not to the goal.
            ExecutionRejectedError: If the trajectory was rejected by the robot.
        """
        if (
            self._execute_future is not None
            and not self._execute_future.done()
        ):
            raise RuntimeError("Last _execute_future did not complete")
        if (
            self._stop_execution_future is not None
            and not self._stop_execution_future.done()
        ):
            raise RuntimeError("Last _stop_execution_future did not complete")

        # Check if the robot is safe to execute
        if not self._safe_to_execute_callback():
            raise NotSafeToExecuteError()

        if isinstance(trajectory, RobotTrajectory):
            trajectory = [trajectory]

        initial_state = self.current_state

        # Push all trajectories to TEM
        for traj in trajectory:
            self.trajectory_execution_manager.push(
                traj.get_robot_trajectory_msg()
            )

        assert not self._execution_lock.locked()
        tpe: concurrent.futures.ThreadPoolExecutor = (
            asyncio.get_running_loop()._default_executor  # type: ignore
        )
        try:
            # self._execute_future = tpe.submit(
            #     _execute_and_wait, weakref.proxy(self)
            # )
            self._execute_future = tpe.submit(self._execute_and_wait)
            status = await asyncio.wrap_future(self._execute_future)
        except asyncio.CancelledError:
            # self._stop_execution_future = tpe.submit(
            #     _stop_execution, weakref.proxy(self)
            # )
            self.stop_execution()
            # self._stop_execution_future = tpe.submit(self._stop_execution)
            raise

        # Return the trajectory if the execution was successful, otherwise raise
        # an error based on the execution status and safe to execute flag
        if status:
            return
        elif not self._safe_to_execute_callback():
            raise NotSafeToExecuteError(status)
        elif all_close_robot_states(
            initial_state,
            self.current_state,
            position_tolerance=self.node.param(
                "trajectory_execution.allowed_start_tolerance"
            ),
        ):
            raise ExecutionRejectedError(status)
        else:
            raise ExecutionInterruptedError(status)

    async def execute(
        self, trajectory: RobotTrajectory | list[RobotTrajectory]
    ):
        """Execute the given robot trajectory.

        Args:
            trajectory: Trajectory to execute

        Raises:
            NotSafeToExecuteError: If the robot is not safe to execute.
            ExecutionInterruptedError: If the robot moved but not to the goal.
            ExecutionRejectedError: If the trajectory was rejected by the robot.
        """
        return await self._execute(trajectory)

    def cache_trajectories(self, cache_kwargs: list[TrajectoryCacheKwargs]):
        """Cache the given trajectory.

        Args:
            trajectory: The trajectory to cache.
            **kwargs: Keyword arguments to pass to `FuzzyTrajectoryCache.cache_trajectory()`.
        """
        if not self.node.param("trajectory_cache.freeze_cache"):
            for kwargs in cache_kwargs:
                self._trajectory_cache.cache_trajectory(**kwargs)
            self.log(f"Cached {len(cache_kwargs)} trajectories successfully")
        else:
            self.log("Cache is frozen, skipping cache")

    async def _plan_and_execute(
        self,
        request: Optional[PlanRequest | ConcatPlanRequest] = None,
        cache_trajectory: bool = True,
        **kwargs: Any,
    ) -> list[TrajectoryCacheKwargs] | None:
        """Plan and execute a trajectory, using the cached trajectory if available.

        Args:
            *args: Arguments to pass to `create_plan_request()`.
            cache_trajectory: Whether to cache the planned trajectory.
            use_cache: Whether to use the cached trajectory.
            **kwargs: Keyword arguments to pass to `create_plan_request()`
                and `execute()`.

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.

        Raises:
            ValueError: If start_state is provided in kwargs.
            PlanningError: If the planning fails.
            ExecutionError: If the execution fails.
        """
        self.log(
            "Planning and executing trajectory (with cache)", severity="DEBUG"
        )

        if (
            request is not None and request.start_state is not None
        ) or "start_state" in kwargs:
            raise ValueError("start_state is not allowed in plan_and_execute")

        # Plan and return immediately if already at goal
        trajectory, cache_kwargs = await self.plan(request=request, **kwargs)

        # Execute desired request
        await self._execute(trajectory)

        # Cache the trajectory if requested
        if cache_trajectory and cache_kwargs is not None:
            cache_kwargs[-1]["true_end_state"] = self.current_state
            self.cache_trajectories(cache_kwargs)
            return None

        return cache_kwargs

    async def plan_and_execute(
        self,
        request: Optional[PlanRequest | ConcatPlanRequest] = None,
        cache_trajectory: bool = True,
        **kwargs: Any,
    ) -> list[TrajectoryCacheKwargs] | None:
        """Plan and execute a trajectory, using the cached trajectory if available.

        Args:
            *args: Arguments to pass to `create_plan_request()`.
            cache_trajectory: Whether to cache the planned trajectory.
            use_cache: Whether to use the cached trajectory.
            **kwargs: Keyword arguments to pass to `create_plan_request()`
                and `execute()`.

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.

        Raises:
            ValueError: If start_state is provided in kwargs.
            PlanningError: If the planning fails.
            ExecutionError: If the execution fails.
        """
        return await self._plan_and_execute(
            request, cache_trajectory, **kwargs
        )

    ###########################################################################
    ########## Reset (simulation) #############################################
    ###########################################################################

    async def clear_scene_and_reset(
        self, end_goal: Optional[PlanGoalT] = None, **kwargs
    ):
        """Ignore collisions and move robot to end_goal asynchronously.

        To be used only in simulation. With the real robot, the user should
        manually (via the teach pendant) move the robot away from the collision
        objects.

        Using this function will reset any attached collision objects
        to their initial poses and move the robot to the target pose, ignoring
        collisions.

        Args:
            end_goal: The goal to move to after moving out of collision.
            **kwargs: Keyword arguments to pass to `plan_and_execute()`.
        """
        if not self.node.param("simulate"):
            raise RuntimeError(
                "Planning scene can only be cleared in simulation!"
            )

        self.log(
            "Clearing planning scene and moving out of collision (simulation only)"
        )

        self.remove_all_collision_objects()
        if end_goal is None:
            end_goal = "idle"
        await self._plan_and_execute(goal=end_goal, **kwargs)
        self._init_planning_scene()

    def __enter__(self) -> Self:
        """Enter the context manager."""
        self._trajectory_cache.__enter__()
        return self

    def __exit__(
        self,
        exc_type: Optional[type[BaseException]],
        exc_value: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ):
        """Exit the context manager."""
        self._trajectory_cache.__exit__(exc_type, exc_value, exc_tb)

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

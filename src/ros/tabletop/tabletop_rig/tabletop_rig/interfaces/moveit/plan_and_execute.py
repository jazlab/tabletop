import asyncio
import threading
from collections.abc import (
    Callable,
    Iterable,
)
from copy import deepcopy
from typing import Any, Optional, cast

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.planning_scene import PlanningScene  # type: ignore
from moveit.core.robot_model import RobotModel  # type: ignore
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore
from moveit.planning import (
    MultiPipelinePlanRequestParameters,
    PlanningComponent,
    PlanRequestParameters,
    TrajectoryExecutionManager,
)

from tabletop_rig.interfaces.moveit.planning_scene import (
    PlanningSceneInterface,
)
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import (
    ExecuteRequest,
    ExecutionInterruptedError,
    ExecutionRejectedError,
    MaxPlanningAttemptsReachedError,
    NotSafeToExecuteError,
    PlanningGoalT,
    PlanOnceError,
    PlanRequest,
    TrajectoryError,
    TrajectoryErrorCodes,
    all_close_poses_stamped,
    all_close_robot_states,
    robot_trajectory_copy,
)
from tabletop_rig.utils.trajectory_cache import FuzzyTrajectoryCache


def always_safe():
    """Dummy callback to always return True"""
    return True


class PlanAndExecuteInterface(PlanningSceneInterface):
    ###########################################################################
    ########## Initialization #################################################
    ###########################################################################

    def __init__(
        self,
        node: BaseNode,
        logger_name: str = "moveit_plan_interface",
        safe_to_execute_callback: Optional[Callable[[], bool]] = None,
    ):
        """Initializes the MoveItPlanInterface"""
        super().__init__(node, logger_name)

        # Optional user callback that is checked before executing
        if safe_to_execute_callback is None:
            self._safe_to_execute_callback = always_safe
        else:
            self._safe_to_execute_callback = safe_to_execute_callback

        # Trajectory cache to store previously executed trajectories
        trajectory_cache_config = self.node.get_parameter_wrapper(
            "trajectory_cache.kwargs"
        )
        allowed_start_tolerance = self.node.get_parameter_wrapper(
            "trajectory_execution.allowed_start_tolerance"
        )
        self.trajectory_cache = FuzzyTrajectoryCache(
            scene_hash=self.scene_hash,
            robot_state_tolerance=allowed_start_tolerance,
            **trajectory_cache_config,
        )

        # Execution lock to ensure only one execution command is run at a time
        self.execution_lock = threading.Lock()

        self.log("MoveIt plan and execute interface initialized")

    def register_safe_to_execute_callback(self, callback: Callable[[], bool]):
        """Register additional callback for teensy sensor subscription

        Args:
            callback: Callable that takes TeensySensor message as argument and returns None
        """
        self._safe_to_execute_callback = callback

    ###########################################################################
    ########## Parameter Convenience Properties ###############################
    ###########################################################################

    @property
    def simulate(self) -> bool:
        """Get the simulation flag."""
        return self.node.get_parameter_wrapper("simulate")

    @property
    def use_cached_trajectories(self) -> bool:
        return self.node.get_parameter_wrapper(
            "trajectory_cache.use_cached_trajectories"
        )

    @property
    def freeze_trajectory_cache(self) -> bool:
        return self.node.get_parameter_wrapper("trajectory_cache.freeze_cache")

    ###########################################################################
    ########## MoveIt Convenience Methods and Properties ######################
    ###########################################################################

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

    @property
    def robot_model(self) -> RobotModel:
        """Get the robot model."""
        return self.moveit_py.get_robot_model()

    @property
    def trajectory_execution_manager(self) -> TrajectoryExecutionManager:
        """Get the trajectory execution manager."""
        return self.moveit_py.get_trajectory_execution_manager()

    @property
    def current_state(self) -> RobotState:
        with self.planning_scene_ro() as scene:
            return deepcopy(scene.current_state)

    def get_named_target_states(
        self, group_name: Optional[str] = None
    ) -> list[str]:
        """Get the named target states from the planning component."""
        return self.get_planning_component(group_name).named_target_states()

    ###########################################################################
    ########## Robot States ###################################################
    ###########################################################################

    def all_close_poses_stamped(
        self,
        pose_stamped1: PoseStamped,
        pose_stamped2: PoseStamped,
        position_tolerance: Optional[
            float | Iterable[float] | np.ndarray
        ] = None,
        orientation_tolerance: Optional[
            float | Iterable[float] | np.ndarray
        ] = None,
        use_euler_tolerance: Optional[bool] = None,
    ) -> bool:
        """Check if two pose stamped messages are all close.

        Performs a reference frame change if necessary.

        Args:
            pose_stamped1: The first pose stamped message.
            pose_stamped2: The second pose stamped message.
            position_tolerance: The tolerance for the position.
            orientation_tolerance: The tolerance for the orientation.
            use_euler_tolerance: Whether to use euler tolerance.
        Returns:
            True if the two pose stamped messages are all close, False otherwise.
        """
        if position_tolerance is None:
            position_tolerance = cast(
                float | list[float],
                self.node.get_parameter_wrapper(
                    "planning.goal_position_tolerance"
                ),
            )
        if orientation_tolerance is None:
            orientation_tolerance = cast(
                float | list[float],
                self.node.get_parameter_wrapper(
                    "planning.goal_orientation_tolerance"
                ),
            )
        if use_euler_tolerance is None:
            use_euler_tolerance = cast(
                bool,
                self.node.get_parameter_wrapper(
                    "planning.use_euler_tolerance"
                ),
            )

        if pose_stamped1.header.frame_id != pose_stamped2.header.frame_id:
            pose_stamped2 = self.change_reference_frame(
                pose_stamped2, pose_stamped1.header.frame_id
            )

        return all_close_poses_stamped(
            pose_stamped1,
            pose_stamped2,
            position_tolerance,
            orientation_tolerance,
            use_euler_tolerance,
        )

    def all_close_robot_states(
        self,
        robot_state1: RobotState,
        robot_state2: RobotState,
        position_tolerance: Optional[float | dict[str, float]] = None,
        velocity_tolerance: Optional[float | dict[str, float]] = None,
        acceleration_tolerance: Optional[float | dict[str, float]] = None,
    ) -> bool:
        """Check if two robot states are all close."""
        if position_tolerance is None:
            position_tolerance = self.node.get_parameter_wrapper(
                "trajectory_execution.allowed_start_tolerance"
            )
        assert position_tolerance is not None
        return all_close_robot_states(
            robot_state1,
            robot_state2,
            position_tolerance,
            velocity_tolerance,
            acceleration_tolerance,
        )

    ###########################################################################
    ########## Planning and execution #########################################
    ###########################################################################

    def get_target_state(
        self, target_name: str, group_name: Optional[str] = None
    ) -> RobotState:
        """Get the named target state from the planning component."""
        if target_name == "idle":
            target_name = self.node.get_parameter_wrapper(
                "predefined_states.idle_state"
            )
        elif target_name == "pre_present":
            target_name = self.node.get_parameter_wrapper(
                "predefined_states.pre_present_state"
            )

        joint_state_dict = self.get_planning_component(
            group_name
        ).get_named_target_state_values(target_name)
        robot_state = self.current_state
        robot_state.joint_positions = joint_state_dict
        robot_state.update()
        return robot_state

    def get_default_plan_request(self) -> PlanRequest:
        """Get the default plan request."""
        kwargs = self.node.get_parameter_wrapper("planning.defaults")
        return PlanRequest(
            goal=PoseStamped(), start_state=self.current_state, **kwargs
        )

    def create_plan_request(
        self, goal: PlanningGoalT, **kwargs: Any
    ) -> tuple[PlanRequest, dict[str, Any]]:
        """Parse the planning kwargs.

        Args:
            goal: The goal to plan for.
            **kwargs: Additional keyword arguments to override the default plan
                request.

        Returns:
            A tuple of the parsed kwargs and any unused kwargs.
        """
        self.log("Parsing plan args", severity="DEBUG")

        request = self.get_default_plan_request()

        # TODO: Implement pose_link functionality
        if "pose_link" in kwargs and kwargs["pose_link"] != request.pose_link:
            raise NotImplementedError(
                "pose_link functionality is not implemented"
            )

        # Override the default kwargs with the provided kwargs
        for key in request.__slots__:
            if key in kwargs:
                setattr(request, key, kwargs.pop(key))

        # Set the goal to the target state if the goal is a configuration name
        if isinstance(goal, str):
            goal = self.get_target_state(goal, request.group_name)
        elif isinstance(goal, PoseStamped):
            if goal.header.frame_id != self.planning_frame:
                goal = self.change_reference_frame(goal, self.planning_frame)
        request.goal = goal

        return request, kwargs

    def _pre_plan(
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
        if request.pose_link != self.default_pose_link:
            raise NotImplementedError(
                "pose_link functionality is not implemented"
            )

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

    def _plan_once_impl(
        self,
        planning_component: PlanningComponent,
        request_params: PlanRequestParameters
        | MultiPipelinePlanRequestParameters,
        planning_scene: Optional[PlanningScene] = None,
    ) -> RobotTrajectory:
        """Plan a trajectory to the given waypoint once.

        Args:
            planning_component: The planning component to use.
            request_params: The request parameters to use.
            planning_scene: The planning scene to use.

        Returns:
            The planned trajectory.

        Raises:
            PlanOnceError: If the planning fails.
        """
        self.log(
            f"Planning once with request params {request_params} and planning scene {planning_scene}",
            severity="DEBUG",
        )
        if isinstance(request_params, MultiPipelinePlanRequestParameters):
            plan_response = planning_component.plan(
                self.moveit_py,
                multi_plan_parameters=request_params,
                planning_scene=planning_scene,
            )
        else:
            plan_response = planning_component.plan(
                self.moveit_py,
                single_plan_parameters=request_params,
                planning_scene=planning_scene,
            )

        if not plan_response:
            raise PlanOnceError(plan_response.error_code)

        return plan_response.trajectory

    def _plan_impl(
        self,
        *args: Any,
        request: Optional[PlanRequest] = None,
        cancel_event: Optional[threading.Event] = None,
        **kwargs: Any,
    ) -> RobotTrajectory | None:
        """
        Plan a trajectory to the given waypoint, retrying up to max_plan_attempts
        times until successful.

        Args:
            *args: Arguments to pass to `create_plan_request()`.
            request: The request to plan for. If not provided, the request is
                created from args and kwargs.
            cancel_event: An event that can be used to cancel planning.
            **kwargs: Keyword arguments to pass to `create_plan_request()`.

        Returns:
            The planned trajectory, or None if the goal is already reached.

        Raises:
            ValueError: If the request or arguments are invalid.
            MaxPlanningAttemptsReachedError: If the maximum number of planning
                attempts is reached.
            asyncio.CancelledError: If the planning is cancelled by the cancel_event.

        See Also:
            `create_plan_request()`: For parameter details
        """
        # Parse the planning args if requested
        if request is None:
            request, unused_kwargs = self.create_plan_request(*args, **kwargs)
            if len(unused_kwargs) > 0:
                raise ValueError(f"Unused kwargs: {unused_kwargs}")
        elif len(args) > 0 or len(kwargs) > 0:
            raise ValueError(
                f"Additional arguments ({args}) or kwargs ({kwargs}) cannot be provided if plan_request is provided"
            )

        # Check if the goal is already reached
        if isinstance(request.goal, PoseStamped):
            if self.all_close_poses_stamped(
                request.goal, self.eef_pose_stamped()
            ):
                self.log("Already at goal, skipping planning")
                return None
        elif self.all_close_robot_states(request.goal, self.current_state):
            self.log("Already at start state, skipping planning")
            return None

        # Get the planning component and request parameters
        planning_component, request_params = self._pre_plan(request)

        # Plan until successful or max attempts reached
        errors: list[PlanOnceError] = []
        for i in range(request.max_plan_attempts):
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Plan cancelled")
            try:
                trajectory = self._plan_once_impl(
                    planning_component, request_params, request.planning_scene
                )
                self.log(
                    f"Planning attempt {i + 1}/{request.max_plan_attempts} succeeded",
                    severity="DEBUG",
                )
                return trajectory
            except PlanOnceError as e:
                self.log(
                    f"Planning attempt {i + 1}/{request.max_plan_attempts} failed: {e}",
                    severity="WARN",
                )
                errors.append(e)
        else:
            raise MaxPlanningAttemptsReachedError(errors)

    async def plan(self, *args: Any, **kwargs: Any) -> RobotTrajectory | None:
        """Asynchronously calls `_plan_impl()` method in a separate thread.

        See Also:
            `_plan_impl()`: For parameter and implementation details.
        """
        cancel_event = threading.Event()
        try:
            return await asyncio.to_thread(
                self._plan_impl, *args, cancel_event=cancel_event, **kwargs
            )
        finally:
            cancel_event.set()

    def get_empty_trajectory(self) -> RobotTrajectory:
        return RobotTrajectory(self.robot_model)

    def get_default_execute_request(self) -> ExecuteRequest:
        """Get the default execute request."""
        kwargs = self.node.get_parameter_wrapper("execution.defaults")
        return ExecuteRequest(trajectory=self.get_empty_trajectory(), **kwargs)

    def create_execute_request(
        self, trajectory: RobotTrajectory, **kwargs: Any
    ) -> tuple[ExecuteRequest, dict[str, Any]]:
        """Create an execute request.

        Args:
            trajectory: The robot trajectory to execute.
            **kwargs: Additional keyword arguments to override the default
                execute request.

        Returns:
            An execute request.
        """
        request = self.get_default_execute_request()

        request.trajectory = trajectory

        # Override the default kwargs with the provided kwargs
        for key in request.__slots__:
            if key in kwargs:
                setattr(request, key, kwargs.pop(key))

        return request, kwargs

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

        assert len(trajectory) > 1

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

    def preprocess_trajectory(
        self, request: ExecuteRequest
    ) -> RobotTrajectory:
        """Preprocess the trajectory using the given request.

        Args:
            request: The request to preprocess the trajectory with.

        Returns:
            The preprocessed trajectory.
        """
        # Apply time parameterization and smoothing to the trajectory
        trajectory = request.trajectory
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

        # Validate the trajectory
        if request.validate_trajectory:
            self._validate_trajectory(trajectory)

        return trajectory

    def _execute_impl(
        self,
        *args: Any,
        request: Optional[ExecuteRequest] = None,
        preprocess_trajectory: bool = True,
        **kwargs: Any,
    ):
        """Execute the given robot trajectory.

        Args:
            *args: Arguments to pass to `create_execute_request()`.
            request: The request to execute. If not provided, the request is
                created from args and kwargs.
            **kwargs: Keyword arguments to pass to `create_execute_request()`.

        Raises:
            ValueError: If the request or arguments are invalid.
            NotSafeToExecuteError: If the robot is not safe to execute.
            ExecutionInterruptedError: If the robot moved but not to the goal.
            ExecutionRejectedError: If the trajectory was rejected by the robot.
            TrajectoryError: If the trajectory processing fails or the
                trajectory is invalid.

        See Also:
            `create_execute_request()`: For parameter details
        """

        if request is None:
            request, unused_kwargs = self.create_execute_request(
                *args, **kwargs
            )
            if len(unused_kwargs) > 0:
                raise ValueError(f"Unused kwargs: {unused_kwargs}")
        elif len(args) > 0 or len(kwargs) > 0:
            raise ValueError(
                f"Additional arguments ({args}) or kwargs ({kwargs}) cannot be provided if request is provided"
            )

        if preprocess_trajectory:
            trajectory = self.preprocess_trajectory(request)
        else:
            trajectory = request.trajectory

        # Check if the robot is safe to execute
        if not self._safe_to_execute_callback():
            raise NotSafeToExecuteError()

        # Execute the trajectory
        initial_state = self.current_state
        self.trajectory_execution_manager.push(
            trajectory.get_robot_trajectory_msg()
        )

        assert not self.execution_lock.locked()
        with self.execution_lock:
            execution_status = (
                self.trajectory_execution_manager.execute_and_wait()
            )
        print("Execution status:", execution_status)

        # Return the trajectory if the execution was successful, otherwise raise
        # an error based on the execution status and safe to execute flag
        if execution_status:
            return
        elif not self._safe_to_execute_callback():
            raise NotSafeToExecuteError(execution_status)
        elif not self.all_close_robot_states(
            initial_state, self.current_state
        ):
            raise ExecutionInterruptedError(execution_status)
        else:
            raise ExecutionRejectedError(execution_status)

    async def execute(self, *args: Any, **kwargs: Any):
        """Asynchronously calls `_execute_impl()` method in a separate thread.

        See Also:
            `_execute_impl()`: For parameter and implementation details.
        """
        try:
            return await asyncio.to_thread(self._execute_impl, *args, **kwargs)
        finally:
            print("Stopping execution")
            if self.execution_lock.locked():
                self.trajectory_execution_manager.stop_execution()
            print("Execution stopped")

    def cache_trajectory(self, trajectory: RobotTrajectory, **kwargs: Any):
        """Cache the given trajectory.

        Args:
            trajectory: The trajectory to cache.
            **kwargs: Keyword arguments to pass to `FuzzyTrajectoryCache.cache_trajectory()`.
        """
        if not self.freeze_trajectory_cache:
            self.trajectory_cache.cache_trajectory(trajectory, **kwargs)
            self.log("Cached trajectory successfully")
        else:
            self.log("Cache is frozen, skipping cache")

    async def plan_and_execute(
        self,
        *args: Any,
        cache_trajectory: bool = True,
        use_cache: bool = True,
        **kwargs: Any,
    ) -> dict[str, Any] | None:
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

        if "start_state" in kwargs:
            raise ValueError("start_state is not allowed in plan_and_execute")

        start_state = self.current_state

        # Parse the planning kwargs
        plan_request, execute_kwargs = self.create_plan_request(
            *args, start_state=start_state, **kwargs
        )

        # Attempt to execute the cached trajectory, otherwise plan and execute normally
        if self.use_cached_trajectories and use_cache:
            try:
                # TODO: Refactor so caching happens in the plan() function
                trajectories = self.trajectory_cache.get_trajectories(
                    plan_request
                )
            except KeyError:
                self.log(
                    "No cached trajectory found, planning and executing normally"
                )
            else:
                self.log(
                    "Cached trajectories found, trying to execute in order of path length"
                )
                for trajectory in trajectories:
                    try:
                        await self.execute(trajectory, **execute_kwargs)
                        return
                    except (ExecutionRejectedError, TrajectoryError) as e:
                        self.log(
                            f"Error attempting to execute cached trajectory: {e}",
                            severity="WARN",
                        )
                self.log(
                    "All cached trajectories failed, planning and executing normally"
                )
        else:
            self.log(
                "Not using cached trajectories, planning and executing normally"
            )

        # Reset the start state to the current state
        plan_request.start_state = self.current_state

        # Plan and execute normally
        trajectory = await self.plan(request=plan_request)
        if trajectory is None:
            return
        await self.execute(
            trajectory, validate_trajectory=False, **execute_kwargs
        )

        to_cache_kwargs = {
            "trajectory": trajectory,
            "request": plan_request,
            "true_end_state": self.current_state,
        }

        # Cache the trajectory if requested
        if cache_trajectory:
            self.cache_trajectory(**to_cache_kwargs)

        return to_cache_kwargs

    # TODO: Move retry logic to Commander

    ###########################################################################
    ########## Reset (simulation) #############################################
    ###########################################################################

    async def move_out_of_collision_simulation(
        self, end_goal: Optional[PlanningGoalT] = None, **kwargs
    ):
        """Move the robot out of collision with the scene asynchronously.

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
        self.log("Moving out of collision")
        if not self.simulate:
            raise RuntimeError("This function is only available in simulation")

        self.remove_all_collision_objects()
        if end_goal is None:
            end_goal = "idle"
        await self.plan_and_execute(end_goal, **kwargs)
        self.init_planning_scene()

    ###########################################################################
    ########## Destroy ########################################################
    ###########################################################################

    def destroy(self):
        if hasattr(self, "trajectory_cache"):
            self.trajectory_cache.close()
        # if hasattr(self, "trajectory_execution_manager"):
        #     self.trajectory_execution_manager.stop_execution()
        super().destroy()

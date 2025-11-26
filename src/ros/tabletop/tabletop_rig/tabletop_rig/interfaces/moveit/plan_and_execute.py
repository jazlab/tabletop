import asyncio
import threading
from collections.abc import Callable, Iterable
from copy import copy, deepcopy
from types import TracebackType
from typing import Any, Optional, Self, cast

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.core.controller_manager import (  # type: ignore[reportMissingModuleSource]
    ExecutionStatus,
)
from moveit.core.planning_scene import (  # type: ignore[reportMissingModuleSource]
    PlanningScene,
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
    PlanOnceError,
    TrajectoryError,
    TrajectoryErrorCodes,
)
from tabletop_rig.interfaces.moveit.planning_scene import (
    PlanningSceneInterface,
)
from tabletop_rig.interfaces.moveit.requests import PlanningGoalT, PlanRequest
from tabletop_rig.interfaces.moveit.trajectory_cache import (
    FuzzyTrajectoryCache,
)
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import (
    all_close_poses_stamped,
    all_close_robot_states,
    robot_trajectory_copy,
)


class PlanAndExecuteInterface(PlanningSceneInterface):
    def __init__(
        self,
        node: BaseNode,
        safe_to_execute_callback: Callable[[], bool],
        logger_name: str = "moveit_plan_interface",
    ):
        """Initializes the MoveItPlanInterface

        Args:
            safe_to_execute_callback: Function to evaluate before executing to
                determine if it is safe to execute
        """
        super().__init__(node, logger_name)

        # REQUIRED user callback that is checked before executing
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

    ###########################################################################
    ########## Properties #####################################################
    ###########################################################################

    @property
    def executing(self) -> bool:
        return self.execution_lock.locked()

    ###########################################################################
    ########## Parameter Convenience Properties ###############################
    ###########################################################################

    @property
    def simulate(self) -> bool:
        """Get the simulation flag."""
        return self.node.get_parameter_wrapper("simulate")

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

    def create_plan_request(
        self, goal: PlanningGoalT, **kwargs: Any
    ) -> PlanRequest:
        """Parse the planning kwargs.

        Args:
            goal: The goal to plan for.
            **kwargs: Additional keyword arguments to override the default plan
                request.

        Returns:
            A tuple of the parsed kwargs and any unused kwargs.
        """
        default_kwargs = self.node.get_parameter_wrapper("planning.defaults")
        kwargs = default_kwargs | kwargs
        return PlanRequest(goal=goal, **kwargs)

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
            assert not isinstance(request.goal, str)

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

    def post_process_trajectory(
        self, trajectory: RobotTrajectory, request: PlanRequest
    ) -> RobotTrajectory:
        """Preprocess the trajectory using the given request.

        Args:
            request: The request to preprocess the trajectory with.

        Returns:
            The preprocessed trajectory.
        """
        # Apply time parameterization and smoothing to the trajectory
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
    ) -> tuple[RobotTrajectory, PlanRequest] | None:
        """Retrieve trajectory from cache or plan a trajectory"""
        # Parse the planning args if requested
        if request is None:
            request = self.create_plan_request(*args, **kwargs)
        elif len(args) > 0 or len(kwargs) > 0:
            raise ValueError(
                f"Additional arguments ({args}) or kwargs ({kwargs}) cannot be provided if plan_request is provided"
            )
        else:
            request = copy(request)

        # Set start state if None
        if request.start_state is None:
            request.start_state = self.current_state

        # Transform goal to world frame or valid robot state
        if isinstance(request.goal, PoseStamped):
            if request.goal.header.frame_id != self.planning_frame:
                request.goal = self.change_reference_frame(
                    request.goal, self.planning_frame
                )
        elif isinstance(request.goal, str):
            request.goal = self.get_target_state(
                request.goal, request.group_name
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

        # Attempt to validate cached trajectories
        if (
            self.node.get_parameter_wrapper(
                "trajectory_cache.use_cached_trajectories"
            )
            and request.use_cache
        ):
            try:
                # TODO: Refactor so caching happens in the plan() function
                trajectories = self.trajectory_cache.get_trajectories(request)
            except KeyError:
                self.log(
                    "No cached trajectory found, planning and executing normally"
                )
            else:
                self.log(
                    "Cached trajectories found, validating in order of path length"
                )
                for trajectory in trajectories:
                    if cancel_event is not None and cancel_event.is_set():
                        raise asyncio.CancelledError("Plan cancelled")

                    try:
                        trajectory = self.post_process_trajectory(
                            trajectory, request
                        )
                        self._validate_trajectory(trajectory)
                        return trajectory, request
                    except TrajectoryError as e:
                        self.log(
                            f"Error attempting to use cached trajectory: {e}",
                            severity="WARN",
                        )
                self.log("All cached trajectories failed, planning normally")
        else:
            self.log(
                "Not using cached trajectories, planning and executing normally"
            )

        # Otherwise, plan normally

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
                break
            except PlanOnceError as e:
                self.log(
                    f"Planning attempt {i + 1}/{request.max_plan_attempts} failed: {e}",
                    severity="WARN",
                )
                errors.append(e)
        else:
            raise MaxPlanningAttemptsReachedError(errors)

        if cancel_event is not None and cancel_event.is_set():
            raise asyncio.CancelledError("Plan cancelled")

        trajectory = self.post_process_trajectory(trajectory, request)
        # TODO: Maybe revalidate
        # self._validate_trajectory(trajectory)

        return trajectory, request

    def _execute_impl(self, trajectory: RobotTrajectory):
        """Trajectory execution synchronous implementation"""
        # Check if the robot is safe to execute
        if not self._safe_to_execute_callback():
            raise NotSafeToExecuteError()

        assert not self.execution_lock.locked()
        with self.execution_lock:
            # Execute the trajectory
            initial_state = self.current_state
            self.trajectory_execution_manager.push(
                trajectory.get_robot_trajectory_msg()
            )
            execution_status: ExecutionStatus = (
                self.trajectory_execution_manager.execute_and_wait()
            )

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

    async def plan(
        self, *args: Any, request: Optional[PlanRequest] = None, **kwargs: Any
    ) -> tuple[RobotTrajectory, PlanRequest] | None:
        """Retrieve from cache or plan a trajectory to the given waypoint,
        retrying up to max_plan_attempts times until successful.

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

        See Also:
            `_plan_impl()`: For parameter and implementation details.
        """
        cancel_event = threading.Event()
        try:
            return await asyncio.to_thread(
                self._plan_impl,
                *args,
                request=request,
                cancel_event=cancel_event,
                **kwargs,
            )
        finally:
            cancel_event.set()

    def stop_execution(self):
        """Stop execution of the trajectory, if currently executing"""
        if self.execution_lock.locked():
            self.trajectory_execution_manager.stop_execution()

    async def execute(self, trajectory: RobotTrajectory):
        """Execute the given robot trajectory.

        Args:
            trajectory: Trajectory to execute

        Raises:
            NotSafeToExecuteError: If the robot is not safe to execute.
            ExecutionInterruptedError: If the robot moved but not to the goal.
            ExecutionRejectedError: If the trajectory was rejected by the robot.
        """
        try:
            return await asyncio.to_thread(self._execute_impl, trajectory)
        except asyncio.CancelledError:
            if self.execution_lock.locked():
                self.trajectory_execution_manager.stop_execution()
            raise

    def cache_trajectory(self, trajectory: RobotTrajectory, **kwargs: Any):
        """Cache the given trajectory.

        Args:
            trajectory: The trajectory to cache.
            **kwargs: Keyword arguments to pass to `FuzzyTrajectoryCache.cache_trajectory()`.
        """
        if not self.node.get_parameter_wrapper(
            "trajectory_cache.freeze_cache"
        ):
            self.trajectory_cache.cache_trajectory(trajectory, **kwargs)
            self.log("Cached trajectory successfully")
        else:
            self.log("Cache is frozen, skipping cache")

    async def plan_and_execute(
        self,
        *args: Any,
        request: Optional[PlanRequest] = None,
        cache_trajectory: bool = True,
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

        if (
            request is not None and request.start_state is not None
        ) or "start_state" in kwargs:
            raise ValueError("start_state is not allowed in plan_and_execute")

        # Plan and return immediately if already at goal
        response = await self.plan(*args, request=request, **kwargs)
        if response is None:
            return None
        trajectory, parsed_request = response

        # Execute desired request
        await self.execute(trajectory)

        # Cache the trajectory if requested
        to_cache_kwargs = {
            "trajectory": trajectory,
            "request": parsed_request,
            "true_end_state": self.current_state,
        }
        if cache_trajectory:
            self.cache_trajectory(**to_cache_kwargs)

        return to_cache_kwargs

    ###########################################################################
    ########## Reset (simulation) #############################################
    ###########################################################################

    async def clear_scene_and_reset(
        self, end_goal: Optional[PlanningGoalT] = None, **kwargs
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
        if not self.simulate:
            raise RuntimeError(
                "Planning scene can only be cleared in simulation!"
            )

        self.log(
            "Clearing planning scene and moving out of collision (simulation only)"
        )

        self.remove_all_collision_objects()
        if end_goal is None:
            end_goal = "idle"
        await self.plan_and_execute(end_goal, **kwargs)
        self._init_planning_scene()

    def __enter__(self) -> Self:
        """Enter the context manager."""
        self.trajectory_cache.__enter__()
        return self

    def __exit__(
        self,
        exc_type: Optional[type[BaseException]],
        exc_value: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ):
        """Exit the context manager."""
        self.trajectory_cache.__exit__(exc_type, exc_value, exc_tb)

    ###########################################################################
    ########## Destroy ########################################################
    ###########################################################################

    def destroy(self):
        if hasattr(self, "trajectory_cache"):
            self.trajectory_cache.close()
        # if hasattr(self, "trajectory_execution_manager"):
        #     self.trajectory_execution_manager.stop_execution()
        super().destroy()

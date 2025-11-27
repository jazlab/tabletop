import asyncio
from collections.abc import Callable, Coroutine
from enum import IntEnum
from typing import Any, Optional

from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterNotDeclaredException

from tabletop_rig.exceptions import (
    ExecutionError,
    ObjectManipulationError,
    PlanningError,
)
from tabletop_rig.interfaces.moveit.plan_and_execute import (
    PlanAndExecuteInterface,
)
from tabletop_rig.interfaces.moveit.requests import PlanGoalT
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import (
    change_reference_frame_pose_stamped,
    matrix_from_pose_msg,
    pose_stamped_msg,
)


class ObjectPhase(IntEnum):
    PRE_FETCH = 0
    PRE_ATTACH = 1
    ATTACH = 2
    POST_ATTACH = 3
    POST_FETCH = 4
    PRE_PRESENT = 5
    PRESENT = 6
    UNPRESENT = 7
    PRE_RETURN = 8
    PRE_DETACH = 9
    DETACH = 10
    POST_DETACH = 11
    POST_RETURN = 12
    IDLE = 13


OBJECT_MOUNT_PHASES = [
    ObjectPhase.PRE_ATTACH,
    ObjectPhase.ATTACH,
    ObjectPhase.POST_ATTACH,
    ObjectPhase.POST_FETCH,
    ObjectPhase.PRE_DETACH,
    ObjectPhase.DETACH,
    ObjectPhase.POST_DETACH,
    ObjectPhase.POST_RETURN,
]


def object_manipulation_lock_decorator(
    coro_fn: Callable[..., Coroutine],
) -> Callable[..., Coroutine]:
    """Decorator for methods that should be run with the object manipulation lock."""

    async def wrapper(
        self: "ObjectManipulationInterface", *args: Any, **kwargs: Any
    ):
        async with self.object_manipulation_lock:
            return await coro_fn(self, *args, **kwargs)

    return wrapper


class ObjectManipulationInterface(PlanAndExecuteInterface):
    def __init__(
        self,
        node: BaseNode,
        safe_to_execute_callback: Callable[[], bool],
        logger_name: str = "moveit_plan_interface",
    ):
        """Initializes the MoveItObjectInterface"""
        super().__init__(node, safe_to_execute_callback, logger_name)

        self.object_manipulation_lock = asyncio.Lock()

        self.object_phase = ObjectPhase.IDLE

        self.log("MoveIt object manipulation interface initialized")

    ###########################################################################
    ########## Parameter Convenience Properties ###############################
    ###########################################################################

    @property
    def object_mount_ids(self) -> list[str]:
        """Get the object mount ids from the parameter server."""
        return self.node.get_parameter_wrapper("object_manipulation.mount_ids")

    ###########################################################################
    ########## Object Manipulation Convenience Methods ########################
    ###########################################################################

    def object_init_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the initial pose of an object from the parameters."""
        return self.grid_object_poses[object_id]

    def _object_init_pose_stamped_with_offset(
        self, object_id: str, offset: list[float]
    ) -> PoseStamped:
        """Get the initial pose of an object from the parameters with an offset."""
        init_pose_stamped = self.object_init_pose_stamped(object_id)
        old_frame_transform = matrix_from_pose_msg(init_pose_stamped.pose)
        new_frame_id = init_pose_stamped.header.frame_id
        new_frame_transform = self.get_frame_transform(new_frame_id)

        pose_stamped = pose_stamped_msg(position=offset)

        return change_reference_frame_pose_stamped(
            old_pose_stamped=pose_stamped,
            old_frame_transform=old_frame_transform,
            new_frame_transform=new_frame_transform,
            new_frame_id=new_frame_id,
        )

    def pre_fetch_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-fetch pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.node.get_parameter_wrapper(
                "predefined_poses.pre_fetch_offset"
            ),
        )

    def pre_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-attach pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.node.get_parameter_wrapper(
                "predefined_poses.pre_attach_offset"
            ),
        )

    def attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the attach pose of an object."""
        return self.object_init_pose_stamped(object_id)

    def post_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-attach pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.node.get_parameter_wrapper(
                "predefined_poses.post_attach_offset"
            ),
        )

    def post_fetch_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-fetch pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.node.get_parameter_wrapper(
                "predefined_poses.post_fetch_offset"
            ),
        )

    def pre_present_pose_stamped(self, _: str) -> PoseStamped:
        """Get the pre-present pose."""
        return self.create_pose_stamped(
            **self.node.get_parameter_wrapper(
                "predefined_poses.pre_present_pose"
            )
        )

    def unpresent_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the unpresent (pre-present) pose."""
        return self.pre_present_pose_stamped(object_id)

    def pre_return_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-return (post-fetch) pose of an object."""
        return self.post_fetch_pose_stamped(object_id)

    def pre_detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-detach (post-attach) pose of an object."""
        return self.post_attach_pose_stamped(object_id)

    def detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the detach (object init) pose of an object."""
        return self.object_init_pose_stamped(object_id)

    def post_detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-detach (pre-attach) pose of an object."""
        return self.pre_attach_pose_stamped(object_id)

    def post_return_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-return (pre-fetch) pose of an object."""
        return self.pre_fetch_pose_stamped(object_id)

    ###########################################################################
    ########## Fetch, present, and return #####################################
    ###########################################################################

    def _get_phase_goal(
        self,
        phase: ObjectPhase,
        object_id: str,
        goal: PlanGoalT | None = None,
    ) -> PlanGoalT:
        """Get the goal for the given phase and object.

        Args:
            phase: The phase to get the goal for
            object_id: The ID of the object to get the goal for
            goal: The goal to use if the phase is present or unpresent

        Returns:
            The goal for the given phase and object.
        """
        match phase:
            case ObjectPhase.PRESENT:
                if goal is None:
                    raise ValueError(
                        "Goal is required for present and unpresent phases"
                    )
                return goal
            case ObjectPhase.IDLE:
                return self.get_target_state("idle")
            case ObjectPhase.PRE_PRESENT:
                try:
                    return self.get_target_state("pre_present")
                except ParameterNotDeclaredException:
                    return self.pre_present_pose_stamped(object_id)
            case _:
                return getattr(self, f"{phase.name.lower()}_pose_stamped")(
                    object_id
                )

    async def _object_phase(
        self,
        object_id: str,
        phase: ObjectPhase,
        goal: PlanGoalT | None = None,
        **kwargs: Any,
    ) -> list[dict[str, Any]] | None:
        """Plan and execute a phase of the object manipulation process.

        This is a helper function for the object manipulation process.

        Args:
            object_id: The ID of the object to manipulate
            phase: The phase to manipulate the object in
            cache_trajectory: Whether to cache the trajectory after a single
                phase
            **kwargs: Additional keyword arguments to pass to `_plan_and_execute_cached()`

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.
        """
        self.log(f"{phase.name} phase for object {object_id}")

        goal = self._get_phase_goal(phase, object_id, goal)
        extra_kwargs = {}
        extra_kwargs["planning_pipeline"] = "linear"
        extra_kwargs["use_cache"] = False

        if phase in OBJECT_MOUNT_PHASES:
            self.allow_collision(*zip(*self.allowed_object_mount_collisions))

        if phase == ObjectPhase.DETACH:
            extra_kwargs["velocity_scaling_factor"] = (
                self.node.get_parameter_wrapper(
                    "object_manipulation.detach_velocity_scaling_factor"
                )
            )

        match phase:
            case (
                ObjectPhase.PRE_FETCH
                | ObjectPhase.PRE_PRESENT
                | ObjectPhase.PRE_RETURN
                | ObjectPhase.IDLE
            ):
                del extra_kwargs["planning_pipeline"]
                del extra_kwargs["use_cache"]
            case (
                ObjectPhase.PRE_ATTACH
                | ObjectPhase.ATTACH
                | ObjectPhase.POST_DETACH
                | ObjectPhase.POST_RETURN
            ):
                self.allow_collision(object_id, self.touch_links)
            case ObjectPhase.POST_ATTACH | ObjectPhase.DETACH:
                self.allow_collision(object_id, self.object_mount_ids)

        self.log(f"{phase.name} goal: {goal}", severity="DEBUG")

        cache_kwargs = None
        try:
            cache_kwargs = await self.plan_and_execute(
                goal, **kwargs, **extra_kwargs
            )
        except PlanningError as e:
            match phase:
                case (
                    ObjectPhase.POST_FETCH
                    | ObjectPhase.PRESENT
                    | ObjectPhase.UNPRESENT
                    | ObjectPhase.PRE_DETACH
                ):
                    self.log(
                        f"Error while planning for {phase.name} phase with linear pipeline: {e}",
                        severity="WARN",
                    )
                    self.log(
                        f"Attempting to plan and execute {phase.name} phase with default pipeline",
                        severity="WARN",
                    )
                    await self.plan_and_execute(goal, **kwargs)
                case _:
                    raise
        finally:
            if phase in OBJECT_MOUNT_PHASES:
                self.disallow_collision(
                    *zip(*self.allowed_object_mount_collisions)
                )
            match phase:
                case (
                    ObjectPhase.PRE_ATTACH
                    | ObjectPhase.ATTACH
                    | ObjectPhase.POST_DETACH
                    | ObjectPhase.POST_RETURN
                ):
                    self.disallow_collision(object_id, self.touch_links)
                case ObjectPhase.POST_ATTACH | ObjectPhase.DETACH:
                    self.disallow_collision(object_id, self.object_mount_ids)

        match phase:
            case ObjectPhase.ATTACH:
                self.attach_collision_object(
                    object_id,
                    self.default_pose_link,
                    touch_links=self.touch_links,
                )
            case ObjectPhase.DETACH:
                self.detach_collision_object(object_id)

        return cache_kwargs

    @object_manipulation_lock_decorator
    async def fetch_object(
        self, object_id: str, cache_trajectories: bool = True
    ):
        """Fetch an object from its mount.

        The robot moves to the object's mount, attaches the object, and moves
        to the object's post-fetch pose. It uses cached trajectories if
        available and only caches the planned trajectories if the full fetch
        process is successful. This addresses the issue of the robot getting
        "stuck" in a state that it cannot complete the full fetch process and
        caching trajectories that are unusable.

        Args:
            object_id: The ID of the object to fetch
            cache_trajectories: Whether to cache the trajectories after fetching
                the object

        Raises:
            ValueError: If the object ID is not a valid collision object
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        self.log(f"Fetching object {object_id}")

        if len(self.attached_collision_object_ids) > 0:
            raise ObjectManipulationError(
                "Cannot fetch object while another object is attached"
            )

        # Check that the object ID is valid
        if object_id not in self.collision_object_ids:
            raise ValueError(f"{object_id} is not a valid collision object")

        # Iterate through the fetch phases, returning the object to its mount
        # if the fetch fails
        cache_kwargs: list[dict[str, Any]] = []
        try:
            for i in range(ObjectPhase.PRE_FETCH, ObjectPhase.POST_FETCH + 1):
                # self.object_phase = ObjectPhase(i)
                kwargs = await self._object_phase(
                    object_id, ObjectPhase(i), cache_trajectory=False
                )
                if kwargs is not None and cache_trajectories:
                    cache_kwargs.extend(kwargs)
        except (PlanningError, ExecutionError):
            # TODO: Move restart logic
            # self.log(
            #     f"Error while fetching object: {e}",
            #     severity="ERROR",
            # )
            # self.log("Attempting to return object to mount", severity="WARN")
            # start_idx = ObjectPhase.IDLE - i  # type: ignore
            # for i in range(start_idx, ObjectPhase.IDLE + 1):
            #     await self._object_phase(
            #         object_id, ObjectPhase(i), cache_trajectory=False
            #     )
            raise

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_lock_decorator
    async def pre_present_object(self):
        """Present the currently attached object at the specified end goal.

        Args:
            goal: The goal to present the object at
        """
        object_id = self.get_exactly_one_attached_object_id()
        self.log(f"Pre-presenting object {object_id}")

        # Pre-present phase
        await self._object_phase(object_id, ObjectPhase.PRE_PRESENT)

    @object_manipulation_lock_decorator
    async def unpresent_object(self):
        """Unpresent the currently attached object and move it to its pre-return pose."""
        object_id = self.get_exactly_one_attached_object_id()
        self.log(f"Unpresenting object {object_id}")

        # Unpresent phase
        await self._object_phase(object_id, ObjectPhase.UNPRESENT)

        # Pre-return phase
        self._pre_return_cache_kwargs = await self._object_phase(
            object_id, ObjectPhase.PRE_RETURN, cache_trajectory=False
        )

    @object_manipulation_lock_decorator
    async def return_object(self, cache_trajectories: bool = True):
        """Return an object to its original position.

        Args:
            cache_trajectories: Whether to cache the trajectories after
                returning the object

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        object_id = self.get_exactly_one_attached_object_id()
        if object_id not in self.grid_object_poses:
            raise RuntimeError(
                f"Object {object_id} is not in the grid, cannot return it"
            )
        self.log(f"Returning object {object_id}")

        cache_kwargs: list[dict[str, Any]] = []

        # Cache the unpresent trajectory if it exists
        if not hasattr(self, "_pre_return_cache_kwargs"):
            raise RuntimeError("Object was not unpresented before returning")

        if self._pre_return_cache_kwargs is not None:
            cache_kwargs.extend(self._pre_return_cache_kwargs)
            del self._pre_return_cache_kwargs

        # Iterate through the unpresenting and returning phases
        for i in range(ObjectPhase.PRE_DETACH, ObjectPhase.IDLE + 1):
            kwargs = await self._object_phase(
                object_id, ObjectPhase(i), cache_trajectory=False
            )
            if kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    ###########################################################################
    ########## Reset ##########################################################
    ###########################################################################

    async def reset_rig(self, end_goal: Optional[PlanGoalT] = None):
        """Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting rig")
        try:
            if len(self.attached_collision_object_ids) > 0:
                object_id = self.get_exactly_one_attached_object_id()
                if object_id in self.grid_object_poses:
                    self._pre_return_cache_kwargs = await self._object_phase(
                        object_id, ObjectPhase.PRE_RETURN
                    )
                    await self.return_object()
            if end_goal is not None:
                await self.plan_and_execute(end_goal)
        except (PlanningError, ExecutionError) as e:
            if self.simulate:
                self.log(
                    f"Error while resetting rig: {type(e).__name__}: {e}",
                    severity="ERROR",
                )
                self.log(
                    "Clearing planning scene and moving out of collision (simulation only)",
                    severity="WARN",
                )
                await self.clear_scene_and_reset(end_goal)
            else:
                raise

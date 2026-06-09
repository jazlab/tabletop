"""Pydantic request models for motion planning operations.

This module defines structured request objects for the MoveIt planning interface.
Using Pydantic models ensures type safety and validation for planning parameters.

The request models support:
- Single trajectory planning (PlanRequest)
- Multi-waypoint trajectory planning (ConcatPlanRequest)
- Object reset configurations (ObjectResetConfig)

Type Definitions:
    PlanGoalT: Union type for planning goals — `RobotState` (joint-space),
        `PoseStamped` (Cartesian), `str` (named target),
        `list[Constraints]` (raw motion-plan constraints, interpreted as
        an OR of alternative AND-of-sub-constraints sets), `JointStateDict`
        (absolute partial joint goal), or `JointStateDeltaDict` (relative
        partial joint goal).
    SinglePlanResponseT: Response type for single trajectory planning
    PlanResponseT: Response type for multi-trajectory planning
"""

from typing import Annotated, Any, Optional, TypedDict

from annotated_types import Gt, Le
from geometry_msgs.msg import PoseStamped
from moveit.core.planning_scene import (  # type: ignore[reportMissingModuleSource]
    PlanningScene,
)
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from moveit_msgs.msg import Constraints
from pydantic import BaseModel, NonNegativeFloat, PositiveFloat, PositiveInt

# Re-export the Constraints message type for users who want to import
# everything they need to build a PlanRequest from this module alone.
__all__ = [
    "PlanGoalT",
    "JointStateDict",
    "JointStateDeltaDict",
    "PlanRequest",
    "ConcatPlanRequest",
    "ObjectResetConfig",
    "TrajectoryCacheKwargs",
    "SinglePlanResponseT",
    "PlanResponseT",
]


def is_real_number(x: Any) -> bool:
    """Check if a value is a real number (int or float, but not bool).

    Args:
        x: Value to check.

    Returns:
        True if x is a numeric type (excluding bool), False otherwise.
    """
    if isinstance(x, (float, int)) and not isinstance(x, bool):
        return True
    return False


class JointStateDict(dict[str, float]):
    """Absolute joint-space goal as a mapping of joint name to position.

    A partial mapping is allowed: only the joints you want to move need to be
    present. Any active joint of the planning group that is *not* listed is
    filled in with its position from the start state (the current robot state
    when no explicit start state is given), so unspecified joints are held
    where they already are.

    This is a thin ``dict[str, float]`` subclass so the planning pipeline can
    distinguish an absolute joint goal from a relative one (`JointStateDeltaDict`)
    and from a plain ``dict`` via ``isinstance``. Construct it like a dict::

        JointStateDict({"left_shoulder_pan_joint": 1.57})

    Positions follow REP 103 units (radians for revolute joints, meters for
    prismatic joints).
    """


class JointStateDeltaDict(dict[str, float]):
    """Relative joint-space goal as a mapping of joint name to delta.

    Like `JointStateDict`, but each value is *added* to the corresponding joint's
    start position rather than replacing it. Unspecified joints are left at
    their start position (equivalent to a delta of ``0.0``).

    A distinct subclass from `JointStateDict` so the two can be told apart by
    ``isinstance``. Construct it like a dict::

        # rotate the elbow back by 0.2 rad, leave every other joint put
        JointStateDeltaDict({"left_elbow_joint": -0.2})

    Deltas follow REP 103 units (radians for revolute joints, meters for
    prismatic joints).
    """


PlanGoalT = (
    RobotState
    | PoseStamped
    | str
    | list[Constraints]
    | JointStateDict
    | JointStateDeltaDict
)
"""Type alias for planning goal types.

May be one of:
- `RobotState`: joint-space goal.
- `PoseStamped`: Cartesian goal for `pose_link` (frame-converted internally).
- `str`: name of a predefined joint state in the SRDF.
- `list[Constraints]`: raw motion-plan constraints, passed through to the
    planner's `setGoal(vector<Constraints>)` overload. The list is
    interpreted as an OR of alternatives; each `Constraints` is an AND
    of its sub-constraints. Goals of this kind bypass the trajectory
    cache (no canonical end-state to key on).
- `JointStateDict`: absolute partial joint goal (dict of joint name -> position).
    Unprovided joints are filled from the start state, then the goal is
    resolved to a `RobotState` internally.
- `JointStateDeltaDict`: relative partial joint goal (dict of joint name ->
    delta). Each delta is added to the start-state position; unprovided
    joints are unchanged. Resolved to a `RobotState` internally.
"""

BoundFloat = Annotated[float, Gt(0.0), Le(1.0)]


class _BasePlanRequest(
    BaseModel,
    validate_assignment=True,
    arbitrary_types_allowed=True,
    extra="forbid",
):
    """Base class for planning request parameters.

    Contains common parameters shared between single and multi-waypoint
    planning requests including trajectory post-processing options.

    Attributes:
        group_name: MoveIt planning group name.
        start_state: Starting robot configuration. If None, uses current state.
        pose_link: End-effector link for Cartesian goals.
        planning_pipeline: Which planner to use (e.g., "linear", "ptp", "aps_rrt_star").
        path_constraints: Optional path constraints for planning.
        planning_scene: Custom planning scene. If None, uses current scene.
        planning_time: Allowed planning time for one planning attempt.
        max_attempts: Maximum planning attempts before failing.
        exp_backoff_factor: Planning time is multiplied by this factor after each failed attempt.
        use_cache: Whether to use the trajectory cache.
        apply_totg: Apply Time-Optimal Trajectory Generation.
        apply_smoothing: Apply trajectory smoothing.
        velocity_scaling_factor: Scale factor for velocity limits (0.0-1.0).
        acceleration_scaling_factor: Scale factor for acceleration limits (0.0-1.0).
        path_tolerance: Tolerance for path approximation.
        resample_dt: Time step for trajectory resampling.
        min_angle_change: Minimum angle change threshold for waypoints.
        mitigate_overshoot: Enable overshoot mitigation in TOTG.
        overshoot_threshold: Threshold for overshoot detection.
    """

    start_state: Optional[RobotState] = None
    pose_link: Optional[str] = None
    group_name: Optional[str] = None
    planning_pipeline: Optional[str] = None
    path_constraints: Optional[Constraints] = None
    planning_scene: Optional[PlanningScene] = None
    planning_time: Optional[PositiveFloat] = None
    max_attempts: Optional[PositiveInt] = None
    exp_backoff_factor: Optional[PositiveFloat] = None
    use_cache: bool = True
    apply_totg: bool = True
    apply_smoothing: bool = False
    velocity_scaling_factor: BoundFloat = 1.0
    acceleration_scaling_factor: BoundFloat = 1.0
    path_tolerance: PositiveFloat = 0.001
    resample_dt: PositiveFloat = 0.05
    min_angle_change: PositiveFloat = 0.001
    mitigate_overshoot: bool = False
    overshoot_threshold: PositiveFloat = 0.002


class PlanRequest(
    _BasePlanRequest,
    validate_assignment=True,
    arbitrary_types_allowed=True,
    extra="forbid",
):
    """Request for planning a single trajectory to a goal.

    Extends _BasePlanRequest with a single goal specification.

    Attributes:
        goal: The planning goal as a RobotState (joint space), PoseStamped
            (Cartesian space), string (named target), or a list of
            moveit_msgs/Constraints (raw constraint goal; OR of alternatives).
    """

    goal: PlanGoalT


class ConcatPlanRequest(
    _BasePlanRequest,
    validate_assignment=True,
    arbitrary_types_allowed=True,
    extra="forbid",
):
    """Request for planning and concatenating multiple trajectory segments.

    Extends _BasePlanRequest with multiple waypoint goals. The resulting
    trajectory visits each goal in sequence.

    Attributes:
        goals: List of planning goals to visit in order. Each element may be
            any `PlanGoalT` — including `list[Constraints]`, which makes the
            outer `goals` field a `list[list[Constraints]]` for that entry.
        dts: Optional dwell times at each waypoint (in seconds).
        loop: If True, add a segment returning to the first goal.
        post_process_after_concat: Apply TOTG/smoothing after concatenation
            rather than per-segment.
    """

    goals: list[PlanGoalT]
    dts: Optional[list[NonNegativeFloat]] = None
    loop: bool = False
    post_process_after_concat: bool = False

    def generate_plan_requests(self) -> list[PlanRequest]:
        """Generate individual PlanRequests for each goal.

        Creates a PlanRequest for each goal in the goals list, inheriting
        all parameters from this request except start_state (which only
        applies to the first segment).

        Returns:
            List of PlanRequest objects, one per goal.
        """
        kwargs = {}
        for name in _BasePlanRequest.model_fields.keys():
            if name == "start_state":
                continue
            kwargs[name] = self.__getattribute__(name)

        requests: list[PlanRequest] = []
        for goal in self.goals:
            requests.append(PlanRequest(goal=goal, **kwargs))

        requests[0].start_state = self.start_state

        return requests


class ObjectResetConfig(
    BaseModel,
    validate_assignment=True,
    arbitrary_types_allowed=True,
    extra="forbid",
):
    """Configuration for resetting an object to its initial position.

    Defines the motion sequence and collision settings for returning
    a manipulated object to its starting location.

    Attributes:
        start_goal: Goal position where the object starts (for fetching).
        reset_request: Multi-waypoint request for the reset motion sequence.
        object_allowed_collision_ids: IDs of objects to allow collision with
            the manipulated object during reset.
        additional_allowed_collisions: Additional collision pairs to ignore
            as (link1, link2) tuples.
    """

    start_goal: PlanGoalT
    reset_request: ConcatPlanRequest
    object_allowed_collision_ids: Optional[list[str]] = None
    additional_allowed_collisions: Optional[list[tuple[str, str]]] = None


class TrajectoryCacheKwargs(TypedDict):
    """Keyword arguments for caching a trajectory.

    Attributes:
        trajectory: The planned trajectory to cache.
        request: The original planning request.
        true_end_state: Optional actual end state (may differ from planned).
    """

    trajectory: RobotTrajectory
    request: PlanRequest


SinglePlanResponseT = tuple[RobotTrajectory, TrajectoryCacheKwargs | None]
"""Response type for single trajectory planning: (trajectory, cache_kwargs)."""

PlanResponseT = tuple[RobotTrajectory, list[TrajectoryCacheKwargs] | None]
"""Response type for multi-trajectory planning: (combined_trajectory, cache_kwargs_list)."""

from typing import Any, NotRequired, Optional, TypedDict

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
from pydantic import BaseModel


def is_real_number(x: Any) -> bool:
    if isinstance(x, (float, int)) and not isinstance(x, bool):
        return True
    return False


PlanGoalT = RobotState | PoseStamped | str


class _BasePlanRequest(
    BaseModel,
    validate_assignment=True,
    arbitrary_types_allowed=True,
    extra="forbid",
):
    """Request to plan a trajectory"""

    start_state: Optional[RobotState] = None
    pose_link: Optional[str] = None
    group_name: Optional[str] = None
    planning_pipeline: Optional[str] = None
    path_constraints: Optional[Constraints] = None
    planning_scene: Optional[PlanningScene] = None
    max_attempts: int = 3
    use_cache: bool = True
    apply_totg: bool = True
    apply_smoothing: bool = False
    velocity_scaling_factor: float = 1.0
    acceleration_scaling_factor: float = 1.0
    path_tolerance: float = 0.001
    resample_dt: float = 0.05
    min_angle_change: float = 0.001
    mitigate_overshoot: bool = False
    overshoot_threshold: float = 0.002


class PlanRequest(
    _BasePlanRequest,
    validate_assignment=True,
    arbitrary_types_allowed=True,
    extra="forbid",
):
    """Request to plan a trajectory"""

    goal: PlanGoalT


class ConcatPlanRequest(
    _BasePlanRequest,
    validate_assignment=True,
    arbitrary_types_allowed=True,
    extra="forbid",
):
    """Request to plan a series of trajectories and concatenate them"""

    goals: list[PlanGoalT]
    dts: Optional[list[float]] = None
    loop: bool = False
    post_process_after_concat: bool = False

    def generate_plan_requests(self) -> list[PlanRequest]:
        """Generate a PlanRequest for each goal in 'goals'

        The 'start_state' for the first PlanRequest is set to self.start_state.

        The remaining parameters for each PlanRequest are shared from self (not copied).
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
    """Request to reset an object"""

    start_goal: PlanGoalT
    reset_request: ConcatPlanRequest
    object_allowed_collision_ids: Optional[list[str]] = None
    additional_allowed_collisions: Optional[list[tuple[str, str]]] = None


class TrajectoryCacheKwargs(TypedDict):
    trajectory: RobotTrajectory
    request: PlanRequest
    true_end_state: NotRequired[RobotState]


SinglePlanResponseT = tuple[RobotTrajectory, TrajectoryCacheKwargs | None]
PlanResponseT = tuple[RobotTrajectory, list[TrajectoryCacheKwargs] | None]

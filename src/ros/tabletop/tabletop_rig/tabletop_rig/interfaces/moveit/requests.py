from abc import ABC, abstractmethod
from dataclasses import dataclass, fields
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


def is_real_number(x: Any) -> bool:
    if isinstance(x, (float, int)) and not isinstance(x, bool):
        return True
    return False


PlanGoalT = RobotState | PoseStamped | str


@dataclass
class TypeCheckedDataclass(ABC):
    """Adds type checking to dataclass"""

    def __post_init__(self):
        """Type check the request."""
        for name in self.__dataclass_fields__:
            self._validate_attribute(name, getattr(self, name))

    def __setattr__(self, name: str, value: Any) -> None:
        """Set an attribute."""
        if name not in self.__dataclass_fields__:
            raise AttributeError(f"Invalid attribute: {name}")
        self._validate_attribute(name, value)
        object.__setattr__(self, name, value)

    @abstractmethod
    def _validate_attribute(self, name: str, value: Any):
        """Check the type of an attribute

        This should be implement logic for type checking any type strict
        attributes, and should raise TypeError if a type check fails

        Exact type checking implementation is the inheriting class's
        responsibility
        """


@dataclass
class _BasePlanRequest(TypeCheckedDataclass):
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

    def _validate_attribute(self, name: str, value: Any):
        """Check the type of an attribute."""

        type_error = False
        match name:
            case "start_state":
                if value is not None and not isinstance(value, RobotState):
                    type_error = True
            case "pose_link":
                if value is not None and not isinstance(value, str):
                    type_error = True
            case "group_name":
                if value is not None and not isinstance(value, str):
                    type_error = True
            case "planning_pipeline":
                if value is not None and not isinstance(value, str):
                    type_error = True
            case "path_constraints":
                if value is not None and not isinstance(value, Constraints):
                    type_error = True
            case "planning_scene":
                if value is not None and not isinstance(value, PlanningScene):
                    type_error = True
            case "use_cache":
                if not isinstance(value, bool):
                    type_error = True
            case "max_attempts":
                if not isinstance(value, int):
                    type_error = True
            case "apply_totg":
                if not isinstance(value, bool):
                    type_error = True
            case "apply_smoothing":
                if not isinstance(value, bool):
                    type_error = True
            case "velocity_scaling_factor":
                if not is_real_number(value):
                    type_error = True
            case "acceleration_scaling_factor":
                if not is_real_number(value):
                    type_error = True
            case "path_tolerance":
                if not is_real_number(value):
                    type_error = True
            case "resample_dt":
                if not is_real_number(value):
                    type_error = True
            case "min_angle_change":
                if not is_real_number(value):
                    type_error = True
            case "mitigate_overshoot":
                if not isinstance(value, bool):
                    type_error = True
            case "overshoot_threshold":
                if not is_real_number(value):
                    type_error = True
            case _:
                raise AssertionError(f"Invalid attribute: {name}")

        if type_error:
            raise ValueError(f"Invalid {name} type: {type(value)}")


@dataclass(kw_only=True)
class PlanRequest(_BasePlanRequest):
    """Request to plan a trajectory"""

    goal: PlanGoalT

    def _validate_attribute(self, name: str, value: Any):
        """Check the type of an attribute."""

        type_error = False
        match name:
            case "goal":
                if not isinstance(value, (RobotState, PoseStamped, str)):
                    type_error = True
            case _:
                super()._validate_attribute(name, value)

        if type_error:
            raise ValueError(f"Invalid {name} type: {type(value)}")


@dataclass(kw_only=True)
class ConcatPlanRequest(_BasePlanRequest):
    """Request to plan a series of trajectories and concatenate them"""

    goals: list[PlanGoalT]
    dts: Optional[list[float]] = None
    loop: bool = False
    post_process_after_concat: bool = False

    # def __init__(
    #     self,
    #     *,
    #     requests: Optional[list[PlanRequest]] = None,
    #     goals: Optional[list[PlanGoalT]] = None,
    #     dts: Optional[list[float]] = None,
    #     loop: bool = False,
    #     post_process_after_concat: bool = False,
    #     **common_kwargs: Any,
    # ):
    #     if requests is not None:
    #         if goals is not None:
    #             raise ValueError(
    #                 "'goals' cannot be provided if 'requests' is provided"
    #             )
    #         # if len(common_kwargs) > 0:
    #         #     raise ValueError(
    #         #         "No additional 'common_kwargs' can be provided if "
    #         #         "'requests' is provided"
    #         #     )
    #         # requests = [copy(request) for request in requests]
    #     elif goals is not None:
    #         kwargs = common_kwargs.copy()
    #         if "start_state" in kwargs:
    #             del kwargs["start_state"]
    #
    #         if post_process_after_concat:
    #             kwargs["apply_totg"] = False
    #             kwargs["apply_smoothing"] = False
    #
    #         requests = []
    #         for goal in goals:
    #             requests.append(PlanRequest(goal=goal, **kwargs))
    #     else:
    #         raise ValueError("Either 'goals' or 'requests' must be provided")
    #
    #     self.requests = requests
    #     self.dts = dts
    #     self.loop = loop
    #     self.post_process_after_concat = post_process_after_concat
    #
    #     for key, value in common_kwargs.items():
    #         self.__setattr__(key, value)

    def _validate_attribute(self, name: str, value: Any):
        """Check the type of an attribute."""

        type_error = False
        match name:
            case "goals":
                if not isinstance(value, list):
                    type_error = True
                else:
                    for i, goal in enumerate(value):
                        if not isinstance(
                            goal, (RobotState, PoseStamped, str)
                        ):
                            raise ValueError(
                                f"Invalid goals[{i}] type: {type(goal)}"
                            )
            case "dts":
                if value is not None:
                    if not isinstance(value, list):
                        type_error = True
                    else:
                        for i, dt in enumerate(value):
                            if not is_real_number(dt):
                                raise ValueError(
                                    f"Invalid dts[{i}] type: {type(dt)}"
                                )
            case "loop":
                if not isinstance(value, bool):
                    type_error = True
            case "post_process_after_concat":
                if not isinstance(value, bool):
                    type_error = True
            case _:
                super()._validate_attribute(name, value)

        if type_error:
            raise ValueError(f"Invalid {name} type: {type(value)}")

    def generate_plan_requests(self) -> list[PlanRequest]:
        """Generate a PlanRequest for each goal in 'goals'

        The 'start_state' for the first PlanRequest is set to self.start_state.

        The remaining parameters for each PlanRequest are shared from self (not copied).
        """
        kwargs = {}
        for field in fields(_BasePlanRequest):
            if field.name == "start_state":
                continue
            kwargs[field.name] = self.__getattribute__(field.name)

        requests: list[PlanRequest] = []
        for goal in self.goals:
            requests.append(PlanRequest(goal=goal, **kwargs))

        requests[0].start_state = self.start_state

        return requests


@dataclass(kw_only=True)
class ObjectResetConfig(TypeCheckedDataclass):
    """Request to reset an object"""

    start_goal: PlanGoalT
    reset_request: ConcatPlanRequest
    allowed_collision_ids: Optional[list[str]] = None

    def _validate_attribute(self, name: str, value: Any):
        """Check the type of an attribute."""

        type_error = False
        match name:
            case "start_goal":
                if not isinstance(value, (RobotState, PoseStamped, str)):
                    type_error = True
            case "reset_request":
                if not isinstance(value, ConcatPlanRequest):
                    type_error = True
            case "allowed_collision_ids":
                if value is not None:
                    if not isinstance(value, list):
                        type_error = True
                    else:
                        for i, collision_id in enumerate(value):
                            if not isinstance(collision_id, str):
                                raise ValueError(
                                    f"Invalid allowed_collision_ids[{i}] type: {type(collision_id)}"
                                )
            case _:
                super()._validate_attribute(name, value)

        if type_error:
            raise ValueError(f"Invalid {name} type: {type(value)}")


class TrajectoryCacheKwargs(TypedDict):
    trajectory: RobotTrajectory
    request: PlanRequest
    true_end_state: NotRequired[RobotState]


SinglePlanResponseT = tuple[RobotTrajectory, TrajectoryCacheKwargs | None]
PlanResponseT = tuple[RobotTrajectory, list[TrajectoryCacheKwargs] | None]

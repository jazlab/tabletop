from dataclasses import dataclass
from typing import Any, Optional

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

PlanGoalT = RobotState | PoseStamped | str


def is_real_number(x: Any) -> bool:
    if isinstance(x, (float, int)) and not isinstance(x, bool):
        return True
    return False


@dataclass(kw_only=True)
class BasePlanRequest:
    """Request to plan a trajectory"""

    start_state: Optional[RobotState] = None
    pose_link: Optional[str] = None
    group_name: Optional[str] = None
    planning_pipeline: Optional[str] = None
    path_constraints: Optional[Constraints] = None
    planning_scene: Optional[PlanningScene] = None
    max_attempts: int = 1
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

    def _validate_attribute(self, name: str, value: Any):
        """Check the types of the request."""
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
            case "max_attempts":
                if not isinstance(value, int):
                    type_error = True
            case _:
                raise AssertionError(f"Invalid attribute: {name}")

        if type_error:
            raise ValueError(f"Invalid {name} type: {type(value)}")


@dataclass(kw_only=True)
class PlanRequest(BasePlanRequest):
    """Request to plan a trajectory"""

    goal: PlanGoalT

    def _validate_attribute(self, name: str, value: Any):
        """Check the types of the request."""
        type_error = False
        match name:
            case "goal":
                if not isinstance(value, (RobotState, PoseStamped, str)):
                    type_error = True
            case _:
                super()._validate_attribute(name, value)

        if type_error:
            raise ValueError(f"Invalid {name} type: {type(value)}")


@dataclass(init=False)
class ConcatPlanRequest(BasePlanRequest):
    """Request to plan a series of trajectories and concatenate them"""

    requests: list[PlanRequest]
    dts: Optional[list[float]]
    loop: bool
    post_process_after_concat: bool

    def __init__(
        self,
        *,
        goals: Optional[list[PlanGoalT]] = None,
        requests: Optional[list[PlanRequest]] = None,
        request_kwargs: Optional[list[dict[str, Any]]] = None,
        dts: Optional[list[float]] = None,
        loop: bool = False,
        post_process_after_concat: bool = False,
        **kwargs: Any,
    ):
        if requests is not None:
            if goals is not None or request_kwargs is not None:
                raise ValueError(
                    "Neither 'goals' nor 'request_kwargs' cannot be provided if 'requests' is provided"
                )
            # requests = [copy(request) for request in requests]
        elif goals is not None:
            if request_kwargs is None:
                request_kwargs = [{}] * len(goals)
            elif len(request_kwargs) != len(goals):
                raise ValueError(
                    "request_kwargs must be the same length as goals"
                )

            common_kwargs = kwargs.copy()
            if "start_state" in common_kwargs:
                del common_kwargs["start_state"]

            if post_process_after_concat:
                common_kwargs["apply_totg"] = False
                common_kwargs["apply_smoothing"] = False

            requests = []
            for goal, req_kwargs in zip(goals, request_kwargs):
                req_kwargs = common_kwargs | req_kwargs
                requests.append(PlanRequest(goal=goal, **req_kwargs))
        else:
            raise ValueError(
                "Either 'goals' and 'request_kwargs', or 'requests', must be provided"
            )

        self.requests = requests
        self.dts = dts
        self.loop = loop
        self.post_process_after_concat = post_process_after_concat

        for key, value in kwargs.items():
            self.__setattr__(key, value)

    def _validate_attribute(self, name: str, value: Any):
        """Check the types of the request."""
        type_error = False
        match name:
            case "requests":
                if not isinstance(value, list):
                    type_error = True
                else:
                    for i, request in enumerate(value):
                        if not isinstance(request, PlanRequest):
                            raise ValueError(
                                f"Invalid requests[{i}] type: {type(request)}"
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


PlanResponseT = tuple[RobotTrajectory | None, list[dict[str, Any]]]

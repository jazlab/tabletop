from dataclasses import dataclass
from typing import Any, Optional

from geometry_msgs.msg import PoseStamped
from moveit.core.planning_scene import (  # type: ignore[reportMissingModuleSource]
    PlanningScene,
)
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit_msgs.msg import Constraints

PlanningGoalT = RobotState | PoseStamped | str


@dataclass(slots=True, kw_only=True)
class PlanRequest:
    """Request for a plan."""

    goal: PlanningGoalT
    start_state: Optional[RobotState] = None
    pose_link: Optional[str] = None
    group_name: Optional[str] = None
    planning_pipeline: Optional[str] = None
    path_constraints: Optional[Constraints] = None
    planning_scene: Optional[PlanningScene] = None
    max_plan_attempts: int = 1
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
        for name in PlanRequest.__slots__:
            self._validate_attribute(name, getattr(self, name))

    def _validate_attribute(self, name: str, value: Any):
        """Check the types of the request."""
        if name not in PlanRequest.__slots__:
            raise AttributeError(f"Invalid attribute: {name}")

        match name:
            case "goal":
                if not isinstance(value, (RobotState, PoseStamped, str)):
                    raise ValueError(f"Invalid goal type: {type(value)}")
            case "start_state":
                if value is not None and not isinstance(value, RobotState):
                    raise ValueError(
                        f"Invalid start state type: {type(value)}"
                    )
            case "pose_link":
                if value is not None and not isinstance(value, str):
                    raise ValueError(f"Invalid pose link type: {type(value)}")
            case "group_name":
                if value is not None and not isinstance(value, str):
                    raise ValueError(f"Invalid group name type: {type(value)}")
            case "planning_pipeline":
                if value is not None and not isinstance(value, str):
                    raise ValueError(
                        f"Invalid planning pipeline type: {type(value)}"
                    )
            case "path_constraints":
                if value is not None and not isinstance(value, Constraints):
                    raise ValueError(
                        f"Invalid path constraints type: {type(value)}"
                    )
            case "planning_scene":
                if value is not None and not isinstance(value, PlanningScene):
                    raise ValueError(
                        f"Invalid planning scene type: {type(value)}"
                    )
            case "use_cache":
                if not isinstance(value, bool):
                    raise ValueError(f"Invalid use cache type: {type(value)}")
            case "apply_totg":
                if not isinstance(value, bool):
                    raise ValueError(f"Invalid apply totg type: {type(value)}")
            case "apply_smoothing":
                if not isinstance(value, bool):
                    raise ValueError(
                        f"Invalid apply smoothing type: {type(value)}"
                    )
            case "velocity_scaling_factor":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid velocity scaling factor type: {type(value)}"
                    )
            case "acceleration_scaling_factor":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid acceleration scaling factor type: {type(value)}"
                    )
            case "path_tolerance":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid path tolerance type: {type(value)}"
                    )
            case "resample_dt":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid resample dt type: {type(value)}"
                    )
            case "min_angle_change":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid min angle change type: {type(value)}"
                    )
            case "mitigate_overshoot":
                if not isinstance(value, bool):
                    raise ValueError(
                        f"Invalid mitigate overshoot type: {type(value)}"
                    )
            case "overshoot_threshold":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid overshoot threshold type: {type(value)}"
                    )
            case "max_plan_attempts":
                if not isinstance(value, int):
                    raise ValueError(
                        f"Invalid max plan attempts type: {type(value)}"
                    )
            case _:
                raise ValueError(f"Invalid attribute: {name}")

    def __setattr__(self, name: str, value: Any) -> None:
        """Set an attribute."""
        self._validate_attribute(name, value)
        object.__setattr__(self, name, value)

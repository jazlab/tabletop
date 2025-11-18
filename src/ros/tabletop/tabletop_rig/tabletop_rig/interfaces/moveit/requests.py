from dataclasses import dataclass
from typing import Any

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

PlanningGoalT = RobotState | PoseStamped | str


@dataclass(slots=True, kw_only=True)
class PlanRequest:
    """Request for a plan."""

    goal: RobotState | PoseStamped
    start_state: RobotState
    pose_link: str
    group_name: str
    planning_pipeline: str
    path_constraints: Constraints | None
    planning_scene: PlanningScene | None
    max_plan_attempts: int

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
                if not isinstance(value, (RobotState, PoseStamped)):
                    raise ValueError(f"Invalid goal type: {type(value)}")
            case "start_state":
                if not isinstance(value, RobotState):
                    raise ValueError(
                        f"Invalid start state type: {type(value)}"
                    )
            case "pose_link":
                if not isinstance(value, str):
                    raise ValueError(f"Invalid pose link type: {type(value)}")
            case "group_name":
                if not isinstance(value, str):
                    raise ValueError(f"Invalid group name type: {type(value)}")
            case "planning_pipeline":
                if not isinstance(value, str):
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


@dataclass(slots=True, kw_only=True)
class ExecuteRequest:
    """Request for an execute."""

    trajectory: RobotTrajectory
    validate_trajectory: bool
    apply_totg: bool
    apply_smoothing: bool
    velocity_scaling_factor: float
    acceleration_scaling_factor: float
    path_tolerance: float
    resample_dt: float
    min_angle_change: float
    mitigate_overshoot: bool
    overshoot_threshold: float

    def __post_init__(self):
        """Type check the request."""
        for name in ExecuteRequest.__slots__:
            self._validate_attribute(name, getattr(self, name))

    def _validate_attribute(self, name: str, value: Any):
        """Check the types of the request."""
        if name not in ExecuteRequest.__slots__:
            raise AttributeError(f"Invalid attribute: {name}")

        match name:
            case "trajectory":
                if not isinstance(value, RobotTrajectory):
                    raise ValueError(f"Invalid trajectory type: {type(value)}")
            case "validate_trajectory":
                if not isinstance(value, bool):
                    raise ValueError(
                        f"Invalid validate trajectory type: {type(value)}"
                    )
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
            case "max_execution_attempts":
                if not isinstance(value, int):
                    raise ValueError(
                        f"Invalid max execution attempts type: {type(value)}"
                    )
            case _:
                raise ValueError(f"Invalid attribute: {name}")

    def __setattr__(self, name: str, value: Any) -> None:
        """Set an attribute."""
        self._validate_attribute(name, value)
        object.__setattr__(self, name, value)

from enum import Enum
from typing import Any, Optional

from moveit.core.controller_manager import (  # type: ignore[reportMissingModuleSource]
    ExecutionStatus,
)
from moveit_msgs.msg import MoveItErrorCodes

# Constants

MOVEIT_ERROR_CODE_MAP = {
    v: k
    for k, v in type(
        MoveItErrorCodes
    )._Metaclass_MoveItErrorCodes__constants.items()  # type: ignore
}
"""MoveIt error code map from error code to string, for logging."""


# Enums


class TrajectoryErrorCodes(Enum):
    """Trajectory error codes."""

    TOTG_FAILED = -1
    SMOOTHING_FAILED = -2
    INVALID_TRAJECTORY = -3


# Exceptions


class ROSSleepError(Exception):
    """Error while sleeping in a ROS node."""


class ServiceCallTimeoutError(Exception):
    """Service call timed out."""


class ServiceCallUnsuccessfulError(Exception):
    """Service call returned with a failure status."""


class ActionCallUnsuccessfulError(Exception):
    """Action call failed."""


class CommanderRecoverableError(Exception):
    """Recoverable error that can be retried."""


class PlanningError(CommanderRecoverableError):
    """Planning error."""


class PlanOnceError(PlanningError):
    """Planning error."""

    def __init__(self, error_code: MoveItErrorCodes):
        self.error_code = error_code
        super().__init__(
            f"Plan once error: {MOVEIT_ERROR_CODE_MAP[error_code.val]}"
        )

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, PlanOnceError):
            return self.error_code.val == other.error_code.val
        return False


class MaxPlanningAttemptsReachedError(PlanningError):
    """Maximum number of planning attempts reached."""

    def __init__(self, errors: list[PlanOnceError]):
        self.errors = errors
        if all(e == errors[0] for e in errors):
            error_code_str = f"same error: {errors[0]}"
        else:
            error_code_strs = [str(e) for e in errors]
            error_code_str = f"different errors: {error_code_strs}"
        super().__init__(
            f"Max planning attempts ({len(errors)}) reached with {error_code_str}"
        )


class ExecutionError(CommanderRecoverableError):
    """Execution error."""


class TrajectoryError(ExecutionError):
    """Trajectory error."""

    def __init__(self, error_code: TrajectoryErrorCodes):
        self.error_code = error_code
        super().__init__(f"Trajectory error: {error_code}")

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, TrajectoryError):
            return self.error_code == other.error_code
        return False


class ExecutionRejectedError(ExecutionError):
    """Execution rejected (robot did not move)."""

    def __init__(self, execution_status: ExecutionStatus):
        self.execution_status = execution_status
        super().__init__(f"Execution rejected: {execution_status.status}")

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, ExecutionRejectedError):
            return (
                self.execution_status.status == other.execution_status.status
            )
        return False


class ExecutionInterruptedError(ExecutionError):
    """Execution interrupted (robot moved but not to the goal)."""

    def __init__(self, execution_status: ExecutionStatus):
        self.execution_status = execution_status
        super().__init__(f"Execution interrupted: {execution_status.status}")


class NotSafeToExecuteError(ExecutionError):
    """Not safe to execute."""

    def __init__(self, execution_status: Optional[ExecutionStatus] = None):
        self.execution_status = execution_status
        msg = "Not safe to execute"
        if execution_status is not None:
            msg += f": {execution_status.status}"
        super().__init__(msg)


class ObjectManipulationError(CommanderRecoverableError):
    """Error while manipulating object."""

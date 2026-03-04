"""Custom exceptions and error codes for the tabletop_rig package.

This module defines a hierarchy of exceptions used for handling errors during
motion planning, trajectory execution, and object manipulation. It also provides
utility mappings for MoveIt error codes and custom trajectory error codes.

Exception Hierarchy:
    Exception
    ├── ROSSleepError
    ├── ServiceCallTimeoutError
    ├── ServiceCallUnsuccessfulError
    ├── ActionError
    └── MoveitRecoverableError
        ├── PlanningError
        │   ├── PlanOnceError
        │   ├── MaxPlanningAttemptsReachedError
        │   └── TrajectoryError
        ├── ExecutionError
        │   ├── ExecutionRejectedError
        │   ├── ExecutionInterruptedError
        │   └── NotSafeToExecuteError
        └── ObjectManipulationError
"""

from enum import Enum
from typing import Any, Optional

from moveit.core.controller_manager import (  # type: ignore[reportMissingModuleSource]
    ExecutionStatus,
)
from moveit_msgs.msg import MoveItErrorCodes

# Constants

MOVEIT_ERROR_CODE_MAP: dict[int, str] = {
    v: k
    for k, v in type(
        MoveItErrorCodes
    )._Metaclass_MoveItErrorCodes__constants.items()  # type: ignore
}
"""dict[int, str]: Maps MoveIt integer error codes to their string names.

This mapping is dynamically generated from the MoveItErrorCodes message
constants, enabling human-readable error messages in exception handling.
"""


# Enums


class TrajectoryErrorCodes(Enum):
    """Error codes for trajectory post-processing failures.

    These codes indicate specific failures during trajectory optimization
    and validation that occur after initial motion planning succeeds.

    Attributes:
        TOTG_FAILED: Time-Optimal Trajectory Generation failed to compute
            valid time parameterization for the path.
        SMOOTHING_FAILED: Trajectory smoothing algorithm failed to produce
            a valid smoothed trajectory.
        INVALID_TRAJECTORY: The trajectory is invalid (e.g., empty, contains
            NaN values, or violates kinematic constraints).
    """

    TOTG_FAILED = -1
    SMOOTHING_FAILED = -2
    INVALID_TRAJECTORY = -3


# Exceptions


class ROSSleepError(Exception):
    """Raised when a ROS sleep operation fails or is interrupted.

    This typically occurs when the ROS context is shut down while a node
    is attempting to sleep, indicating that the node should terminate.
    """


class ServiceCallTimeoutError(Exception):
    """Raised when a ROS service call exceeds its timeout duration.

    This indicates that the service server did not respond within the
    expected time, which may suggest the server is unavailable, overloaded,
    or has crashed.
    """


class ServiceCallUnsuccessfulError(Exception):
    """Raised when a ROS service call completes but returns a failure status.

    Unlike ServiceCallTimeoutError, this indicates the service responded
    but reported that the requested operation could not be completed.
    """


class ActionError(Exception):
    """Raised when a ROS action call fails.

    This covers action failures including rejected goals, aborted execution,
    or any other non-successful terminal state.
    """


class ActionServerWaitTimeoutError(ActionError):
    """Raised when waiting for a ROS action server times out."""


class ActionGoalNotAcceptedError(ActionError):
    """Raised when a ROS action goal request is not accepted."""


class ActionResultUnsuccessfulError(ActionError):
    """Raised when a ROS action get result request succeed returns an unsuccessful status."""


class MoveitRecoverableError(Exception):
    """Base class for MoveIt errors that may be recoverable through retry.

    Subclasses of this exception indicate failures in motion planning or
    execution that are potentially transient and may succeed if retried,
    such as planning failures due to unlucky random sampling or brief
    environmental changes.
    """


class PlanningError(MoveitRecoverableError):
    """Base class for errors that occur during motion planning.

    This includes failures in path planning, trajectory optimization,
    and trajectory validation stages.
    """


class PlanOnceError(PlanningError):
    """Raised when a single planning attempt fails.

    This exception wraps a MoveIt error code from a failed planning request
    and provides human-readable error messages.

    Attributes:
        error_code: The MoveItErrorCodes message containing the failure reason.
    """

    def __init__(self, error_code: MoveItErrorCodes) -> None:
        """Initialize with the MoveIt error code.

        Args:
            error_code: The MoveItErrorCodes message from the failed plan.
        """
        self.error_code = error_code
        super().__init__(
            f"Plan once error: {MOVEIT_ERROR_CODE_MAP[error_code.val]}"
        )

    def __eq__(self, other: Any) -> bool:
        """Check equality based on error code value.

        Args:
            other: Object to compare against.

        Returns:
            True if other is a PlanOnceError with the same error code value.
        """
        if isinstance(other, PlanOnceError):
            return self.error_code.val == other.error_code.val
        return False


class MaxPlanningAttemptsReachedError(PlanningError):
    """Raised when all planning retry attempts have been exhausted.

    This exception aggregates the errors from each failed planning attempt,
    providing insight into whether failures were consistent or varied.

    Attributes:
        errors: List of PlanOnceError instances from each failed attempt.
    """

    def __init__(self, errors: list[PlanOnceError]) -> None:
        """Initialize with the list of errors from each planning attempt.

        Args:
            errors: List of PlanOnceError exceptions from failed attempts.
        """
        self.errors = errors
        if all(e == errors[0] for e in errors):
            error_code_str = f"same error: {errors[0]}"
        else:
            error_code_strs = [str(e) for e in errors]
            error_code_str = f"different errors: {error_code_strs}"
        super().__init__(
            f"Max planning attempts ({len(errors)}) reached with {error_code_str}"
        )


class TrajectoryError(PlanningError):
    """Raised when trajectory post-processing fails.

    This indicates a failure during trajectory optimization (TOTG),
    smoothing, or validation after motion planning has succeeded.

    Attributes:
        error_code: The TrajectoryErrorCodes enum value indicating failure type.
    """

    def __init__(self, error_code: TrajectoryErrorCodes) -> None:
        """Initialize with the trajectory error code.

        Args:
            error_code: The TrajectoryErrorCodes enum value for the failure.
        """
        self.error_code = error_code
        super().__init__(f"Trajectory error: {error_code}")

    def __eq__(self, other: Any) -> bool:
        """Check equality based on error code.

        Args:
            other: Object to compare against.

        Returns:
            True if other is a TrajectoryError with the same error code.
        """
        if isinstance(other, TrajectoryError):
            return self.error_code == other.error_code
        return False


class ExecutionError(MoveitRecoverableError):
    """Base class for errors that occur during trajectory execution.

    This covers failures after motion planning succeeds, when the robot
    controller attempts to follow the planned trajectory.
    """


class ExecutionRejectedError(ExecutionError):
    """Raised when the robot controller rejects trajectory execution.

    This indicates the trajectory was not started, typically due to the
    robot being in protective stop, an invalid trajectory, or controller
    issues. The robot has not moved.

    Attributes:
        execution_status: The ExecutionStatus from MoveIt with rejection details.
    """

    def __init__(self, execution_status: ExecutionStatus) -> None:
        """Initialize with the execution status.

        Args:
            execution_status: The ExecutionStatus containing rejection details.
        """
        self.execution_status = execution_status
        super().__init__(f"Execution rejected: {execution_status.status}")


class ExecutionInterruptedError(ExecutionError):
    """Raised when trajectory execution is interrupted before completion.

    This indicates the robot started moving but stopped before reaching
    the goal, possibly due to an emergency stop, collision detection,
    or external intervention. The robot position is indeterminate.

    Attributes:
        execution_status: The ExecutionStatus from MoveIt with interruption details.
    """

    def __init__(self, execution_status: ExecutionStatus) -> None:
        """Initialize with the execution status.

        Args:
            execution_status: The ExecutionStatus containing interruption details.
        """
        self.execution_status = execution_status
        super().__init__(f"Execution interrupted: {execution_status.status}")


class NotSafeToExecuteError(ExecutionError):
    """Raised when execution is prevented due to safety checks.

    This exception is raised when pre-execution safety validation fails,
    such as when the robot's current state doesn't match the trajectory
    start state or when collision checks fail.

    Attributes:
        execution_status: Optional ExecutionStatus with additional context.
    """

    def __init__(
        self, execution_status: Optional[ExecutionStatus] = None
    ) -> None:
        """Initialize with optional execution status.

        Args:
            execution_status: Optional ExecutionStatus providing context for
                why execution was deemed unsafe.
        """
        self.execution_status = execution_status
        msg = "Not safe to execute"
        if execution_status is not None:
            msg += f": {execution_status.status}"
        super().__init__(msg)


class ObjectManipulationError(MoveitRecoverableError):
    """Raised when object manipulation operations fail.

    This covers failures during object fetching, presenting, or returning
    operations, such as grasp planning failures or state machine errors.
    """


class ObjectMismatchError(ObjectManipulationError):
    """Raised when the user tries to manipulate a different object than that which is currently held"""


class StateTransitionError(ObjectManipulationError):
    """Raised when an object manipulation state transition cannot be completed"""

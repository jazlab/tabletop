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
        │   ├── PlanningPipelineError
        │   ├── TrajectoryError
        │   └── MaxPlanningAttemptsReachedError
        ├── ExecutionError
        │   ├── ExecutionRejectedError
        │   ├── ExecutionInterruptedError
        │   └── NotSafeToExecuteError
        └── ObjectManipulationError
"""

from enum import Enum
from typing import Any

from moveit_msgs.msg import MoveItErrorCodes

from tabletop_rig.utils.logging import msg_to_dict

# Constants

_MOVEIT_ERROR_CODE_MAP: dict[int, str] = {
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


class ServiceClientError(Exception):
    """Raised when a ROS service client call fails.

    This indicates that the service server did not respond within the
    expected time, which may suggest the server is unavailable, overloaded,
    or has crashed.
    """


class ServiceCallTimeoutError(ServiceClientError):
    """Raised when a ROS service call exceeds its timeout duration.

    This indicates that the service server did not respond within the
    expected time, which may suggest the server is unavailable, overloaded,
    or has crashed.
    """


class ServiceCallUnsuccessfulError(ServiceClientError):
    """Raised when a ROS service call completes but returns a failure status.

    Unlike ServiceCallTimeoutError, this indicates the service responded
    but reported that the requested operation could not be completed.
    """


class ActionClientError(Exception):
    """Raised when a ROS action call fails.

    This covers action failures including rejected goals, aborted execution,
    or any other non-successful terminal state.
    """


class ActionServerWaitTimeoutError(ActionClientError):
    """Raised when waiting for a ROS action server times out."""


class ActionGoalNotAcceptedError(ActionClientError):
    """Raised when a ROS action goal request is not accepted."""


class ActionResultUnsuccessfulError(ActionClientError):
    """Raised when a ROS action get result request succeed returns an unsuccessful status.

    Attributes:
        action_name: Name of Action client.
        response: Get result response.
    """

    def __init__(self, action_name: str, response: Any):
        self.action_name = action_name
        self.response = response
        super().__init__(
            f"{action_name} action result request did not succeed "
            f"with status: {response.status}, "
            f"and result: {msg_to_dict(response.result)}"
        )


class MoveitRecoverableError(Exception):
    """Base class for MoveIt errors that may be recoverable through retry.

    Subclasses of this exception indicate failures in motion planning or
    execution that are potentially transient and may succeed if retried,
    such as planning failures due to unlucky random sampling or brief
    environmental changes.

    Attributes:
        group_name: Name of joint model group that caused the error.
    """

    def __init__(self, msg: str, group_name: str):
        """Initialize with the group name for proper handling

        Args:
            group_name: Joint model group name that caused the error.
        """
        self.group_name = group_name

        super().__init__(f"{msg} (group_name: '{group_name}')")


class PlanningError(MoveitRecoverableError):
    """Base class for errors that occur during motion planning.

    This includes failures in path planning, trajectory optimization,
    and trajectory validation stages.
    """


class PlanningPipelineError(PlanningError):
    """Raised when a single planning pipeline attempt fails.

    This exception wraps a MoveIt error code from a failed planning request
    and provides human-readable error messages.

    Attributes:
        error_code: The MoveItErrorCodes message containing the failure reason.
    """

    def __init__(
        self, error_code: MoveItErrorCodes, *, group_name: str
    ) -> None:
        """Initialize with the MoveIt error code.

        Args:
            error_code: The MoveItErrorCodes message from the failed plan.
        """
        self.error_code = error_code
        if error_code.message:
            msg = f", message: {error_code.message}"
        else:
            msg = ""

        if error_code.source:
            src = f", source: {error_code.source}"
        else:
            src = ""

        super().__init__(
            f"Planning pipeline failed with error code: "
            f"{_MOVEIT_ERROR_CODE_MAP[error_code.val]}{msg}{src}",
            group_name=group_name,
        )

    def __eq__(self, other: Any) -> bool:
        """Check equality based on error code value.

        Args:
            other: Object to compare against.

        Returns:
            True if other is a PlanningPipelineError with the same error code value.
        """
        if isinstance(other, PlanningPipelineError):
            return (self.group_name == other.group_name) and (
                self.error_code == other.error_code
            )
        return False


class TrajectoryError(PlanningError):
    """Raised when trajectory post-processing fails.

    This indicates a failure during trajectory optimization (TOTG),
    smoothing, or validation after motion planning has succeeded.

    Attributes:
        error_code: The TrajectoryErrorCodes enum value indicating failure type.
    """

    def __init__(
        self, error_code: TrajectoryErrorCodes, *, group_name: str
    ) -> None:
        """Initialize with the trajectory error code.

        Args:
            error_code: The TrajectoryErrorCodes enum value for the failure.
        """
        self.error_code = error_code
        super().__init__(
            f"Trajectory error: {error_code.name}", group_name=group_name
        )

    def __eq__(self, other: Any) -> bool:
        """Check equality based on error code.

        Args:
            other: Object to compare against.

        Returns:
            True if other is a TrajectoryError with the same error code.
        """
        if isinstance(other, TrajectoryError):
            return (self.group_name == other.group_name) and (
                self.error_code == other.error_code
            )
        return False


class MaxPlanningAttemptsReachedError(PlanningError):
    """Raised when all planning retry attempts have been exhausted.

    This exception aggregates the errors from each failed planning attempt,
    providing insight into whether failures were consistent or varied.

    Attributes:
        errors: List of PlanningPipelineError instances from each failed attempt.
    """

    def __init__(
        self, errors: list[PlanningError], *, group_name: str
    ) -> None:
        """Initialize with the list of errors from each planning attempt.

        Args:
            errors: List of PlanningPipelineError exceptions from failed attempts.
        """
        self.errors = errors
        if all(e == errors[0] for e in errors):
            error_code_str = f"same error: {errors[0]}"
        else:
            error_code_strs = [str(e) for e in errors]
            error_code_str = f"different errors: {error_code_strs}"
        super().__init__(
            f"Max planning attempts ({len(errors)}) reached with {error_code_str}",
            group_name=group_name,
        )


class ExecutionError(MoveitRecoverableError):
    """Base class for errors that occur during trajectory execution.

    This covers failures after motion planning succeeds, when the robot
    controller attempts to follow the planned trajectory.
    """


class NotSafeToExecuteError(ExecutionError):
    """Raised when execution is prevented due to safety checks.

    This exception is raised when pre-execution safety validation fails,
    such as when the robot's current state doesn't match the trajectory
    start state or when collision checks fail.
    """


class ExecutionPreventedError(ExecutionError):
    """Raised when trajectory execution is prevented by external conditions.

    This is distinct from ExecutionRejectedError (not started) and
    ExecutionInterruptedError (started but stopped). The exact conditions
    that trigger this exception are not yet defined.
    """


class ExecutionRejectedError(ExecutionError):
    """Raised when the robot controller rejects trajectory execution.

    This indicates the trajectory was not started, typically due to the
    robot being in protective stop, an invalid trajectory, or controller
    issues. The robot has not moved.
    """


class ExecutionInterruptedError(ExecutionError):
    """Raised when trajectory execution is interrupted before completion.

    This indicates the robot started moving but stopped before reaching
    the goal, possibly due to an emergency stop, collision detection,
    or external intervention. The robot position is indeterminate.
    """


class ExecutionStoppedError(ExecutionError):
    """Raised when trajectory execution stops due to recovery or abort.

    This is distinct from ExecutionInterruptedError (external interruption)
    and ExecutionRejectedError (never started). The exact triggering
    conditions are not yet defined.
    """


class ObjectManipulationError(MoveitRecoverableError):
    """Raised when object manipulation operations fail.

    This covers failures during object fetching, presenting, or returning
    operations, such as grasp planning failures or state machine errors.
    """


class ObjectMismatchError(ObjectManipulationError):
    """Raised when requested object does not match the currently held object.

    This exception is raised when an operation (e.g., return, return_if_grasped)
    is called with an object_id that differs from the one currently in the
    manipulator's grasp.
    """


class StateTransitionError(ObjectManipulationError):
    """Raised when an object manipulation state transition fails.

    This occurs when the state machine encounters an unexpected state or
    a transition that cannot be completed due to hardware or planner errors.
    """


class ManipulationContextExitedError(Exception):
    """Raised when ManipulationContextManager exits after recovery.

    This exception is raised after the manipulation context manager has
    successfully recovered and exited the async context. It signals that
    recovery was completed and the context has been cleaned up.
    """

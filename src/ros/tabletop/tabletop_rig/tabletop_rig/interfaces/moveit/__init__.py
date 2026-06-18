"""MoveIt 2 integration layer for motion planning and execution.

This package provides interfaces for robot motion planning, execution,
and object manipulation using MoveIt 2. It bridges ROS 2 callbacks with
async/await patterns and implements fuzzy trajectory caching for
efficient motion reuse.

Class Hierarchy:
    PlanAndExecuteInterface (extends BaseInterface)
        └── ObjectManipulationInterface: Pick-and-place state machine

    MoveItInterface (extends BaseInterface)
                     : Planning scene management (collision objects, ACM)

These are composed together (ObjectManipulationInterface and
PlanAndExecuteInterface receive a MoveItInterface instance).

Key Classes:
    MoveItInterface: Planning scene management (collision objects, ACM).
    PlanAndExecuteInterface: Motion planning and trajectory execution.
    ObjectManipulationInterface: Pick-and-place state machine.
    TrajectoryCache: Abstract fuzzy trajectory cache (LMDB/KD-tree
        backends).

Request Models:
    PlanRequest: Single trajectory planning request.
    ConcatPlanRequest: Multi-waypoint trajectory planning request.
    PlanGoalT: Union type for planning goals (joint, Cartesian, named).
    ObjectResetConfig: User-defined reset sequence (waypoints, collisions).
"""

from .moveit import MoveItInterface
from .object_manipulation import ObjectManipulationInterface
from .plan_and_execute import PlanAndExecuteInterface
from .requests import ConcatPlanRequest, PlanGoalT, PlanRequest

__all__ = [
    "ConcatPlanRequest",
    "MoveItInterface",
    "ObjectManipulationInterface",
    "PlanAndExecuteInterface",
    "PlanGoalT",
    "PlanRequest",
]

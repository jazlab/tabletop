"""Main MoveIt interface aggregating all motion planning capabilities.

This module provides the top-level MoveItInterface class that combines
all MoveIt-related functionality from the interface hierarchy:

Inheritance Hierarchy:
    BaseInterface
    └── PlanningSceneInterface (collision objects, planning scene)
        └── PlanAndExecuteInterface (motion planning, execution)
            └── ObjectManipulationInterface (pick/place state machine)
                └── MoveItInterface (unified entry point)

The MoveItInterface is the recommended class to instantiate for full
MoveIt functionality, providing:
- Planning scene management
- Motion planning and trajectory execution
- Object manipulation (fetch/present/return)
- Trajectory caching

Usage:
    moveit = MoveItInterface(node, safe_to_execute_callback)
    await moveit.plan_and_execute(PlanRequest(goal=target_pose))
"""

from collections.abc import Callable

from tabletop_rig.interfaces.moveit.object_manipulation import (
    ObjectManipulationInterface,
)
from tabletop_rig.nodes.base import BaseNode


class MoveItInterface(ObjectManipulationInterface):
    """Unified MoveIt interface combining all motion planning capabilities.

    This is the top-level interface class that provides access to all
    MoveIt functionality through a single instance. It inherits from
    ObjectManipulationInterface, which in turn inherits from
    PlanAndExecuteInterface and PlanningSceneInterface.

    Use this class for full MoveIt functionality including planning,
    execution, and object manipulation.

    Args:
        node: The parent ROS2 node.
        safe_to_execute_callback: Function returning True when safe to move robot.
    """

    def __init__(
        self, node: BaseNode, safe_to_execute_callback: Callable[[], bool]
    ) -> None:
        """Initialize the complete MoveIt interface.

        Args:
            node: Parent ROS2 node for creating ROS resources.
            safe_to_execute_callback: Callable returning True when external
                safety conditions allow robot motion (e.g., arms locked,
                safety laser clear).
        """
        super().__init__(
            node, safe_to_execute_callback, logger_name="moveit_interface"
        )

        self.log("MoveIt interface initialized")

from collections.abc import Callable

from tabletop_rig.interfaces.moveit.object_manipulation import (
    ObjectManipulationInterface,
)
from tabletop_rig.nodes.base import BaseNode


class MoveItInterface(ObjectManipulationInterface):
    def __init__(
        self, node: BaseNode, safe_to_execute_callback: Callable[[], bool]
    ):
        """Initializes the MoveItInterface"""
        super().__init__(
            node, safe_to_execute_callback, logger_name="moveit_interface"
        )

        self.log("MoveIt interface initialized")

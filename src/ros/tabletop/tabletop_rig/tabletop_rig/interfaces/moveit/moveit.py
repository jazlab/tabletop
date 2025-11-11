from collections.abc import (
    Callable,
)
from typing import Optional

from tabletop_rig.interfaces.moveit.object_manipulation import (
    ObjectManipulationInterface,
)
from tabletop_rig.nodes.base import BaseNode


class MoveItInterface(ObjectManipulationInterface):
    ###########################################################################
    ########## Initialization #################################################
    ###########################################################################

    def __init__(
        self,
        node: BaseNode,
        safe_to_execute_callback: Optional[Callable[[], bool]] = None,
    ):
        """Initializes the MoveItInterface"""
        super().__init__(node, "moveit_interface", safe_to_execute_callback)

        self.log("MoveIt interface initialized")

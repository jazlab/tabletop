from typing import TYPE_CHECKING

from .base import BaseNode
from .commander import Commander
from .flic import Flic
from .mock_dashboard_client import MockDashboardClient
from .mock_robot_state_helper import MockRobotStateHelper
from .mock_teensy import MockTeensy

__all__ = [
    "BaseNode",
    "Commander",
    "Eyelink",
    "Flic",
    "MockDashboardClient",
    "MockRobotStateHelper",
    "MockTeensy",
]

if TYPE_CHECKING:
    from .eyelink import Eyelink


def __getattr__(name: str):
    """Load optional heavyweight nodes only when explicitly requested."""
    if name == "Eyelink":
        from .eyelink import Eyelink

        return Eyelink
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

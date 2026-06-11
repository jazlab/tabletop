"""Node classes for the tabletop_rig package.

Node classes are imported lazily (PEP 562) so that importing one node
module does not pull in the dependencies of all the others (e.g. the
Eyelink node imports torch, which is unavailable in some containers).
"""

import importlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .base import BaseNode
    from .commander import Commander
    from .eyelink import Eyelink
    from .flic import Flic
    from .mock_dashboard_client import MockDashboardClient
    from .mock_robot_state_helper import MockRobotStateHelper
    from .mock_teensy import MockTeensy
    from .system_check import SystemCheck

_LAZY_IMPORTS = {
    "BaseNode": ".base",
    "Commander": ".commander",
    "Eyelink": ".eyelink",
    "Flic": ".flic",
    "MockDashboardClient": ".mock_dashboard_client",
    "MockRobotStateHelper": ".mock_robot_state_helper",
    "MockTeensy": ".mock_teensy",
    "SystemCheck": ".system_check",
}

__all__ = [
    "BaseNode",
    "Commander",
    "Eyelink",
    "Flic",
    "MockDashboardClient",
    "MockRobotStateHelper",
    "MockTeensy",
    "SystemCheck",
]


def __getattr__(name: str):
    if name in _LAZY_IMPORTS:
        module = importlib.import_module(_LAZY_IMPORTS[name], __package__)
        return getattr(module, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

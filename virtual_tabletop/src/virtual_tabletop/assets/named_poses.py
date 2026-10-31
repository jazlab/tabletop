"""Named arm configurations from the rig's MoveIt SRDF.

The Commander's manipulation state machine moves between named SRDF states
(``idle``, ``fetched``, ``present``, ...) per arm. They are defined as
``group_state`` elements in
``tabletop_moveit_config/srdf/dual_tabletop_macro.srdf.xacro`` with xacro
expressions such as ``${-pi/2}`` for joint values.
"""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from virtual_tabletop import paths

SRDF_MACRO = (
    paths.ROS_TABLETOP_DIR
    / "tabletop_moveit_config"
    / "srdf"
    / "dual_tabletop_macro.srdf.xacro"
)

_EXPR = re.compile(r"^\$\{(.*)\}$")


def _eval_value(value: str) -> float:
    match = _EXPR.match(value.strip())
    if not match:
        return float(value)
    # Only arithmetic on pi appears in the SRDF; refuse anything else.
    expr = match.group(1)
    if not re.fullmatch(r"[\d\s.+\-*/()pi]+", expr):
        raise ValueError(f"Unsupported SRDF expression: {value}")
    return float(eval(expr, {"__builtins__": {}}, {"pi": math.pi}))


def load_named_poses(
    srdf: Path = SRDF_MACRO,
) -> dict[str, dict[str, dict[str, float]]]:
    """Return ``{side: {state_name: {joint_name: value}}}`` from the SRDF.

    ``side`` is ``"left"`` or ``"right"`` (from the ``<side>_manipulator``
    group name); joint names keep their ``left_``/``right_`` prefix.
    """
    root = ET.fromstring(srdf.read_text())
    poses: dict[str, dict[str, dict[str, float]]] = {}
    for state in root.iter("group_state"):
        group = state.get("group")
        if not group.endswith("_manipulator"):
            continue
        side = group.removesuffix("_manipulator")
        poses.setdefault(side, {})[state.get("name")] = {
            joint.get("name"): _eval_value(joint.get("value"))
            for joint in state.iter("joint")
        }
    return poses

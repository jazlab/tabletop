"""Filesystem locations of the rig sources the virtual tabletop reads.

The virtual tabletop never imports ROS. It reads the same configuration,
xacro and mesh files the rig uses, straight out of the repository checkout.
"""

import os
from pathlib import Path

#: Root of the virtual_tabletop project (the directory holding pyproject.toml).
PROJECT_DIR = Path(__file__).resolve().parents[2]

#: Root of the TableTop repository checkout.
REPO_DIR = Path(os.environ.get("TABLETOP_DIR", PROJECT_DIR.parent)).resolve()

#: Git-ignored scratch space for downloads and generated assets.
CACHE_DIR = Path(
    os.environ.get("VTT_CACHE_DIR", PROJECT_DIR / ".cache")
).resolve()

#: Generated MuJoCo model output directory.
GENERATED_DIR = CACHE_DIR / "generated"

#: The virtual tabletop's own configuration (see documentation/configuration.md).
CONFIG_DIR = PROJECT_DIR / "config"
#: Sim-only rig details (smartglass, part colours).
RIG_CONFIG = CONFIG_DIR / "rig.yaml"
#: The monkey: arm file, shoulder placements per model, RL actuation.
SUBJECT_CONFIG = CONFIG_DIR / "subject.yaml"
#: Response buttons on the task objects (RL model).
BUTTONS_CONFIG = CONFIG_DIR / "buttons.yaml"

ROS_TABLETOP_DIR = REPO_DIR / "src" / "ros" / "tabletop"
TABLETOP_DESCRIPTION_DIR = ROS_TABLETOP_DIR / "tabletop_description"
TABLETOP_RIG_DIR = ROS_TABLETOP_DIR / "tabletop_rig"
TABLETOP_TASKS_DIR = ROS_TABLETOP_DIR / "tabletop_tasks"

COMMANDER_CONFIG = TABLETOP_RIG_DIR / "config" / "commander.yaml"
DUAL_RSP_LAUNCH = TABLETOP_DESCRIPTION_DIR / "launch" / "dual_rsp.launch.py"

#: Universal Robots description (apt package ros-jazzy-ur-description in the
#: rig image). Fetched from GitHub into the cache; see ``fetch_ur_description``.
UR_DESCRIPTION_DIR = CACHE_DIR / "ur_description"
UR_DESCRIPTION_URL = (
    "https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git"
)
UR_DESCRIPTION_REF = "7ce98d8c3512b9d0239da93b75de68ea64f61bd6"  # jazzy 3.6.0


def ros_packages() -> dict[str, Path]:
    """Map ROS package names to their source directories.

    Stands in for ``ament_index`` when resolving ``$(find pkg)`` and
    ``package://pkg`` references outside a ROS installation.
    """
    return {
        "tabletop_description": TABLETOP_DESCRIPTION_DIR,
        "tabletop_rig": TABLETOP_RIG_DIR,
        "tabletop_tasks": TABLETOP_TASKS_DIR,
        "ur_description": UR_DESCRIPTION_DIR,
    }

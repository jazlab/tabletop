"""Expand the rig's dual-UR5e xacro into a flat URDF without ROS.

The rig builds its robot description from
``tabletop_description/urdf/dual_tabletop.urdf.xacro`` with the per-arm base
poses defined in ``tabletop_description/launch/dual_rsp.launch.py`` and the
per-arm kinematic calibration in ``tabletop_description/config``. This module
reproduces that expansion with the PyPI ``xacro`` package, resolving
``$(find pkg)`` from :func:`virtual_tabletop.paths.ros_packages` instead of
the ament index.

The ``ur_ros2_control`` blocks in the rig xacro only add ``<ros2_control>``
tags (no links or joints), so the wrapper generated here includes just the
``tabletop_robot`` macro (UR arm + end-effector) for each arm.
"""

import ast
import subprocess
from dataclasses import dataclass
from pathlib import Path

from virtual_tabletop import paths

ARM_SIDES = ("left", "right")
UR_TYPE = "ur5e"

# Shared end-effector defaults from dual_tabletop.urdf.xacro.
EEF_ARGS = {
    "eef_cylinder_length": "0.0365",
    "eef_cylinder_radius": "0.0315",
    "eef_cylinder_mass": "0.5",
    "eef_sphere_radius": "0.06",
    "eef_origin_xyz": "0.0 0.0 -0.005",
    "eef_origin_rpy": "0.0 0.0 0.0",
}


@dataclass(frozen=True)
class ArmMount:
    """Where one arm's base sits in the world frame.

    Attributes:
        side: ``"left"`` or ``"right"``.
        tf_prefix: Link/joint name prefix, e.g. ``"left_"``.
        xyz: Base translation from ``world`` (meters).
        rpy: Base rotation from ``world`` (radians).
    """

    side: str
    tf_prefix: str
    xyz: str
    rpy: str


def fetch_ur_description() -> Path:
    """Clone the pinned UR description into the cache if it is missing."""
    target = paths.UR_DESCRIPTION_DIR
    if not (target / "package.xml").exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            ["git", "clone", "-q", paths.UR_DESCRIPTION_URL, str(target)],
            check=True,
        )
    head = subprocess.run(
        ["git", "-C", str(target), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if head != paths.UR_DESCRIPTION_REF:
        subprocess.run(
            ["git", "-C", str(target), "fetch", "-q", "origin", "jazzy"],
            check=True,
        )
        subprocess.run(
            [
                "git",
                "-C",
                str(target),
                "checkout",
                "-q",
                paths.UR_DESCRIPTION_REF,
            ],
            check=True,
        )
    return target


def read_arm_mounts(
    launch_file: Path = paths.DUAL_RSP_LAUNCH,
) -> list[ArmMount]:
    """Read the per-arm base poses from ``dual_rsp.launch.py``.

    The launch file keeps them in a literal ``side_defaults`` dict, which is
    parsed with :mod:`ast` (the launch file itself imports ROS).
    """
    tree = ast.parse(launch_file.read_text())
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == "side_defaults"
        ):
            side_defaults = ast.literal_eval(node.value)
            break
    else:
        raise ValueError(f"No side_defaults dict found in {launch_file}")

    return [
        ArmMount(
            side=side,
            tf_prefix=side_defaults[side]["tf_prefix"],
            xyz=side_defaults[side]["base_origin_xyz"],
            rpy=side_defaults[side]["base_origin_rpy"],
        )
        for side in ARM_SIDES
    ]


def _wrapper_xacro(mounts: list[ArmMount]) -> str:
    """Build a top-level xacro that instantiates both tabletop arms."""
    desc = paths.TABLETOP_DESCRIPTION_DIR
    ur = paths.UR_DESCRIPTION_DIR
    arms = []
    for mount in mounts:
        side = mount.side
        arms.append(f"""
  <xacro:tabletop_robot
    name="{mount.tf_prefix}tabletop"
    tf_prefix="{mount.tf_prefix}"
    parent="world"
    joint_limits_parameters_file="{ur}/config/{UR_TYPE}/joint_limits.yaml"
    kinematics_parameters_file="{desc}/config/{side}_ur5e_calibration.yaml"
    physical_parameters_file="{ur}/config/{UR_TYPE}/physical_parameters.yaml"
    visual_parameters_file="{ur}/config/{UR_TYPE}/visual_parameters.yaml"
    safety_limits="false"
    safety_pos_margin="0.15"
    safety_k_position="20"
    force_abs_paths="true"
    eef_cylinder_length="{EEF_ARGS["eef_cylinder_length"]}"
    eef_cylinder_radius="{EEF_ARGS["eef_cylinder_radius"]}"
    eef_cylinder_mass="{EEF_ARGS["eef_cylinder_mass"]}"
    eef_sphere_radius="{EEF_ARGS["eef_sphere_radius"]}">
    <origin xyz="{mount.xyz}" rpy="{mount.rpy}" />
    <origin xyz="{EEF_ARGS["eef_origin_xyz"]}"
            rpy="{EEF_ARGS["eef_origin_rpy"]}" />
  </xacro:tabletop_robot>""")
    return f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="tabletop">
  <xacro:include filename="{desc}/urdf/tabletop_macro.xacro" />
  <link name="world" />
{"".join(arms)}
</robot>
"""


def _patch_xacro_find() -> None:
    """Make xacro resolve ``$(find pkg)`` from the repository checkout."""
    from xacro import substitution_args

    packages = paths.ros_packages()

    def _eval_find(pkg: str) -> str:
        try:
            return str(packages[pkg])
        except KeyError:
            raise KeyError(
                f"ROS package '{pkg}' is not known to virtual_tabletop; add "
                "it to virtual_tabletop.paths.ros_packages()"
            ) from None

    substitution_args._eval_find = _eval_find
    substitution_args._eval_dict["find"] = _eval_find


def _resolve_package_uris(urdf: str) -> str:
    """Rewrite ``package://pkg/...`` mesh URIs to absolute file paths."""
    for pkg, pkg_dir in paths.ros_packages().items():
        urdf = urdf.replace(f"package://{pkg}/", f"{pkg_dir}/")
    return urdf.replace("file://", "")


def expand_urdf(output: Path | None = None) -> Path:
    """Expand the dual-arm description to a flat URDF file.

    Args:
        output: Destination file. Defaults to
            ``<cache>/generated/dual_tabletop.urdf``.

    Returns:
        Path of the written URDF. Mesh references are absolute paths.
    """
    import xacro

    fetch_ur_description()
    _patch_xacro_find()

    output = output or paths.GENERATED_DIR / "dual_tabletop.urdf"
    output.parent.mkdir(parents=True, exist_ok=True)
    wrapper = output.with_suffix(".wrapper.xacro")
    wrapper.write_text(_wrapper_xacro(read_arm_mounts()))

    doc = xacro.process_file(str(wrapper))
    output.write_text(_resolve_package_uris(doc.toprettyxml(indent="  ")))
    return output

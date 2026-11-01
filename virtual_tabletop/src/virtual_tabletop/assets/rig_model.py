"""The rig model (bag replay, the foraging sim, parity checks).

Generated from the rig's own sources (read from the repository checkout,
never hand-copied):

- both UR5e arms and end-effectors: ``dual_tabletop.urdf.xacro`` expanded by
  :mod:`virtual_tabletop.assets.robots` (base poses from
  ``dual_rsp.launch.py``, kinematic calibration from
  ``tabletop_description/config``);
- named arm poses (home keyframe): the MoveIt SRDF
  (:mod:`virtual_tabletop.assets.named_poses`);
- rig structure, reset stations, planning volumes and the task-object grid:
  ``commander.yaml`` (:mod:`virtual_tabletop.assets.rig_config`);
- sim-only details: ``config/rig.yaml`` (smartglass, part colours) and
  ``config/subject.yaml`` (the monkey at its ``rig`` placement).

The robots carry gravity compensation and joint position actuators, so the
model holds its pose when simulated. Task objects are mocap bodies,
positioned kinematically (seated on their mounts, or attached to an
end-effector while held). The monkey is static here (the arm file's own
actuators, unused).

Written to ``<cache>/generated/tabletop.xml`` by ``vtt-build``.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from virtual_tabletop import paths
from virtual_tabletop.assets.parts import (
    GROUP_COLLISION,
    GROUP_PLANNING,
    MeshLibrary,
    add_environment,
    add_home_key,
    add_rig,
    add_smartglass,
    add_subject,
    add_visual_meshes,
    write_model,
)
from virtual_tabletop.assets.rig_config import RigScene, load_rig_scene
from virtual_tabletop.assets.robots import ARM_SIDES, expand_urdf
from virtual_tabletop.assets.sim_config import (
    load_rig_config,
    load_subject_config,
)

RIG_MODEL = paths.GENERATED_DIR / "tabletop.xml"

#: Robot frames exposed as sites, per arm (names without the tf prefix).
#: ``tool0`` is the UR flange frame the TCP pose broadcaster reports; ``eef``
#: is the tabletop end-effector frame objects attach to.
ROBOT_SITES = ("base", "tool0", "eef")

#: Position-actuator stiffness (N m/rad) per UR joint, critically damped.
ROBOT_KP = {
    "shoulder_pan_joint": 2000.0,
    "shoulder_lift_joint": 2000.0,
    "elbow_joint": 2000.0,
    "wrist_1_joint": 500.0,
    "wrist_2_joint": 500.0,
    "wrist_3_joint": 500.0,
}


def _urdf_origin(element: ET.Element | None) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(pos, quat_wxyz)`` of a URDF ``<origin>`` element."""
    if element is None:
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    xyz = np.array([float(v) for v in element.get("xyz", "0 0 0").split()])
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    x, y, z, w = Rotation.from_euler("xyz", rpy).as_quat()
    return xyz, np.array([w, x, y, z])


def _urdf_for_mujoco(urdf: Path) -> tuple[Path, dict[str, list]]:
    """Strip visuals from the URDF and collect them separately.

    MuJoCo's URDF importer cannot load ``.dae`` files and only allows one
    color per visual, so visuals are re-added later from split meshes.

    Returns:
        The MuJoCo-ready URDF path, and per-link lists of
        ``(mesh_file, pos, quat, rgba_or_None)`` visuals.
    """
    tree = ET.parse(urdf)
    root = tree.getroot()
    visuals: dict[str, list] = {}
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise NotImplementedError(
                    f"Non-mesh visual on link {link.get('name')}"
                )
            pos, quat = _urdf_origin(visual.find("origin"))
            color = visual.find("material/color")
            rgba = (
                tuple(float(v) for v in color.get("rgba").split())
                if color is not None
                else None
            )
            visuals.setdefault(link.get("name"), []).append(
                (Path(mesh.get("filename")), pos, quat, rgba)
            )
            link.remove(visual)

    mujoco_tag = ET.SubElement(root, "mujoco")
    ET.SubElement(
        mujoco_tag,
        "compiler",
        fusestatic="false",
        discardvisual="false",
        strippath="false",
        balanceinertia="true",
    )
    out = urdf.with_suffix(".mujoco.urdf")
    tree.write(out)
    return out, visuals


def _add_grid_objects(
    spec: mujoco.MjSpec, library: MeshLibrary, scene: RigScene
):
    for obj in scene.grid_objects:
        x, y = obj.grid_idx
        body = spec.worldbody.add_body(
            name=obj.object_id,
            pos=obj.pose.position,
            quat=obj.pose.quat,
            mocap=True,
        )
        add_visual_meshes(
            body,
            library,
            obj.mesh_path,
            obj.mesh_path.resolve().stem,
            rgba=(*obj.rgba[:3], 1.0),
            conaffinity=1,  # the monkey can touch objects
        )
        body.add_site(
            name=f"mount_{x}_{y}",
            size=(0.008, 0, 0),
            rgba=(*obj.rgba[:3], 1.0),
            group=GROUP_PLANNING,
        )


def _decorate_robots(
    spec: mujoco.MjSpec, library: MeshLibrary, visuals: dict[str, list]
):
    for geom in spec.geoms:
        if geom.parent.name.startswith(ARM_SIDES):
            geom.group = GROUP_COLLISION
            geom.rgba = (0.8, 0.4, 0.2, 0.4)
            geom.contype = 0
            geom.conaffinity = 1  # the monkey can touch the robots
    for link, link_visuals in visuals.items():
        body = spec.body(link)
        for mesh_file, pos, quat, rgba in link_visuals:
            prefix = (
                f"ur5e_{mesh_file.stem}"
                if "ur_description" in str(mesh_file)
                else mesh_file.stem
            )
            add_visual_meshes(
                body, library, mesh_file, prefix, pos=pos, quat=quat, rgba=rgba
            )
    for body in spec.bodies:
        if body.name.startswith(ARM_SIDES):
            body.gravcomp = 1.0
    for side in ARM_SIDES:
        for frame in ROBOT_SITES:
            spec.body(f"{side}_{frame}").add_site(
                name=f"{side}_{frame}",
                size=(0.01, 0, 0),
                rgba=(1, 1, 0, 1),
                group=GROUP_PLANNING,
            )
        for joint, kp in ROBOT_KP.items():
            actuator = spec.add_actuator(
                name=f"{side}_{joint}",
                target=f"{side}_{joint}",
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
            )
            actuator.set_to_position(kp=kp, dampratio=1.0, inheritrange=True)
            spec.joint(f"{side}_{joint}").armature = 0.1


def build_rig_spec(
    rig_config: dict | None = None, subject_config: dict | None = None
) -> mujoco.MjSpec:
    """Assemble the rig model as an (uncompiled) ``MjSpec``.

    Args:
        rig_config: Overrides ``config/rig.yaml``.
        subject_config: Overrides ``config/subject.yaml`` (e.g. to try
            another placement).
    """
    rig_config = rig_config or load_rig_config()
    subject_config = subject_config or load_subject_config()
    urdf, visuals = _urdf_for_mujoco(expand_urdf())
    spec = mujoco.MjSpec.from_file(str(urdf))
    library = MeshLibrary(spec)
    _decorate_robots(spec, library, visuals)
    add_environment(spec)
    scene = load_rig_scene()
    add_rig(spec, library, scene, rig_config)
    add_smartglass(spec, rig_config)
    _add_grid_objects(spec, library, scene)
    add_subject(spec, subject_config, "rig")
    add_home_key(spec, spec.compile())
    return spec


def build_rig_model(
    output: Path = RIG_MODEL,
    rig_config: dict | None = None,
    subject_config: dict | None = None,
) -> Path:
    """Generate the rig model, writing MJCF to ``output``."""
    return write_model(build_rig_spec(rig_config, subject_config), output)


def load_rig_model(path: Path = RIG_MODEL) -> mujoco.MjModel:
    """Load the rig model, building it first if it doesn't exist."""
    if not path.exists():
        build_rig_model(path)
    return mujoco.MjModel.from_xml_path(str(path))

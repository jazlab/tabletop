"""Generate the MuJoCo model of the TableTop rig from the rig's own sources.

Inputs (read from the repository checkout, never hand-copied):

- Both UR5e arms and end-effectors: ``dual_tabletop.urdf.xacro`` expanded by
  :mod:`virtual_tabletop.assets.robots` (base poses from ``dual_rsp.launch.py``,
  kinematic calibration from ``tabletop_description/config``).
- Named arm poses (home keyframe): the MoveIt SRDF, via
  :mod:`virtual_tabletop.assets.named_poses`.
- Rig structure, reset stations, planning volumes and the task-object grid:
  ``commander.yaml`` via :mod:`virtual_tabletop.assets.rig_config`.
- Sim-only details no rig file records (subject placement, smartglass pane,
  part colors): ``virtual_tabletop/config/sim_rig.yaml``.

Geom groups follow the MuJoCo Menagerie convention, so the viewer's group
toggles work as expected:

- 2: visual meshes
- 3: robot collision meshes (from the URDF)
- 4: MoveIt planning volumes, frames and mount sites (hidden by default)

Contacts are limited to the monkey against the robots and task objects
(``contype``/``conaffinity`` bit 1); the robots, rig and objects never collide
with each other, since they are driven kinematically from logs or plans.

The robots carry gravity compensation and joint position actuators, so the
model holds its pose when simulated. Task objects are mocap bodies,
positioned kinematically (seated on their mounts, or attached to an
end-effector while held).

Run ``vtt-build`` (or ``python -m virtual_tabletop.assets.build``) to write
``<cache>/generated/tabletop.xml``.
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from virtual_tabletop import paths
from virtual_tabletop.assets.meshes import split_by_color
from virtual_tabletop.assets.named_poses import load_named_poses
from virtual_tabletop.assets.rig_config import RigScene, load_rig_scene
from virtual_tabletop.assets.robots import ARM_SIDES, expand_urdf

GROUP_VISUAL = 2
GROUP_COLLISION = 3
GROUP_PLANNING = 4

#: Thickness given to zero-size planning boxes (MuJoCo needs size > 0).
MIN_BOX_SIZE = 1e-3

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

#: Prefix for every element of the attached monkey arm.
SUBJECT_PREFIX = "monkey/"

#: Keyframe holding both arms at the SRDF ``idle`` pose.
HOME_KEY = "home"


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


def load_sim_config(path: Path = paths.SIM_CONFIG) -> dict:
    """Load ``config/sim_rig.yaml``."""
    return yaml.safe_load(path.read_text())


class _MeshLibrary:
    """Adds each source mesh's parts to the spec once, sharing them."""

    def __init__(self, spec: mujoco.MjSpec):
        self._spec = spec
        self._parts: dict[Path, list[tuple[str, tuple | None, str | None]]]
        self._parts = {}

    def parts(
        self, source: Path, prefix: str
    ) -> list[tuple[str, tuple | None, str | None]]:
        """Return ``(mesh_name, rgba, part_name)`` for each part."""
        source = source.resolve()
        if source not in self._parts:
            entries = []
            # CAD assemblies stay split per part: merged, the rig exceeds
            # MuJoCo's 200k-face STL limit, and parts keep their names.
            by_part = source.suffix.lower() in (".step", ".stp")
            for i, part in enumerate(split_by_color(source, by_part=by_part)):
                name = f"{prefix}_{i}"
                mesh = self._spec.add_mesh(name=name, file=str(part.path))
                # Visual meshes may be open shells; don't require a volume.
                mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
                entries.append((name, part.rgba, part.name))
            self._parts[source] = entries
        return self._parts[source]


def _add_visual_meshes(
    body: mujoco.MjsBody,
    library: _MeshLibrary,
    source: Path,
    prefix: str,
    *,
    pos=(0, 0, 0),
    quat=(1, 0, 0, 0),
    rgba: tuple | None = None,
    part_colors: dict[str, list[float]] | None = None,
    contype: int = 0,
    conaffinity: int = 0,
) -> None:
    part_colors = part_colors or {}
    for name, part_rgba, part_name in library.parts(source, prefix):
        body.add_geom(
            name=f"{body.name}_{name}_visual" if body.name else None,
            type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname=name,
            pos=pos,
            quat=quat,
            rgba=part_colors.get(part_name)
            or rgba
            or part_rgba
            or (0.7, 0.7, 0.7, 1.0),
            contype=contype,
            conaffinity=conaffinity,
            group=GROUP_VISUAL,
        )


def _add_environment(spec: mujoco.MjSpec) -> None:
    spec.modelname = "tabletop"
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080
    spec.stat.center = (0.95, 0.95, 0.9)
    spec.stat.extent = 1.6
    spec.add_texture(
        name="skybox",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=(0.3, 0.5, 0.7),
        rgb2=(0, 0, 0),
        width=512,
        height=512,
    )
    spec.add_texture(
        name="floor",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=(0.1, 0.2, 0.3),
        rgb2=(0.2, 0.3, 0.4),
        width=512,
        height=512,
    )
    floor = spec.add_material(name="floor", texrepeat=(4, 4), reflectance=0.2)
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor"
    world = spec.worldbody
    world.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=(3, 3, 0.05),
        pos=(0.95, 0.98, 0),
        material="floor",
        contype=0,
        conaffinity=0,
    )
    world.add_light(
        name="key",
        pos=(0.95, -1.0, 3.0),
        dir=(0, 0.4, -1),
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=(0.6, 0.6, 0.6),
        castshadow=0,
    )
    world.add_light(
        name="fill", pos=(0.95, 0.95, 2.5), dir=(0, 0, -1), diffuse=(0.4,) * 3
    )
    # Over the subject's shoulder, looking into the presentation window.
    world.add_camera(
        name="subject",
        pos=(0.95, -0.45, 0.95),
        xyaxes=(1, 0, 0, 0, 0.3, 1),
        fovy=55,
    )
    world.add_camera(
        name="overview",
        pos=(3.0, -1.2, 2.6),
        xyaxes=(0.6, 0.8, 0, -0.45, 0.34, 0.83),
        fovy=50,
    )


def _add_rig(
    spec: mujoco.MjSpec,
    library: _MeshLibrary,
    scene: RigScene,
    sim_config: dict,
):
    world = spec.worldbody
    for box in scene.primitives:
        world.add_geom(
            name=box.name,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[max(s, MIN_BOX_SIZE) / 2 for s in box.size],
            pos=box.pose.position,
            quat=box.pose.quat,
            rgba=box.rgba,
            contype=0,
            conaffinity=0,
            group=GROUP_PLANNING,
        )
    for rig_mesh in scene.meshes:
        body = world.add_body(
            name=rig_mesh.name,
            pos=rig_mesh.pose.position,
            quat=rig_mesh.pose.quat,
        )
        _add_visual_meshes(
            body,
            library,
            rig_mesh.path,
            rig_mesh.path.stem,
            part_colors=sim_config.get("part_colors"),
        )
        for sub, pose in rig_mesh.subframes.items():
            # Subframe rpy in commander.yaml is in world orientation but
            # expressed in the mesh frame, like MoveIt subframes.
            body.add_site(
                name=f"{rig_mesh.name}/{sub}",
                pos=pose.position,
                quat=pose.quat,
                size=(0.01, 0, 0),
                group=GROUP_PLANNING,
            )

    glass = sim_config["smartglass"]
    world.add_geom(
        name="smartglass",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[s / 2 for s in glass["size"]],
        pos=glass["center"],
        rgba=glass["occluded_rgba"],
        contype=0,
        conaffinity=0,
        group=GROUP_VISUAL,
    )


def _add_grid_objects(
    spec: mujoco.MjSpec, library: _MeshLibrary, scene: RigScene
):
    for obj in scene.grid_objects:
        x, y = obj.grid_idx
        body = spec.worldbody.add_body(
            name=obj.object_id,
            pos=obj.pose.position,
            quat=obj.pose.quat,
            mocap=True,
        )
        _add_visual_meshes(
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
    spec: mujoco.MjSpec, library: _MeshLibrary, visuals: dict[str, list]
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
            _add_visual_meshes(
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


def _add_subject(spec: mujoco.MjSpec, sim_config: dict) -> None:
    """Attach the monkey arm at its configured shoulder pose."""
    subject = sim_config["subject"]
    arm_file = paths.PROJECT_DIR / subject["arm_file"]
    arm = mujoco.MjSpec.from_file(str(arm_file))
    # Make mesh paths absolute: the written MJCF lives elsewhere.
    mesh_dir = arm_file.parent / (arm.meshdir or "")
    for mesh in arm.meshes:
        mesh.file = str((mesh_dir / mesh.file).resolve())
    x, y, z, w = Rotation.from_euler("z", subject["yaw"]).as_quat()
    frame = spec.worldbody.add_frame(
        pos=subject["shoulder_position"], quat=(w, x, y, z)
    )
    spec.attach(arm, prefix=SUBJECT_PREFIX, frame=frame)
    for geom in spec.geoms:
        if geom.name.startswith(SUBJECT_PREFIX):
            geom.contype = 1
            geom.conaffinity = 0


def _add_home_key(spec: mujoco.MjSpec, model: mujoco.MjModel) -> None:
    """Add a keyframe with both arms at the SRDF ``idle`` pose."""
    poses = load_named_poses()
    qpos = model.qpos0.copy()
    ctrl = np.zeros(model.nu)
    for side in ARM_SIDES:
        for joint, value in poses[side]["idle"].items():
            qpos[model.jnt_qposadr[model.joint(joint).id]] = value
            ctrl[model.actuator(joint).id] = value
    # Muscles rest at their minimum activation.
    ctrl[model.actuator_trntype == mujoco.mjtTrn.mjTRN_TENDON] = 0.002
    mocap_body = np.flatnonzero(model.body_mocapid >= 0)
    order = np.argsort(model.body_mocapid[mocap_body])
    spec.add_key(
        name=HOME_KEY,
        qpos=qpos,
        ctrl=ctrl,
        mpos=model.body_pos[mocap_body[order]].ravel(),
        mquat=model.body_quat[mocap_body[order]].ravel(),
    )


def build_spec() -> mujoco.MjSpec:
    """Assemble the full rig model as an (uncompiled) ``MjSpec``."""
    sim_config = load_sim_config()
    urdf, visuals = _urdf_for_mujoco(expand_urdf())
    spec = mujoco.MjSpec.from_file(str(urdf))
    library = _MeshLibrary(spec)
    _decorate_robots(spec, library, visuals)
    _add_environment(spec)
    scene = load_rig_scene()
    _add_rig(spec, library, scene, sim_config)
    _add_grid_objects(spec, library, scene)
    _add_subject(spec, sim_config)
    _add_home_key(spec, spec.compile())
    return spec


def build(output: Path | None = None) -> Path:
    """Generate and compile the rig model, writing MJCF to ``output``."""
    output = output or paths.GENERATED_DIR / "tabletop.xml"
    output.parent.mkdir(parents=True, exist_ok=True)
    spec = build_spec()
    spec.compile()
    output.write_text(spec.to_xml())
    return output


def load_model(path: Path | None = None) -> mujoco.MjModel:
    """Load the generated model, building it first if it doesn't exist."""
    path = path or paths.GENERATED_DIR / "tabletop.xml"
    if not path.exists():
        build(path)
    return mujoco.MjModel.from_xml_path(str(path))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("-o", "--output", type=Path, default=None)
    args = parser.parse_args()
    print(build(args.output))


if __name__ == "__main__":
    main()

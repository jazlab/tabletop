"""Pieces shared by the rig model and the RL model.

- :class:`MeshLibrary` / :func:`add_visual_meshes`: source meshes split into
  single-colour (or per-part) MuJoCo meshes, shared within a spec;
- :func:`add_environment`: floor, lights, cameras (``subject``,
  ``overview``);
- :func:`add_rig` / :func:`add_smartglass`: the rig CAD, planning volumes
  and the smartglass pane (``config/rig.yaml``);
- :func:`add_subject` / :func:`add_subject_servos`: the monkey arms at a
  placement from ``config/subject.yaml``, and the RL model's servos;
- :func:`add_home_key`: the ``home`` keyframe.

Geom groups follow the MuJoCo Menagerie convention, so the viewer's group
toggles work as expected: 2 visual meshes; 3 collision geoms (robot
collision meshes, the RL cup's hollow collision); 4 MoveIt planning volumes,
frames and mount sites (hidden by default).

Contacts are limited to the monkey (``contype``/``conaffinity`` bit 1): its
bones against the robots and task objects, and against each other. The
robots, rig and objects never collide with each other, since they are driven
kinematically. Bones sharing a joint overlap slightly and are filtered out
(parent-child pairs, plus the arm file's ``<exclude>``s).
"""

from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from virtual_tabletop import paths
from virtual_tabletop.assets import subject
from virtual_tabletop.assets.meshes import split_by_color
from virtual_tabletop.assets.named_poses import load_named_poses
from virtual_tabletop.assets.rig_config import RigScene
from virtual_tabletop.assets.robots import ARM_SIDES

GROUP_VISUAL = 2
GROUP_COLLISION = 3
GROUP_PLANNING = 4

#: Thickness given to zero-size planning boxes (MuJoCo needs size > 0).
MIN_BOX_SIZE = 1e-3

#: Keyframe holding both robot arms at the SRDF ``idle`` pose.
HOME_KEY = "home"

#: Shoulder placements in ``config/subject.yaml``: the rig model (replay,
#: foraging) and the RL model.
PLACEMENTS = ("rig", "rl")


class MeshLibrary:
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


def add_visual_meshes(
    body: mujoco.MjsBody,
    library: MeshLibrary,
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


def add_environment(spec: mujoco.MjSpec) -> None:
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


def add_rig(
    spec: mujoco.MjSpec,
    library: MeshLibrary,
    scene: RigScene,
    rig_config: dict,
    *,
    primitives: bool = True,
    meshes: set[str] | None = None,
):
    """Add the planning volumes (``primitives``) and the rig meshes.

    Args:
        meshes: Names of ``planning_scene.rig.meshes`` to include (default
            all: the frame ``rig`` and the reset stations).
    """
    world = spec.worldbody
    for box in scene.primitives if primitives else []:
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
        if meshes is not None and rig_mesh.name not in meshes:
            continue
        body = world.add_body(
            name=rig_mesh.name,
            pos=rig_mesh.pose.position,
            quat=rig_mesh.pose.quat,
        )
        add_visual_meshes(
            body,
            library,
            rig_mesh.path,
            rig_mesh.path.stem,
            part_colors=rig_config.get("part_colors"),
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


def add_smartglass(spec: mujoco.MjSpec, rig_config: dict) -> None:
    """The switchable occluder pane (``config/rig.yaml``)."""
    glass = rig_config["smartglass"]
    spec.worldbody.add_geom(
        name="smartglass",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[s / 2 for s in glass["size"]],
        pos=glass["center"],
        rgba=glass["occluded_rgba"],
        contype=0,
        conaffinity=0,
        group=GROUP_VISUAL,
    )


def add_subject(
    spec: mujoco.MjSpec, subject_config: dict, placement: str
) -> None:
    """Attach both monkey arms at a ``placements`` entry's shoulder poses.

    Args:
        placement: ``"rig"`` (replay, foraging) or ``"rl"``.
    """
    right_arm = paths.PROJECT_DIR / subject_config["arm_file"]
    for side, pose in subject_config["placements"][placement].items():
        arm_file = subject.arm_file(side, right_arm)
        arm = mujoco.MjSpec.from_file(str(arm_file))
        # Make mesh paths absolute: the written MJCF lives elsewhere.
        mesh_dir = arm_file.parent / (arm.meshdir or "")
        for mesh in arm.meshes:
            mesh.file = str((mesh_dir / mesh.file).resolve())
        x, y, z, w = Rotation.from_euler("z", pose["yaw"]).as_quat()
        frame = spec.worldbody.add_frame(
            pos=pose["shoulder_position"], quat=(w, x, y, z)
        )
        spec.attach(arm, prefix=subject.prefix(side), frame=frame)
    for geom in spec.geoms:
        if geom.name.startswith(subject.SUBJECT_PREFIX):
            geom.contype = 1
            geom.conaffinity = 1  # bones collide with each other too


def add_subject_servos(spec: mujoco.MjSpec, subject_config: dict) -> None:
    """Joint position servos for the monkey (RL model; provisional).

    Replaces the arm file's actuators with one position servo per monkey
    joint (target = joint angle, ``ctrlrange`` = the joint's range), and
    optionally compensates the monkey's gravity. Gains and limits are
    ``rl_actuation`` in ``config/subject.yaml``.
    """
    params = subject_config["rl_actuation"]
    for actuator in list(spec.actuators):
        if actuator.name.startswith(subject.SUBJECT_PREFIX):
            spec.delete(actuator)
    for joint in spec.joints:
        if not joint.name.startswith(subject.SUBJECT_PREFIX):
            continue
        actuator = spec.add_actuator(
            name=joint.name,
            target=joint.name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
        )
        actuator.set_to_position(
            kp=params["kp"], dampratio=1.0, inheritrange=True
        )
        actuator.forcelimited = True
        actuator.forcerange = [-params["max_torque"], params["max_torque"]]
    if params.get("gravity_compensation", False):
        for body in spec.bodies:
            if body.name.startswith(subject.SUBJECT_PREFIX):
                body.gravcomp = 1.0


def add_home_key(spec: mujoco.MjSpec, model: mujoco.MjModel) -> None:
    """Add a keyframe with both robot arms (if any) at the SRDF ``idle``
    pose; the monkey at 0 (arms hanging, fingers straight)."""
    poses = load_named_poses()
    qpos = model.qpos0.copy()
    ctrl = np.zeros(model.nu)
    robot_joints = {model.joint(j).name for j in range(model.njnt)}
    for side in ARM_SIDES:
        for joint, value in poses[side]["idle"].items():
            if joint not in robot_joints:
                continue  # model without robot arms
            qpos[model.jnt_qposadr[model.joint(joint).id]] = value
            ctrl[model.actuator(joint).id] = value
    mocap_body = np.flatnonzero(model.body_mocapid >= 0)
    order = np.argsort(model.body_mocapid[mocap_body])
    spec.add_key(
        name=HOME_KEY,
        qpos=qpos,
        ctrl=ctrl,
        mpos=model.body_pos[mocap_body[order]].ravel(),
        mquat=model.body_quat[mocap_body[order]].ravel(),
    )


def write_model(spec: mujoco.MjSpec, output: Path) -> Path:
    """Compile ``spec`` and write its MJCF to ``output``."""
    output.parent.mkdir(parents=True, exist_ok=True)
    spec.compile()
    output.write_text(spec.to_xml())
    return output

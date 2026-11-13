"""MuJoCo model for the play environment: right monkey arm + one object.

Built with ``MjSpec`` from the virtual tabletop's own parts (environment,
the monkey arm at the ``rl`` shoulder placement, its joint position
servos), with the left arm removed, plus:

- ``object``: one free-jointed body holding a whole *library* of shapes
  (:mod:`.shapes`). Only the active shape's geoms are collidable and
  visible; the env swaps mass and inertia when it changes shape. This keeps
  one free joint (6 dofs) regardless of the library size.
- ``platform``: a mocap pedestal (box) the free object rests on; parked out
  of the way when the object is fixed.
- ``object_weld``: a weld equality between ``object`` and the world, turned
  on for fixed-object trials (the object is "held in space").

Collision bits: arm = 1, object = 1|2, floor = 2, platform = 1|2. So the arm
touches the object, the platform and itself; the object touches the arm,
platform and floor; the arm never touches the floor.
"""

import copy

import mujoco
import numpy as np
from virtual_tabletop.assets.parts import (
    add_environment,
    add_subject,
    add_subject_servos,
)
from virtual_tabletop.assets.sim_config import load_subject_config

from .hand import HandSpec, apply_ranges, check_ranges, get_hand
from .shapes import Shape, make_library, make_library_v1

SIDE = "right"
PREFIX = "monkey/right/"
PARKED = np.array([1.0, 1.5, -1.0])

PLATFORM_HALF = np.array([0.04, 0.04, 0.15])  # max half-sizes (m)

#: Servo torque limits (N m): fingers, wrist, shoulder/elbow/pronation.
FINGER_TORQUE = 0.4
WRIST_TORQUE = 1.5
ARM_TORQUE = 5.0


def _right_arm_config():
    cfg = copy.deepcopy(load_subject_config())
    for placement in cfg["placements"].values():
        placement.pop("left", None)
    return cfg


def _lookat(pos, target, up=(0, 0, 1)):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f /= np.linalg.norm(f)
    x = np.cross(f, up)
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    return (*x, *y)


WORK_CENTRE = (1.03, 0.33, 0.73)


def _add_cameras(spec):
    w = spec.worldbody
    c = WORK_CENTRE
    for name, pos, fovy in [
        ("persp", (1.42, 0.72, 1.02), 50),
        ("side", (1.50, 0.30, 0.82), 50),
        ("front", (1.03, 0.88, 0.90), 50),
        ("top", (1.03, 0.37, 1.30), 45),
        ("subject_view", (0.88, -0.05, 1.05), 50),
    ]:
        up = (0, 1, 0) if name == "top" else (0, 0, 1)
        w.add_camera(name=name, pos=pos, xyaxes=_lookat(pos, c, up)
                     if name != "top" else (1, 0, 0, 0, 1, 0), fovy=fovy)
    # Close-ups that keep the object centred (orientation tracks it).
    for name, pos in [("close", (1.30, 0.62, 0.92)),
                      ("close_side", (1.36, 0.28, 0.80))]:
        w.add_camera(name=name, pos=pos, fovy=40,
                     mode=mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY,
                     targetbody="object")


def build_spec(shapes: list[Shape], hand: HandSpec | None = None
               ) -> mujoco.MjSpec:
    cfg = _right_arm_config()
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    add_environment(spec)
    spec.modelname = "ue_play"
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 5.0
    spec.option.noslip_iterations = 0
    _add_cameras(spec)
    add_subject(spec, cfg, "rl")
    add_subject_servos(spec, cfg)
    if hand is not None:
        apply_ranges(spec, hand, PREFIX)
    # Torque limits per joint group. The PPO model uses 5 N m everywhere,
    # which lets a finger joint push ~250 N at the tip: blocks get crushed
    # into the bones and wedged. Monkey-scale limits instead.
    for act in spec.actuators:
        n = act.name.rsplit("/", 1)[-1]
        if n.startswith(("thumb", "index", "middle", "ring", "pinky")):
            limit = FINGER_TORQUE
        elif n.startswith("wrist"):
            limit = WRIST_TORQUE
        else:
            limit = ARM_TORQUE
        act.forcerange = [-limit, limit]
    for g in spec.geoms:
        if g.name == "ground":
            g.contype, g.conaffinity = 2, 2
            g.friction = (1.0, 0.005, 0.0001)
        elif g.name.startswith(PREFIX):
            g.friction = (1.2, 0.01, 0.0005)
    # Object library on one free body.
    obj = spec.worldbody.add_body(name="object", pos=PARKED)
    obj.add_freejoint(name="object_free")
    for i, shape in enumerate(shapes):
        for k, part in enumerate(shape.parts):
            mesh_name = f"{shape.name}_p{k}"
            spec.add_mesh(name=mesh_name, uservert=part.ravel().tolist())
            obj.add_geom(
                name=f"obj/{shape.name}/{k}",
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=mesh_name,
                rgba=(0.8, 0.3, 0.3, 1.0),
                contype=0,
                conaffinity=0,
                condim=4,
                friction=(1.2, 0.01, 0.0005),
                density=500.0,
                group=2,
            )
    # Platform (mocap pedestal).
    plat = spec.worldbody.add_body(name="platform", pos=PARKED + [0, 0.5, 0],
                                   mocap=True)
    plat.add_geom(name="platform", type=mujoco.mjtGeom.mjGEOM_BOX,
                  size=PLATFORM_HALF, rgba=(0.55, 0.5, 0.45, 1.0),
                  contype=3, conaffinity=3, friction=(1.0, 0.005, 0.0001),
                  group=2)
    # A thin rod that "holds" fixed objects (visual only).
    rod = spec.worldbody.add_body(name="holder", pos=PARKED + [0, 1, 0],
                                  mocap=True)
    rod.add_geom(name="holder", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                 size=(0.004, 0.2, 0), pos=(0, 0, -0.2),
                 rgba=(0.3, 0.3, 0.3, 1.0), contype=0, conaffinity=0, group=2)
    eq = spec.add_equality(name="object_weld",
                           type=mujoco.mjtEq.mjEQ_WELD,
                           objtype=mujoco.mjtObj.mjOBJ_BODY,
                           name1="world", name2="object")
    eq.active = False
    eq.solref = (0.005, 1.0)
    return spec


def shape_inertials(shapes: list[Shape]) -> list[dict]:
    """Mass properties of each shape alone (compiled separately)."""
    out = []
    for shape in shapes:
        spec = mujoco.MjSpec()
        b = spec.worldbody.add_body(name="b")
        b.add_freejoint()
        for k, part in enumerate(shape.parts):
            spec.add_mesh(name=f"m{k}", uservert=part.ravel().tolist())
            b.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"m{k}",
                       density=500.0)
        m = spec.compile()
        bid = m.body("b").id
        out.append({
            "mass": float(m.body_mass[bid]),
            "ipos": m.body_ipos[bid].copy(),
            "iquat": m.body_iquat[bid].copy(),
            "inertia": m.body_inertia[bid].copy(),
        })
    return out


class PlayModel:
    """A compiled play model plus its shape library and lookups."""

    def __init__(self, n_shapes: int = 32, shape_seed: int = 0,
                 hand: str = "v0", shapes: str = "v0"):
        self.hand = get_hand(hand)
        if shapes == "v0":
            self.shapes = make_library(n_shapes, shape_seed)
        elif shapes == "v1":
            # 12 base shapes x 3 sizes = 36 (n_shapes is ignored).
            self.shapes = make_library_v1(12, shape_seed)
        else:
            raise ValueError(f"unknown shape set {shapes!r}")
        self.inertials = shape_inertials(self.shapes)
        self.spec = build_spec(self.shapes, self.hand)
        self.model = self.spec.compile()
        check_ranges(self.model, self.hand, PREFIX)
        m = self.model
        self.object_body = m.body("object").id
        self.object_qadr = m.jnt_qposadr[m.joint("object_free").id]
        self.object_dofadr = m.jnt_dofadr[m.joint("object_free").id]
        self.platform_mocap = m.body_mocapid[m.body("platform").id]
        self.platform_geom = m.geom("platform").id
        self.holder_mocap = m.body_mocapid[m.body("holder").id]
        self.weld = m.equality("object_weld").id
        self.shape_geoms = [
            [m.geom(f"obj/{s.name}/{k}").id for k in range(len(s.parts))]
            for s in self.shapes
        ]
        self.all_object_geoms = np.array(
            [g for gs in self.shape_geoms for g in gs])
        self.floor_geom = m.geom("ground").id
        # Arm.
        self.joint_names = [
            m.joint(j).name for j in range(m.njnt)
            if m.joint(j).name.startswith(PREFIX)
        ]
        jids = [m.joint(n).id for n in self.joint_names]
        self.qadr = m.jnt_qposadr[jids]
        self.dofadr = m.jnt_dofadr[jids]
        self.jnt_range = m.jnt_range[jids].copy()
        self.actuators = np.array([m.actuator(n).id for n in self.joint_names])
        self.arm_bodies = np.array(
            [b for b in range(m.nbody) if m.body(b).name.startswith(PREFIX)])
        self.arm_geom_mask = np.zeros(m.ngeom, bool)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] in set(self.arm_bodies):
                self.arm_geom_mask[g] = True
        # Tactile patches: one per hand segment, plus forearm and upper arm.
        segs = ["palm"]
        for f in ("thumb", "index", "middle", "ring", "pinky"):
            segs += [f, f + "_d1", f + "_d2"]
        self.tactile_names = segs + ["forearm", "upperarm"]
        body_to_patch = {}
        for i, s in enumerate(segs):
            body_to_patch[m.body(PREFIX + s).id] = i
        for b in ("ulna", "radius", "radius_capsule", "hand"):
            body_to_patch[m.body(PREFIX + b).id] = len(segs)
        body_to_patch[m.body(PREFIX + "humerus").id] = len(segs) + 1
        self.geom_patch = np.full(m.ngeom, -1)
        for g in range(m.ngeom):
            self.geom_patch[g] = body_to_patch.get(m.geom_bodyid[g], -1)
        # Hand landmarks: geom centres of the palm and each distal segment.
        self.palm_geom = m.geom(PREFIX + "palm").id
        self.tip_geoms = np.array(
            [m.geom(PREFIX + f + "_d2").id
             for f in ("thumb", "index", "middle", "ring", "pinky")])
        self.fingertip_site = m.site(PREFIX + "fingertip").id
        self.shoulder = np.array(
            _right_arm_config()["placements"]["rl"]["right"]
            ["shoulder_position"])

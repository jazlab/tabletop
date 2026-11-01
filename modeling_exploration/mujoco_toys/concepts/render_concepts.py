"""Build and render candidate environments for reward-free exploration.

Each concept is a small MuJoCo scene (built with MjSpec) with a simple agent
and randomly spawned objects. This script renders one still per concept
plus a labelled contact sheet, for choosing which to develop.

    uv run --no-project --with mujoco --with pillow python render_concepts.py

Writes ``images/<concept>.png`` and ``images/contact_sheet.png``.
"""

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).parent / "images"
AGENT_RGBA = (1.0, 0.45, 0.1, 1.0)
WALL_RGBA = (0.75, 0.78, 0.82, 1.0)


# --- shared pieces ------------------------------------------------------------


def base_spec(floor_half: float = 3.0) -> mujoco.MjSpec:
    spec = mujoco.MjSpec()
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 960
    spec.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=(0.55, 0.7, 0.9),
        rgb2=(0.95, 0.97, 1.0),
        width=256,
        height=256,
    )
    spec.add_texture(
        name="grid",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=(0.86, 0.87, 0.89),
        rgb2=(0.8, 0.81, 0.84),
        width=256,
        height=256,
    )
    floor = spec.add_material(name="floor", texrepeat=(8, 8), reflectance=0.1)
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    world = spec.worldbody
    world.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=(floor_half, floor_half, 0.1),
        material="floor",
    )
    world.add_light(
        pos=(0, -1, 4), dir=(0, 0.3, -1), diffuse=(0.7,) * 3, castshadow=True
    )
    world.add_light(
        pos=(2, 2, 3),
        dir=(-0.5, -0.5, -1),
        diffuse=(0.3,) * 3,
        castshadow=False,
    )
    return spec


def pastel(rng) -> tuple:
    hue = rng.uniform(0, 1)
    rgb = np.array([np.sin(2 * np.pi * (hue + k / 3)) for k in range(3)])
    rgb = 0.55 + 0.4 * rgb
    return (*np.clip(rgb, 0.1, 1.0), 1.0)


def add_walls(spec, half: float, height: float = 0.25, thick: float = 0.05):
    w = spec.worldbody
    for x, y, sx, sy in (
        (0, half, half + thick, thick),
        (0, -half, half + thick, thick),
        (half, 0, thick, half),
        (-half, 0, thick, half),
    ):
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=(sx, sy, height),
            pos=(x, y, height),
            rgba=WALL_RGBA,
        )


def add_ball_agent(spec, pos=(0, 0), radius=0.12, rgba=AGENT_RGBA):
    """A sphere moving in the plane: slide x and y, motor on each."""
    body = spec.worldbody.add_body(name="agent", pos=(*pos, radius))
    for axis, name in (((1, 0, 0), "x"), ((0, 1, 0), "y")):
        body.add_joint(
            name=f"agent_{name}",
            type=mujoco.mjtJoint.mjJNT_SLIDE,
            axis=axis,
            damping=1.0,
        )
        spec.add_actuator(
            name=f"move_{name}",
            target=f"agent_{name}",
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            gear=(10, 0, 0, 0, 0, 0),
        )
    body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(radius, 0, 0),
        rgba=rgba,
        mass=1.0,
    )
    # Eyes, so the agent reads as an agent.
    for dy in (-0.045, 0.045):
        body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=(0.025, 0, 0),
            pos=(0.1, dy, 0.04),
            rgba=(0.1, 0.1, 0.12, 1),
            contype=0,
            conaffinity=0,
        )
    return body


def add_object(spec, rng, pos, kind=None, size=None, rgba=None, mass=None):
    kind = kind or rng.choice(["box", "sphere", "cylinder", "capsule"])
    s = size or rng.uniform(0.05, 0.11)
    body = spec.worldbody.add_body(
        pos=pos, quat=_yaw_quat(rng.uniform(0, np.pi))
    )
    body.add_freejoint()
    geom_type, size3 = {
        "box": (mujoco.mjtGeom.mjGEOM_BOX, (s, s * 0.8, s * 0.7)),
        "sphere": (mujoco.mjtGeom.mjGEOM_SPHERE, (s, 0, 0)),
        "cylinder": (mujoco.mjtGeom.mjGEOM_CYLINDER, (s, s * 0.6, 0)),
        "capsule": (mujoco.mjtGeom.mjGEOM_CAPSULE, (s * 0.5, s, 0)),
    }[kind]
    body.add_geom(
        type=geom_type,
        size=size3,
        rgba=rgba or pastel(rng),
        mass=mass or rng.uniform(0.1, 1.0),
    )
    return body


def _yaw_quat(yaw: float):
    return (np.cos(yaw / 2), 0, 0, np.sin(yaw / 2))


def scatter(rng, n, half, exclude=((0, 0),), min_gap=0.35, z=0.12):
    points = []
    while len(points) < n:
        p = rng.uniform(-half, half, 2)
        if all(
            np.linalg.norm(p - np.array(e)) > min_gap
            for e in [*exclude, *points]
        ):
            points.append(p)
    return [(*p, z) for p in points]


def add_planar_arm(
    spec, base=(0, 0, 0.08), lengths=(0.35, 0.3, 0.25), tip_radius=0.04
):
    """A 3-joint arm moving in the horizontal plane (hinges about z)."""
    parent = spec.worldbody
    parent.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=(0.07, base[2], 0),
        pos=(base[0], base[1], base[2]),
        rgba=(0.3, 0.32, 0.36, 1),
    )
    body = parent.add_body(name="link0", pos=(base[0], base[1], base[2] * 2))
    for i, length in enumerate(lengths):
        body.add_joint(
            name=f"j{i}",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=(0, 0, 1),
            damping=0.2,
        )
        spec.add_actuator(
            name=f"m{i}", target=f"j{i}", trntype=mujoco.mjtTrn.mjTRN_JOINT
        )
        body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=(0, 0, 0, length, 0, 0),
            size=(0.03, 0, 0),
            rgba=AGENT_RGBA,
        )
        if i < len(lengths) - 1:
            body = body.add_body(name=f"link{i + 1}", pos=(length, 0, 0))
    body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(tip_radius, 0, 0),
        pos=(lengths[-1], 0, 0),
        rgba=(0.95, 0.3, 0.05, 1),
    )
    return body


def add_spatial_arm(spec, base=(0, 0, 0.0), lengths=(0.45, 0.4)):
    """A 3-joint arm in 3D: base yaw, shoulder pitch, elbow pitch."""
    w = spec.worldbody
    w.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=(0.09, 0.15, 0),
        pos=(base[0], base[1], 0.15),
        rgba=(0.3, 0.32, 0.36, 1),
    )
    yaw = w.add_body(name="yaw", pos=(base[0], base[1], 0.32))
    yaw.add_joint(name="j0", type=mujoco.mjtJoint.mjJNT_HINGE, axis=(0, 0, 1))
    yaw.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE, size=(0.06, 0, 0), rgba=AGENT_RGBA
    )
    upper = yaw.add_body(name="upper", quat=_pitch_quat(-0.5))
    upper.add_joint(
        name="j1", type=mujoco.mjtJoint.mjJNT_HINGE, axis=(0, 1, 0)
    )
    upper.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        fromto=(0, 0, 0, lengths[0], 0, 0),
        size=(0.035, 0, 0),
        rgba=AGENT_RGBA,
    )
    fore = upper.add_body(
        name="fore", pos=(lengths[0], 0, 0), quat=_pitch_quat(1.3)
    )
    fore.add_joint(name="j2", type=mujoco.mjtJoint.mjJNT_HINGE, axis=(0, 1, 0))
    fore.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        fromto=(0, 0, 0, lengths[1], 0, 0),
        size=(0.03, 0, 0),
        rgba=AGENT_RGBA,
    )
    fore.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(0.05, 0, 0),
        pos=(lengths[1], 0, 0),
        rgba=(0.95, 0.3, 0.05, 1),
    )
    for j in ("j0", "j1", "j2"):
        spec.add_actuator(
            name=f"m_{j}", target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT
        )
    return fore


def _pitch_quat(pitch: float):
    return (np.cos(pitch / 2), 0, np.sin(pitch / 2), 0)


# --- the concepts --------------------------------------------------------------


@dataclass
class Concept:
    key: str
    title: str
    build: Callable
    camera: tuple  # lookat (x, y, z), distance, azimuth, elevation


def playpen(rng):
    spec = base_spec()
    add_walls(spec, 1.5)
    add_ball_agent(spec, (-0.6, -0.5))
    for p in scatter(rng, 9, 1.25, exclude=((-0.6, -0.5),)):
        add_object(spec, rng, p)
    return spec


def rain(rng):
    spec = base_spec()
    add_walls(spec, 1.5)
    add_ball_agent(spec, (0.0, -0.4))
    for p in scatter(rng, 5, 1.2, exclude=((0, -0.4),)):
        add_object(spec, rng, p)
    for _ in range(6):  # falling from the sky
        x, y = rng.uniform(-1.1, 1.1, 2)
        add_object(spec, rng, (x, y, rng.uniform(0.7, 1.8)))
    return spec


def toggle_room(rng):
    spec = base_spec()
    add_walls(spec, 1.5, height=0.35)
    add_ball_agent(spec, (0.2, -0.6))
    w = spec.worldbody
    pads = [(-0.9, -0.9), (0.9, -0.9), (-0.9, 0.6), (0.7, 0.8), (0.0, 0.2)]
    lamp_colors = [
        (1, 0.2, 0.2),
        (0.2, 0.8, 0.3),
        (0.25, 0.4, 1),
        (1, 0.85, 0.1),
        (0.8, 0.3, 0.9),
    ]
    for i, ((x, y), col) in enumerate(zip(pads, lamp_colors, strict=True)):
        lit = i in (1, 3)
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=(0.13, 0.012, 0),
            pos=(x, y, 0.012),
            rgba=(*col, 1) if lit else (0.45, 0.45, 0.5, 1),
        )
        lx = -1.2 + 0.6 * i
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=(0.08, 0, 0),
            pos=(lx, -1.33, 0.45),
            rgba=(*col, 1) if lit else (0.25, 0.25, 0.28, 1),
        )
    for p in scatter(rng, 3, 1.1, exclude=[(0.2, -0.6), *pads]):
        add_object(spec, rng, p)
    return spec


def noisy_tv(rng):
    spec = base_spec()
    add_walls(spec, 1.5, height=0.35)
    add_ball_agent(spec, (-0.5, -0.4))
    w = spec.worldbody
    n = 8
    for i in range(n):
        for j in range(n):
            col = rng.uniform(0, 1, 3)
            w.add_geom(
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=(0.045, 0.01, 0.045),
                pos=(-0.4 + 0.095 * i, -1.43, 0.12 + 0.095 * j),
                rgba=(*col, 1),
            )
    for p in scatter(rng, 6, 1.2, exclude=((-0.5, -0.4),)):
        add_object(spec, rng, p)
    return spec


def hills(rng):
    spec = base_spec()
    add_walls(spec, 1.8, height=0.3)
    w = spec.worldbody
    for _ in range(5):
        x, y = rng.uniform(-1.3, 1.3, 2)
        r = rng.uniform(0.35, 0.6)
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
            size=(r, r * rng.uniform(0.8, 1.2), rng.uniform(0.12, 0.3)),
            pos=(x, y, 0),
            rgba=(0.55, 0.72, 0.5, 1),
        )
    w.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.5, 0.25, 0.02),
        pos=(0.9, -1.2, 0.17),
        quat=_pitch_quat(-0.35),
        rgba=(0.7, 0.6, 0.45, 1),
    )
    add_ball_agent(spec, (-0.4, -1.2))
    for p in scatter(rng, 6, 1.4, exclude=((-0.4, -1.2),), z=0.5):
        add_object(spec, rng, p, kind=rng.choice(["sphere", "cylinder"]))
    return spec


def sticky_ball(rng):
    spec = base_spec()
    add_walls(spec, 1.5)
    agent = add_ball_agent(spec, (0, -0.2), rgba=(0.2, 0.75, 0.45, 1))
    spec.add_actuator(
        name="stick", target="agent", trntype=mujoco.mjtTrn.mjTRN_BODY
    )
    stick = spec.actuators[-1]
    stick.set_to_adhesion(gain=20)
    stick.ctrllimited = True
    stick.ctrlrange = (0, 1)
    # Two small objects already stuck to the agent's sides.
    for dx, dy in ((0.16, 0.05), (-0.05, 0.16)):
        b = agent.add_body(pos=(dx, dy, -0.04))
        b.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=(0.04, 0.04, 0.04),
            rgba=pastel(rng),
        )
    for p in scatter(rng, 8, 1.2, exclude=((0, -0.2),)):
        add_object(spec, rng, p, size=rng.uniform(0.04, 0.07))
    return spec


def tether(rng):
    spec = base_spec()
    add_walls(spec, 1.5)
    agent = add_ball_agent(spec, (-0.3, -0.3))
    agent.add_site(name="a_site", size=(0.01, 0, 0), pos=(0.08, 0.06, 0.08))
    toy = add_object(
        spec,
        rng,
        (0.35, 0.25, 0.12),
        kind="box",
        size=0.12,
        rgba=(0.3, 0.55, 0.9, 1),
        mass=2.0,
    )
    toy.add_site(name="t_site", size=(0.01, 0, 0), pos=(-0.1, -0.08, 0.06))
    t = spec.add_tendon(
        name="rope",
        limited=True,
        range=(0, 0.95),
        width=0.02,
        rgba=(0.95, 0.8, 0.25, 1),
    )
    t.wrap_site("a_site")
    t.wrap_site("t_site")
    for p in scatter(rng, 5, 1.2, exclude=((-0.3, -0.3), (0.35, 0.25))):
        add_object(spec, rng, p)
    return spec


def billiards(rng):
    spec = base_spec()
    w = spec.worldbody
    w.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(1.4, 0.8, 0.02),
        pos=(0, 0, 0.02),
        rgba=(0.15, 0.5, 0.3, 1),
        friction=(0.05, 0.005, 0.0001),
    )
    for x, y, sx, sy in (
        (0, 0.85, 1.45, 0.05),
        (0, -0.85, 1.45, 0.05),
        (1.45, 0, 0.05, 0.8),
        (-1.45, 0, 0.05, 0.8),
    ):
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=(sx, sy, 0.07),
            pos=(x, y, 0.07),
            rgba=(0.45, 0.3, 0.2, 1),
        )
    agent = add_ball_agent(
        spec, (-0.8, 0), radius=0.08, rgba=(0.97, 0.97, 0.97, 1)
    )
    agent.pos = (-0.8, 0, 0.12)
    k = 0
    for row in range(4):
        for i in range(row + 1):
            add_object(
                spec,
                rng,
                (0.4 + row * 0.14, (i - row / 2) * 0.165, 0.12),
                kind="sphere",
                size=0.08,
            )
            k += 1
    return spec


def two_rooms(rng):
    spec = base_spec()
    add_walls(spec, 1.6, height=0.3)
    w = spec.worldbody
    for y0, y1 in ((-1.6, -0.35), (0.35, 1.6)):
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=(0.05, (y1 - y0) / 2, 0.3),
            pos=(0, (y0 + y1) / 2, 0.3),
            rgba=WALL_RGBA,
        )
    add_ball_agent(spec, (-1.0, -0.8))
    for p in scatter(rng, 8, 1.3, exclude=((-1.0, -0.8),)):
        if p[0] < 0.25:
            p = (abs(p[0]) + 0.3, p[1], p[2])
        add_object(spec, rng, p)
    return spec


def arm_pucks(rng):
    spec = base_spec()
    w = spec.worldbody
    w.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.9, 0.9, 0.04),
        pos=(0.45, 0, 0.04),
        rgba=(0.6, 0.5, 0.4, 1),
    )
    for x, y, sx, sy in (
        (0.45, 0.92, 0.92, 0.02),
        (0.45, -0.92, 0.92, 0.02),
        (1.37, 0, 0.02, 0.9),
    ):
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=(sx, sy, 0.05),
            pos=(x, y, 0.13),
            rgba=WALL_RGBA,
        )
    add_planar_arm(spec, base=(-0.3, 0, 0.06))
    for p in scatter(
        rng, 7, 0.75, exclude=((-0.3, 0), (0.1, 0)), min_gap=0.25, z=0.12
    ):
        add_object(
            spec,
            rng,
            (p[0] + 0.5, p[1], 0.12),
            kind="cylinder",
            size=rng.uniform(0.05, 0.09),
        )
    return spec


def arm_gadgets(rng):
    spec = base_spec()
    w = spec.worldbody
    w.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(1.0, 1.0, 0.04),
        pos=(0, 0, 0.04),
        rgba=(0.6, 0.5, 0.4, 1),
    )
    add_planar_arm(spec, base=(0, 0, 0.06))
    # A drawer that slides out (slide joint).
    drawer = w.add_body(pos=(0.75, 0.3, 0.16))
    drawer.add_joint(
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=(1, 0, 0),
        range=(-0.2, 0),
        damping=2,
    )
    drawer.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.12, 0.15, 0.06),
        rgba=(0.4, 0.55, 0.85, 1),
    )
    drawer.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        fromto=(-0.14, -0.06, 0, -0.14, 0.06, 0),
        size=(0.015, 0, 0),
        rgba=(0.2, 0.2, 0.2, 1),
    )
    # A door on a hinge.
    door = w.add_body(pos=(-0.6, 0.55, 0.16))
    door.add_joint(
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=(0, 0, 1),
        range=(-1.5, 0),
        damping=1,
    )
    door.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.2, 0.015, 0.1),
        pos=(0.2, 0, 0),
        rgba=(0.85, 0.65, 0.3, 1),
    )
    # A spinning wheel.
    wheel = w.add_body(pos=(-0.2, -0.65, 0.12))
    wheel.add_joint(
        type=mujoco.mjtJoint.mjJNT_HINGE, axis=(0, 0, 1), damping=0.05
    )
    wheel.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=(0.16, 0.02, 0),
        rgba=(0.85, 0.3, 0.35, 1),
    )
    wheel.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.15, 0.015, 0.03),
        pos=(0, 0, 0.03),
        rgba=(0.95, 0.95, 0.95, 1),
    )
    # A lever.
    lever = w.add_body(pos=(0.55, -0.45, 0.12))
    lever.add_joint(
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=(0, 0, 1),
        range=(-0.8, 0.8),
        damping=0.5,
    )
    lever.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        fromto=(0, 0, 0, -0.22, 0.05, 0),
        size=(0.025, 0, 0),
        rgba=(0.5, 0.8, 0.4, 1),
    )
    lever.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=(0.04, 0.03, 0),
        rgba=(0.3, 0.3, 0.3, 1),
    )
    # A push button on a post (slides when pushed).
    post = w.add_body(pos=(0.15, 0.75, 0.16))
    post.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.08, 0.05, 0.08),
        rgba=(0.6, 0.6, 0.62, 1),
    )
    button = post.add_body(pos=(0, -0.07, 0))
    button.add_joint(
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=(0, 1, 0),
        range=(0, 0.02),
        stiffness=50,
    )
    button.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=(0.035, 0.02, 0),
        quat=(0.7071, 0.7071, 0, 0),
        rgba=(0.9, 0.15, 0.15, 1),
    )
    return spec


def arm_toybox(rng):
    spec = base_spec()
    w = spec.worldbody
    add_spatial_arm(spec, base=(0, 0))
    # An open bin of balls.
    bx, by, h = 0.55, 0.2, 0.12
    for x, y, sx, sy in (
        (bx, by + 0.22, 0.22, 0.015),
        (bx, by - 0.22, 0.22, 0.015),
        (bx + 0.22, by, 0.015, 0.22),
        (bx - 0.22, by, 0.015, 0.22),
    ):
        w.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=(sx, sy, h),
            pos=(x, y, h),
            rgba=(0.45, 0.6, 0.85, 1),
        )
    for _ in range(7):
        x, y = rng.uniform(-0.15, 0.15, 2)
        add_object(
            spec,
            rng,
            (bx + x, by + y, rng.uniform(0.06, 0.3)),
            kind="sphere",
            size=0.05,
        )
    for p in scatter(
        rng, 4, 0.9, exclude=((0, 0), (bx, by)), min_gap=0.45, z=0.08
    ):
        add_object(spec, rng, p, size=0.06)
    for _ in range(2):  # falling in
        add_object(spec, rng, (rng.uniform(-0.5, 0.5), -0.5, 1.2), size=0.06)
    return spec


def arm_stack(rng):
    spec = base_spec()
    add_spatial_arm(spec, base=(0, 0))
    colors = [
        (0.9, 0.3, 0.3, 1),
        (0.3, 0.7, 0.4, 1),
        (0.3, 0.45, 0.9, 1),
        (0.95, 0.8, 0.2, 1),
    ]
    for i, col in enumerate(colors):  # a small tower
        add_object(
            spec,
            rng,
            (0.5, -0.3, 0.05 + 0.1 * i),
            kind="box",
            size=0.05,
            rgba=col,
        )
    for p in scatter(
        rng, 5, 0.8, exclude=((0, 0), (0.5, -0.3)), min_gap=0.4, z=0.06
    ):
        add_object(spec, rng, p, kind="box", size=0.05)
    return spec


CONCEPTS = [
    Concept(
        "01_playpen",
        "1. Playpen: ball + random objects",
        playpen,
        ((0, 0, 0), 3.3, -60, -50),
    ),
    Concept(
        "02_rain",
        "2. Object rain: objects spawn from the sky",
        rain,
        ((0, 0, 0.5), 3.8, -60, -25),
    ),
    Concept(
        "03_toggle_room",
        "3. Toggle room: floor pads switch lamps",
        toggle_room,
        ((0, -0.2, 0.25), 3.5, -90, -35),
    ),
    Concept(
        "04_noisy_tv",
        "4. Noisy TV: playpen + random-colour screen",
        noisy_tv,
        ((0, -0.2, 0.3), 3.4, -90, -28),
    ),
    Concept(
        "05_hills",
        "5. Hills and ramp: terrain, rolling objects",
        hills,
        ((0, 0, 0), 3.9, -60, -40),
    ),
    Concept(
        "06_sticky_ball",
        "6. Sticky ball: adhesion picks things up",
        sticky_ball,
        ((0, -0.1, 0), 2.2, -60, -40),
    ),
    Concept(
        "07_tether",
        "7. Tethered toy: a rope to a heavy box",
        tether,
        ((0, 0, 0), 2.6, -60, -50),
    ),
    Concept(
        "08_billiards",
        "8. Billiards: low-friction table, many balls",
        billiards,
        ((0, 0, 0), 3.0, -90, -50),
    ),
    Concept(
        "09_two_rooms",
        "9. Two rooms: objects behind a doorway",
        two_rooms,
        ((0, 0, 0), 3.7, -90, -60),
    ),
    Concept(
        "10_arm_pucks",
        "10. 3-joint planar arm + pucks on a table",
        arm_pucks,
        ((0.4, 0, 0), 2.6, -90, -55),
    ),
    Concept(
        "11_arm_gadgets",
        "11. Planar arm + gadgets (drawer, door, wheel, lever, button)",
        arm_gadgets,
        ((0, 0, 0.1), 2.8, -70, -50),
    ),
    Concept(
        "12_arm_toybox",
        "12. 3D 3-joint arm + bin of balls",
        arm_toybox,
        ((0.3, 0, 0.3), 2.4, -50, -25),
    ),
    Concept(
        "13_arm_blocks",
        "13. 3D 3-joint arm + blocks to topple/stack",
        arm_stack,
        ((0.3, 0, 0.2), 2.2, -40, -25),
    ),
]


def render(concept: Concept, seed: int = 0) -> Path:
    rng = np.random.default_rng(seed)
    model = concept.build(rng).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cam = mujoco.MjvCamera()
    lookat, distance, azimuth, elevation = concept.camera
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    with mujoco.Renderer(model, 600, 800) as renderer:
        renderer.update_scene(data, cam)
        image = Image.fromarray(renderer.render())
    path = OUT / f"{concept.key}.png"
    image.save(path)
    return path


def contact_sheet(paths: list[Path], titles: list[str], cols: int = 4):
    tiles = [Image.open(p).resize((480, 360)) for p in paths]
    rows = (len(tiles) + cols - 1) // cols
    band = 34
    sheet = Image.new("RGB", (cols * 480, rows * (360 + band)), "white")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.load_default(size=17)
    for i, (tile, title) in enumerate(zip(tiles, titles, strict=True)):
        x, y = (i % cols) * 480, (i // cols) * (360 + band)
        sheet.paste(tile, (x, y + band))
        draw.text((x + 8, y + 8), title, fill=(20, 20, 20), font=font)
    path = OUT / "contact_sheet.png"
    sheet.save(path)
    return path


def main():
    OUT.mkdir(exist_ok=True)
    paths = [render(c) for c in CONCEPTS]
    print(contact_sheet(paths, [c.title for c in CONCEPTS]))


if __name__ == "__main__":
    main()

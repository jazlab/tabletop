"""Scene pieces shared by the toy environments: floor, lights, walls.

Collision groups (``contype``/``conaffinity`` bits) are set so that a
gliding agent can skip the floor but still hit everything else:

- bit 1 (``STATIC``): floor, walls, pedestals;
- bit 2 (``AGENT``): the agent.

Objects collide with both (they rest on the floor and are pushed by the
agent); walls collide with both; the floor collides with objects only.
"""

import mujoco
import numpy as np

STATIC, AGENT = 1, 2

AGENT_RGBA = (1.0, 0.45, 0.1, 1.0)
WALL_RGBA = (0.75, 0.78, 0.82, 1.0)


def base_spec(floor_half: float = 3.0, name: str = "toy") -> mujoco.MjSpec:
    """An empty world: floor (collides with objects), sky and lights."""
    spec = mujoco.MjSpec()
    spec.modelname = name
    spec.compiler.degree = False  # angles (joint ranges) in radians
    spec.option.timestep = 0.004
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 960
    spec.visual.headlight.diffuse = (0.1, 0.1, 0.1)  # no glare from above
    spec.visual.headlight.ambient = (0.25, 0.25, 0.25)
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
    floor = spec.add_material(
        name="floor",
        texrepeat=(8, 8),
        reflectance=0.0,
        specular=0.1,
        shininess=0.1,
    )
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    world = spec.worldbody
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=(floor_half, floor_half, 0.1),
        material="floor",
        # MuJoCo uses the larger of two geoms' friction: keep the floor at
        # the objects' 0.5, or the floor's default 1.0 pins heavy objects.
        friction=(0.5, 0.005, 0.0001),
        contype=STATIC,
        conaffinity=STATIC,
    )
    # A directional key light (no hotspot) and a soft fill: even lighting,
    # so pixel observations aren't washed out from any camera.
    world.add_light(
        pos=(0, -1, 4),
        dir=(0.2, 0.3, -1),
        diffuse=(0.45,) * 3,
        specular=(0.05,) * 3,
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        castshadow=True,
    )
    world.add_light(
        pos=(2, 2, 3),
        dir=(-0.5, -0.5, -1),
        diffuse=(0.15,) * 3,
        specular=(0.0,) * 3,
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        castshadow=False,
    )
    return spec


def add_walls(
    spec: mujoco.MjSpec,
    half: float,
    height: float = 0.25,
    thick: float = 0.05,
) -> None:
    """Four walls enclosing the square ``[-half, half]²``."""
    for x, y, sx, sy in (
        (0, half + thick, half + 2 * thick, thick),
        (0, -half - thick, half + 2 * thick, thick),
        (half + thick, 0, thick, half),
        (-half - thick, 0, thick, half),
    ):
        spec.worldbody.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=(sx, sy, height),
            pos=(x, y, height),
            rgba=WALL_RGBA,
            contype=STATIC,
            conaffinity=STATIC | AGENT,
        )


def pastel(rng: np.random.Generator) -> tuple:
    """A random saturated pastel colour."""
    hue = rng.uniform(0, 1)
    rgb = 0.55 + 0.4 * np.sin(2 * np.pi * (hue + np.arange(3) / 3))
    return (*np.clip(rgb, 0.1, 1.0), 1.0)


def yaw_quat(yaw: float) -> tuple:
    return (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))

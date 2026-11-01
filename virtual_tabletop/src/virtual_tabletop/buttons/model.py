"""Add buttons (and hollow collision shapes) to the RL model's stimuli.

Each stimulus body ``stim/<geometry>`` gets:

- a button: a cylinder geom ``stim/<geometry>/button`` (collides with the
  monkey) and a site of the same name at its top centre;
- for geometries listed under ``hollow``: collision primitives
  ``stim/<geometry>/hollow_*`` (geom group 3) replacing the mesh's convex
  hull, which would otherwise fill the cup and hide the button. The mesh
  stays visible but stops colliding.
"""

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from virtual_tabletop.buttons.config import (
    Button,
    Hollow,
    load_buttons,
    load_hollow,
)

COLLISION_GROUP = 3


def button_name(geometry: str) -> str:
    return f"stim/{geometry}/button"


def _quat_z_to(normal: np.ndarray) -> list[float]:
    """Quaternion (w, x, y, z) rotating +z onto ``normal``."""
    rotation, _ = Rotation.align_vectors([normal], [[0.0, 0.0, 1.0]])
    x, y, z, w = rotation.as_quat()
    return [w, x, y, z]


def _add_button(body: mujoco.MjsBody, button: Button) -> None:
    name = button_name(button.geometry)
    body.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[button.radius, button.height / 2, 0],
        pos=button.center,
        quat=_quat_z_to(button.normal),
        rgba=button.rgba,
        contype=0,
        conaffinity=1,  # the monkey can press it
        group=2,
    )
    body.add_site(
        name=name,
        pos=button.position + button.normal * button.height,
        size=[0.003, 0, 0],
        group=4,
    )


def _add_hollow(body: mujoco.MjsBody, hollow: Hollow) -> None:
    """Swap the mesh's collision for a base cylinder plus a wall ring."""
    for geom in body.geoms:
        geom.contype = 0
        geom.conaffinity = 0
    name = f"stim/{hollow.geometry}/hollow"
    z0, z1 = hollow.base_z
    body.add_geom(
        name=f"{name}_base",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[hollow.base_radius, (z1 - z0) / 2, 0],
        pos=[0, 0, (z0 + z1) / 2],
        contype=0,
        conaffinity=1,
        group=COLLISION_GROUP,
    )
    z0, z1 = hollow.wall_z
    mid = (hollow.wall_inner + hollow.wall_outer) / 2
    thickness = hollow.wall_outer - hollow.wall_inner
    # Box half-width covering the outer arc of one segment.
    half_width = hollow.wall_outer * np.tan(np.pi / hollow.segments)
    for k in range(hollow.segments):
        angle = 2 * np.pi * k / hollow.segments
        body.add_geom(
            name=f"{name}_wall{k}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[thickness / 2, half_width, (z1 - z0) / 2],
            pos=[mid * np.cos(angle), mid * np.sin(angle), (z0 + z1) / 2],
            quat=[np.cos(angle / 2), 0, 0, np.sin(angle / 2)],
            contype=0,
            conaffinity=1,
            group=COLLISION_GROUP,
        )


def add_buttons(spec: mujoco.MjSpec, geometries) -> None:
    """Add buttons (and hollow collision) to ``stim/<geometry>`` bodies."""
    buttons, hollow = load_buttons(), load_hollow()
    for geometry in geometries:
        body = spec.body(f"stim/{geometry}")
        if geometry in hollow:
            _add_hollow(body, hollow[geometry])
        if geometry in buttons:
            _add_button(body, buttons[geometry])

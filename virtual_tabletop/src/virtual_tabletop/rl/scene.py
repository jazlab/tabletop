"""The RL model's presentation scene: the presented object, buttons, contacts.

One place that knows the RL model's layout (stimulus bodies and their
geoms, buttons, the monkey's arms), shared by the RL env, the scripted
policies and the design tools, so none of them reaches into another's
internals.
"""

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from virtual_tabletop.assets import subject
from virtual_tabletop.assets.rig_config import Pose, load_rig_scene
from virtual_tabletop.assets.rl_model import RL_PARKED, RL_STIMULI
from virtual_tabletop.buttons.config import load_buttons
from virtual_tabletop.buttons.model import button_name
from virtual_tabletop.task.config import load_foraging_config


@dataclass(frozen=True)
class Stimulus:
    """One stimulus body of the RL model (an object geometry).

    Attributes:
        geometry: Object mesh stem, e.g. ``small_object``.
        body: Body id (``stim/<geometry>``); mocap: its mocap index.
        meshes: Visual mesh geoms (recoloured per presented object).
        contact: Every geom the monkey can touch (a touch), incl. the button.
        button_geom, button_site: The button's geom and top-centre site.
        button_normal: The button's outward normal, object frame.
    """

    geometry: str
    body: int
    mocap: int
    meshes: tuple[int, ...]
    contact: frozenset[int]
    button_geom: int
    button_site: int
    button_normal: np.ndarray


class PresentationScene:
    """The RL model's object presentation, on a model and its data."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model, self.data = model, data
        m = model
        self.sides = subject.sides_in(m)
        self.arm_bodies = {s: subject.body_ids(m, s) for s in self.sides}
        normals = load_buttons()
        self.stimuli = {}
        for stem in RL_STIMULI:
            body = m.body(f"stim/{stem}").id
            geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == body]
            self.stimuli[stem] = Stimulus(
                geometry=stem,
                body=body,
                mocap=int(m.body_mocapid[body]),
                meshes=tuple(
                    g
                    for g in geoms
                    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                ),
                contact=frozenset(g for g in geoms if m.geom_conaffinity[g]),
                button_geom=m.geom(button_name(stem)).id,
                button_site=m.site(button_name(stem)).id,
                button_normal=normals[stem].normal,
            )
        self.current: Stimulus | None = None

    # --- placing the object -----------------------------------------------

    def show(self, geometry: str, pose: Pose, rgba=None) -> Stimulus:
        """Present ``geometry`` at ``pose`` (others parked out of sight)."""
        d = self.data
        for stim in self.stimuli.values():
            if stim.geometry == geometry:
                d.mocap_pos[stim.mocap] = pose.position
                d.mocap_quat[stim.mocap] = pose.quat
            else:
                d.mocap_pos[stim.mocap] = RL_PARKED
        self.current = self.stimuli[geometry]
        if rgba is not None:
            for g in self.current.meshes:
                self.model.geom_rgba[g] = (*rgba[:3], 1.0)
        return self.current

    # --- where things are (world frame; call after kinematics) --------------

    @property
    def object_position(self) -> np.ndarray:
        """The presented object's origin (centre of its base)."""
        return self.data.xpos[self.current.body].copy()

    @property
    def object_center(self) -> np.ndarray:
        """Centre of the presented object's mesh (its bounding volume)."""
        return self.data.geom_xpos[list(self.current.meshes)].mean(axis=0)

    @property
    def button_position(self) -> np.ndarray:
        """Top centre of the presented object's button."""
        return self.data.site_xpos[self.current.button_site].copy()

    @property
    def button_normal(self) -> np.ndarray:
        """Outward normal of the presented object's button."""
        rotation = self.data.xmat[self.current.body].reshape(3, 3)
        return rotation @ self.current.button_normal

    def hand_position(self, side: str) -> np.ndarray:
        """``side``'s fingertip."""
        site = self.model.site(subject.hand_site(side)).id
        return self.data.site_xpos[site].copy()

    # --- contacts (call after collision detection) -------------------------

    def contacting_sides(self, geoms, sides) -> list[str]:
        """Which of ``sides``' arms touch any of ``geoms``."""
        m, d = self.model, self.data
        found = []
        for i in range(d.ncon):
            g1, g2 = d.contact[i].geom1, d.contact[i].geom2
            if g1 in geoms:
                other = m.geom_bodyid[g2]
            elif g2 in geoms:
                other = m.geom_bodyid[g1]
            else:
                continue
            for side in sides:
                if other in self.arm_bodies[side] and side not in found:
                    found.append(side)
        return found

    def touching_sides(self, sides) -> list[str]:
        """Arms (of ``sides``) touching the presented object anywhere."""
        return self.contacting_sides(self.current.contact, sides)

    def pressing_sides(self, sides) -> list[str]:
        """Arms (of ``sides``) touching the presented object's button."""
        return self.contacting_sides({self.current.button_geom}, sides)

    def arm_overlap(self, side: str) -> float:
        """Deepest overlap of ``side``'s arm with the presented object,
        not counting its button (m; 0 if none)."""
        m, d = self.model, self.data
        others = self.current.contact - {self.current.button_geom}
        deepest = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in others:
                arm = m.geom_bodyid[c.geom2]
            elif c.geom2 in others:
                arm = m.geom_bodyid[c.geom1]
            else:
                continue
            if arm in self.arm_bodies[side]:
                deepest = max(deepest, -c.dist)
        return deepest


def object_geometry(object_id: str) -> str:
    """The mesh stem (stimulus geometry) of a grid object."""
    return grid_objects()[object_id].mesh_path.resolve().stem


def grid_objects() -> dict:
    """The rig's grid objects by ``object_id`` (``commander.yaml``)."""
    return {o.object_id: o for o in load_rig_scene().grid_objects}


def presentation_poses(task: str | Path = "foraging_ordered") -> list[Pose]:
    """Every pose ``task`` presents objects at (its generator's ``poses``).

    ``foraging_ordered`` has three: centred, and 5 cm either side with the
    object tilted.
    """
    config = load_foraging_config(task, available=set(grid_objects()))
    return [Pose.from_config(p) for p in config.generator_kwargs["poses"]]


def presentation_pose(task: str | Path = "foraging_ordered") -> Pose:
    """The task's first presentation pose (the one the rig starts with).

    Design tools that show "the" presented object use it.
    """
    return presentation_poses(task)[0]


def pose_key(pose: Pose) -> str:
    """A short stable label for a pose (to key per-pose data)."""
    return ",".join(f"{v:.3f}" for v in (*pose.position, *pose.quat))

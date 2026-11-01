"""Playpen: a ball agent in a walled arena of randomly spawning objects.

The agent is a ball that rolls on the floor under gravity: the action is
a horizontal force on its centre in [-1, 1]² (× ``max_force`` N), with
linear drag limiting its speed to about ``max_force / drag`` m/s. It can
move up and down (riding over small objects) and spins as it rolls (two
dark spots show the rotation). It collides with the walls and the objects.
Objects are boxes, spheres, cylinders and capsules (lying on their side, so
they roll) of random size, mass and colour (``object_kinds``,
``object_scale``, ``object_mass``).

Agent state (6): position x, y, z; velocity x, y, z.

Cameras: ``overview`` (angled, the whole arena), ``top`` (straight down),
``agent`` (follows the agent).
"""

import mujoco
import numpy as np

from mujoco_toys.envs.base import ToyEnv
from mujoco_toys.objects import SHAPES, random_shape
from mujoco_toys.scene import AGENT, AGENT_RGBA, STATIC, add_walls, base_spec


class PlaypenEnv(ToyEnv):
    """See the module docstring; also :class:`~mujoco_toys.envs.base.ToyEnv`.

    Args:
        arena_half: Half the arena's side (m).
        agent_radius: The ball's radius (m).
        max_force: Push force at action 1 (N).
        drag: Linear drag (N per m/s).
        object_kinds: Object shapes to draw from (``box``, ``sphere``,
            ``cylinder``, ``capsule``).
        object_scale: Range of an object's main half-size (m).
        object_mass: Range of an object's mass (kg).
        **kwargs: ``ToyEnv`` options (objects, spawning, observations…).
    """

    def __init__(
        self,
        *,
        arena_half: float = 1.5,
        agent_radius: float = 0.12,
        max_force: float = 10.0,
        drag: float = 8.0,
        object_kinds: tuple[str, ...] = SHAPES,
        object_scale: tuple[float, float] = (0.06, 0.13),
        object_mass: tuple[float, float] = (0.1, 1.0),
        **kwargs,
    ):
        self.arena_half, self.agent_radius = arena_half, agent_radius
        self.max_force, self.drag = max_force, drag
        self.object_kinds = object_kinds
        self.object_scale, self.object_mass = object_scale, object_mass
        super().__init__(**kwargs)

    def _random_shapes(self, rng):
        return [
            random_shape(
                rng, self.object_kinds, self.object_scale, self.object_mass
            )
            for _ in range(self.max_objects)
        ]

    def _build_spec(self, rng) -> mujoco.MjSpec:
        h = self.arena_half
        spec = base_spec(floor_half=h + 1.5, name="playpen")
        add_walls(spec, h)
        r = self.agent_radius
        # Translation (x, y, z slides) on "agent"; rolling (a ball joint) on
        # its child, so the follow camera on "agent" doesn't spin.
        agent = spec.worldbody.add_body(name="agent", pos=(0, 0, r))
        agent.explicitinertial = True  # massless frame; the ball's mass is
        agent.mass, agent.inertia = 1e-3, (1e-6, 1e-6, 1e-6)  # on its child
        for axis, name in (
            ((1, 0, 0), "x"),
            ((0, 1, 0), "y"),
            ((0, 0, 1), "z"),
        ):
            agent.add_joint(
                name=f"agent_{name}",
                type=mujoco.mjtJoint.mjJNT_SLIDE,
                axis=axis,
                damping=self.drag if name != "z" else 0.0,
            )
            if name == "z":
                continue
            spec.add_actuator(
                name=f"push_{name}",
                target=f"agent_{name}",
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                ctrllimited=True,
                ctrlrange=(-self.max_force, self.max_force),
            )
        ball = agent.add_body(name="agent_ball")
        ball.add_joint(
            name="agent_roll", type=mujoco.mjtJoint.mjJNT_BALL, damping=0.002
        )
        ball.add_geom(
            name="agent",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=(r, 0, 0),
            rgba=AGENT_RGBA,
            mass=1.0,
            # 0.5 like everything else: with 1.0 the spinning ball presses
            # objects into the floor instead of pushing them.
            friction=(0.5, 0.005, 0.0001),
            contype=AGENT,
            conaffinity=STATIC,
        )
        for direction in ((1, 0, 0.3), (-0.3, 0.9, -0.3)):  # spots
            d = np.array(direction) / np.linalg.norm(direction)
            ball.add_geom(
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=(0.28 * r, 0, 0),
                pos=0.8 * r * d,
                rgba=(0.12, 0.1, 0.1, 1),
                contype=0,
                conaffinity=0,
                mass=0.0,
            )
        agent.add_camera(
            name="agent", pos=(-0.9, 0, 0.7), xyaxes=(0, -1, 0, 0.6, 0, 0.8)
        )
        spec.worldbody.add_camera(
            name="overview",
            pos=(0, -2.9, 2.7),
            xyaxes=(1, 0, 0, 0, 0.68, 0.73),
        )
        spec.worldbody.add_camera(
            name="top", pos=(0, 0, 4.4), xyaxes=(1, 0, 0, 0, 1, 0)
        )
        return spec

    def _reset_agent(self, rng) -> None:
        h = self.arena_half - self.agent_radius - 0.05
        for name in ("agent_x", "agent_y"):
            self.data.qpos[self.model.joint(name).qposadr[0]] = rng.uniform(
                -h, h
            )

    def _spawn_spot(self, rng) -> np.ndarray:
        return rng.uniform(-self.arena_half + 0.15, self.arena_half - 0.15, 2)

    def _keep_clear(self) -> list:
        x, y = self.agent_position
        return [(x, y, self.agent_radius + 0.05)]

    def _agent_geoms(self) -> set[int]:
        return {self.model.geom("agent").id}

    @property
    def agent_height(self) -> float:
        return float(self.data.xpos[self.model.body("agent").id][2])

    @property
    def agent_position(self) -> np.ndarray:
        return self.data.xpos[self.model.body("agent").id][:2].copy()

    def _agent_state(self) -> np.ndarray:
        body = self.model.body("agent").id
        return np.concatenate([self.data.xpos[body], self.data.qvel[:3]])

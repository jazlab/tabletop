"""Randomly shaped objects that spawn during an episode.

An environment builds a fixed number of object **slots** into its model
(each a free body with one random shape, size, mass and colour, drawn per
episode). At any time a slot is *active* (in the scene) or *parked* (far
away, resting on the floor plane, out of sight and out of reach). Spawning
moves a parked slot to a free spot in the scene; when every slot is
active, the oldest is recycled. No recompiling is needed during an
episode.

Per-slot observation features (:data:`SLOT_FEATURES` numbers): active flag,
position (3), orientation quaternion (4), linear and angular velocity
(3 + 3), half-sizes (3), shape one-hot (4), mass (1).
"""

from collections.abc import Callable, Sequence
from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_toys.scene import AGENT, STATIC, pastel, yaw_quat

SHAPES = ("box", "sphere", "cylinder", "capsule")
SLOT_FEATURES = 1 + 3 + 4 + 3 + 3 + 3 + len(SHAPES) + 1

#: Where parked slots wait (each at its own spot on the floor plane).
PARK_ORIGIN = np.array([60.0, 60.0])


@dataclass(frozen=True)
class Shape:
    """One object's shape (half-sizes in metres, mass in kg)."""

    kind: str
    size: tuple[float, float, float]
    mass: float
    rgba: tuple

    @property
    def rest_height(self) -> float:
        """Height of the body origin when resting upright on the floor."""
        if self.kind == "capsule":
            return self.size[0]  # lying on its side
        return {
            "box": self.size[2],
            "sphere": self.size[0],
            "cylinder": self.size[1],
        }[self.kind]

    @property
    def radius(self) -> float:
        """Footprint radius, for keeping spawn spots apart."""
        return float(
            np.hypot(self.size[0], self.size[1])
            if self.kind == "box"
            else max(self.size[:2])
        )


def random_shape(
    rng: np.random.Generator,
    kinds: Sequence[str] = SHAPES,
    scale: tuple[float, float] = (0.06, 0.13),
    mass: tuple[float, float] = (0.1, 1.0),
) -> Shape:
    kind = str(rng.choice(kinds))
    s = rng.uniform(*scale)
    size = {
        "box": (s, s * rng.uniform(0.6, 1.0), s * rng.uniform(0.5, 1.0)),
        "sphere": (s, 0.0, 0.0),
        "cylinder": (s, s * rng.uniform(0.5, 1.0), 0.0),
        "capsule": (s * 0.5, s, 0.0),
    }[kind]
    return Shape(kind, size, float(rng.uniform(*mass)), pastel(rng))


_GEOM_TYPES = {
    "box": mujoco.mjtGeom.mjGEOM_BOX,
    "sphere": mujoco.mjtGeom.mjGEOM_SPHERE,
    "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
    "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE,
}


def add_slots(spec: mujoco.MjSpec, shapes: Sequence[Shape]) -> None:
    """Add one free body ``obj<i>`` per shape, parked."""
    for i, shape in enumerate(shapes):
        body = spec.worldbody.add_body(
            name=f"obj{i}", pos=(*_park_spot(i), shape.rest_height)
        )
        body.add_freejoint(name=f"obj{i}")
        body.add_geom(
            name=f"obj{i}",
            type=_GEOM_TYPES[shape.kind],
            size=shape.size,
            # Capsules lie on their side so they can roll.
            quat=(0.7071068, 0.7071068, 0, 0)
            if shape.kind == "capsule"
            else (1, 0, 0, 0),
            rgba=shape.rgba,
            mass=shape.mass,
            friction=(0.5, 0.005, 0.0001),
            contype=STATIC,
            conaffinity=STATIC | AGENT,
        )


def _park_spot(i: int) -> np.ndarray:
    return PARK_ORIGIN + np.array([1.0 * (i % 10), 1.0 * (i // 10)])


class ObjectPool:
    """Runtime control of the object slots of a compiled model.

    Args:
        model, data: The compiled model and its data.
        shapes: The slots' shapes (as built by :func:`add_slots`).
        sample_spot: ``rng -> (x, y)``: a candidate spawn spot.
        keep_clear: ``() -> [(x, y, radius)]``: places spawns must avoid
            besides the active objects (e.g. the agent).
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        shapes: Sequence[Shape],
        sample_spot: Callable[[np.random.Generator], np.ndarray],
        keep_clear: Callable[[], list] = list,
    ):
        self.model, self.data, self.shapes = model, data, list(shapes)
        self.sample_spot, self.keep_clear = sample_spot, keep_clear
        self.joints = [model.joint(f"obj{i}") for i in range(len(shapes))]
        self.bodies = [model.body(f"obj{i}").id for i in range(len(shapes))]
        self.geoms = [model.geom(f"obj{i}").id for i in range(len(shapes))]
        self.active = np.zeros(len(shapes), dtype=bool)
        self.spawned_at = np.full(len(shapes), -np.inf)
        self._static = self._static_features()

    def __len__(self) -> int:
        return len(self.shapes)

    # --- spawning ----------------------------------------------------------

    def spawn(
        self,
        rng: np.random.Generator,
        time: float = 0.0,
        drop_height: float = 0.0,
    ) -> int | None:
        """Activate a slot at a free spot; return its index (or None if no
        free spot was found). Recycles the oldest slot if all are active."""
        parked = np.flatnonzero(~self.active)
        i = int(parked[0]) if len(parked) else int(np.argmin(self.spawned_at))
        self.active[i] = False  # don't count it as an obstacle for itself
        spot = self._free_spot(rng, self.shapes[i].radius)
        if spot is None:
            return None
        self.place(
            i,
            (*spot, self.shapes[i].rest_height + drop_height),
            rng.uniform(0, 2 * np.pi),
        )
        self.active[i] = True
        self.spawned_at[i] = time
        return i

    def park(self, i: int) -> None:
        self.place(i, (*_park_spot(i), self.shapes[i].rest_height), 0.0)
        self.active[i] = False

    def place(self, i: int, pos, yaw: float) -> None:
        """Put slot ``i`` at ``pos`` with heading ``yaw``, at rest."""
        j = self.joints[i]
        self.data.qpos[j.qposadr[0] : j.qposadr[0] + 7] = (
            *pos,
            *yaw_quat(yaw),
        )
        self.data.qvel[j.dofadr[0] : j.dofadr[0] + 6] = 0.0

    def _free_spot(self, rng, radius: float, tries: int = 50):
        obstacles = list(self.keep_clear())
        for k in np.flatnonzero(self.active):
            p = self.data.xpos[self.bodies[k]]
            obstacles.append((p[0], p[1], self.shapes[k].radius))
        for _ in range(tries):
            spot = np.asarray(self.sample_spot(rng), dtype=float)
            if all(
                np.hypot(spot[0] - x, spot[1] - y) > radius + r + 0.02
                for x, y, r in obstacles
            ):
                return spot
        return None

    # --- observations ------------------------------------------------------

    def _static_features(self) -> np.ndarray:
        rows = []
        for shape in self.shapes:
            onehot = [float(shape.kind == k) for k in SHAPES]
            rows.append([*shape.size, *onehot, shape.mass])
        return np.array(rows, dtype=float)

    def features(self) -> np.ndarray:
        """``(len(self), SLOT_FEATURES)``; parked slots are all zeros."""
        out = np.zeros((len(self), SLOT_FEATURES))
        for i in np.flatnonzero(self.active):
            j = self.joints[i]
            q = self.data.qpos[j.qposadr[0] : j.qposadr[0] + 7]
            v = self.data.qvel[j.dofadr[0] : j.dofadr[0] + 6]
            out[i] = [1.0, *q, *v, *self._static[i]]
        return out

    def positions(self) -> np.ndarray:
        return self.data.xpos[self.bodies]

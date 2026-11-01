"""Button placements and hollow collision shapes (``config/buttons.yaml``)."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from virtual_tabletop import paths

BUTTONS_CONFIG = paths.BUTTONS_CONFIG


@dataclass(frozen=True)
class Button:
    """A button on one object geometry, in the object frame (m).

    Attributes:
        geometry: Object mesh stem, e.g. ``small_object``.
        position: Centre of the button's base, on the object surface.
        normal: Unit outward normal (the press direction is ``-normal``).
        radius: Button radius.
        height: How far the button stands off the surface.
        rgba: Colour.
    """

    geometry: str
    position: np.ndarray
    normal: np.ndarray
    radius: float
    height: float
    rgba: tuple[float, float, float, float]

    @property
    def center(self) -> np.ndarray:
        """Centre of the button cylinder (object frame)."""
        return self.position + self.normal * self.height / 2


@dataclass(frozen=True)
class Hollow:
    """Collision primitives for a cup-like geometry (object frame, m).

    A solid cylinder ``base`` up to the floor and a ring of ``segments``
    boxes forming the ``wall`` between two radii.
    """

    geometry: str
    base_radius: float
    base_z: tuple[float, float]
    wall_inner: float
    wall_outer: float
    wall_z: tuple[float, float]
    segments: int


def load_buttons(path: Path = BUTTONS_CONFIG) -> dict[str, Button]:
    raw = yaml.safe_load(path.read_text())["buttons"]
    buttons = {}
    for geometry, b in raw.items():
        normal = np.asarray(b["normal"], float)
        buttons[geometry] = Button(
            geometry,
            np.asarray(b["position"], float),
            normal / np.linalg.norm(normal),
            float(b["radius"]),
            float(b["height"]),
            tuple(b.get("rgba", (0.85, 0.1, 0.1, 1.0))),
        )
    return buttons


def load_hollow(path: Path = BUTTONS_CONFIG) -> dict[str, Hollow]:
    raw = yaml.safe_load(path.read_text()).get("hollow") or {}
    return {
        geometry: Hollow(
            geometry,
            float(h["base"]["radius"]),
            tuple(h["base"]["z"]),
            float(h["wall"]["inner_radius"]),
            float(h["wall"]["outer_radius"]),
            tuple(h["wall"]["z"]),
            int(h.get("segments", 24)),
        )
        for geometry, h in raw.items()
    }

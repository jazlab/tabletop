"""Tests for the response buttons: config, model, and reachability."""

import numpy as np
import pytest
import trimesh
from virtual_tabletop.assets.rig_config import load_rig_scene
from virtual_tabletop.assets.rl_model import RL_STIMULI, load_rl_model
from virtual_tabletop.buttons.config import load_buttons, load_hollow
from virtual_tabletop.buttons.model import button_name


def _mesh(geometry: str):
    scene = load_rig_scene()
    path = next(
        o.mesh_path
        for o in scene.grid_objects
        if o.mesh_path.resolve().stem == geometry
    )
    return trimesh.load_mesh(path)


def _surface_heights(mesh, x: float = 0.0) -> list[float]:
    """Heights where the mesh surface crosses the vertical line at
    (``x``, 0), from its y = 0 cross-section."""
    section = mesh.section(plane_origin=(0, 0, 0), plane_normal=(0, 1, 0))
    heights = []
    for entity in section.entities:
        points = section.vertices[entity.points]
        for a, b in zip(points[:-1], points[1:], strict=True):
            if (a[0] - x) * (b[0] - x) < 0:
                f = (x - a[0]) / (b[0] - a[0])
                heights.append(a[2] + f * (b[2] - a[2]))
    return heights


def test_every_stimulus_has_a_button_on_its_surface():
    model = load_rl_model()
    buttons = load_buttons()
    for geometry in RL_STIMULI:
        assert model.geom(button_name(geometry)).id >= 0
        button = buttons[geometry]
        assert button.position[:2] == pytest.approx([0, 0])  # centred
        # Its base sits on the highest surface above the centre (the
        # cup's floor, the drum's top), facing up.
        surface = max(_surface_heights(_mesh(geometry)))
        assert button.position[2] == pytest.approx(surface, abs=0.001)
        assert button.normal == pytest.approx([0, 0, 1])


def test_hollow_cup_collision_matches_the_mesh():
    """The cup's collision primitives follow its walls and floor."""
    hollow = load_hollow()["small_object"]
    mesh = _mesh("small_object")
    for z in (0.04, 0.06, 0.08):  # the upper wall
        section = mesh.section(plane_origin=(0, 0, z), plane_normal=(0, 0, 1))
        radii = sorted(
            np.linalg.norm(section.vertices[e.points][:, :2], axis=1).mean()
            for e in section.entities
        )
        assert radii[0] == pytest.approx(hollow.wall_inner, abs=0.001)
        assert radii[-1] == pytest.approx(hollow.wall_outer, abs=0.001)
    # The floor (the top of the base cylinder) is the mesh's inner floor.
    assert hollow.base_z[1] == pytest.approx(
        max(_surface_heights(mesh)), abs=0.001
    )


@pytest.mark.slow
def test_buttons_are_reachable_without_going_through_the_object():
    from virtual_tabletop.rl.press_paths import PressPlanner

    planner = PressPlanner()
    for geometry in RL_STIMULI:
        for side in ("right", "left"):
            result = planner.solve(geometry, side, starts=24)
            assert result.reachable(), (geometry, side, result)

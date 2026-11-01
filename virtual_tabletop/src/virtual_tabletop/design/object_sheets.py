"""Placement sheets: each object geometry drawn in its own frame.

Informs **button placement** (``config/buttons.yaml``).

::

    uv run vtt-object-sheets [--out DIR]

One PNG per distinct object mesh (default ``.cache/object_sheets/``), with
orthographic views in the **object frame** (the mesh's own frame, in cm:
origin at the centre of the base, +z up) and a cm grid, so a button position
can be read off directly:

- **front**: as the monkey sees it at the presentation pose;
- **side**: from the subject's right;
- **top**: from above (the monkey at the bottom);
- **section**: a vertical cut through the middle, showing walls and any
  cavity;
- a perspective render at the presentation pose, from the monkey's side.

At the presentation pose (quat ``(0.707, 0, 0, -0.707)``, a −90° yaw) the
object's **+x faces the monkey** and its **+y points to the subject's
right**.
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np
import trimesh

from virtual_tabletop import paths
from virtual_tabletop.assets.rig_config import load_rig_scene
from virtual_tabletop.assets.rl_model import load_rl_model
from virtual_tabletop.rl.scene import PresentationScene, presentation_pose

DEFAULT_OUT = paths.CACHE_DIR / "object_sheets"

#: Views: (title, horizontal axis, vertical axis, viewing direction), with
#: axes as (index, sign) in the object frame. The viewing direction points
#: from the viewer into the object.
VIEWS = (
    ("Front (monkey's view)", (1, 1), (2, 1), (-1, 0, 0)),
    ("Side (from the subject's right)", (0, -1), (2, 1), (0, -1, 0)),
    ("Top (monkey at the bottom)", (1, 1), (0, -1), (0, 0, -1)),
)
AXIS_NAMES = "xyz"


def object_meshes() -> dict[str, Path]:
    """Distinct object meshes on the grid, by stem."""
    scene = load_rig_scene()
    return {
        o.mesh_path.resolve().stem: o.mesh_path.resolve()
        for o in scene.grid_objects
    }


def _axis_label(axis: tuple[int, int]) -> str:
    index, sign = axis
    hint = {
        (0, 1): "toward the monkey",
        (0, -1): "toward the rig",
        (1, 1): "subject's right",
        (2, 1): "up",
    }[axis]
    prefix = "" if sign > 0 else "−"
    return f"{prefix}{AXIS_NAMES[index]} (cm), {hint} →"


def _draw_view(ax, mesh, horizontal, vertical, direction) -> None:
    """Shaded orthographic projection (painter's algorithm)."""
    from matplotlib.collections import PolyCollection

    v = mesh.vertices * 100  # cm
    tris = v[mesh.faces]
    normals = mesh.face_normals
    view = np.asarray(direction, float)
    facing = normals @ -view > 0  # faces toward the viewer
    depth = (tris.mean(axis=1) @ view)[facing]
    order = np.argsort(-depth)  # far first
    tris = tris[facing][order]
    shade = 0.35 + 0.65 * (normals[facing][order] @ -view)
    h, hs = horizontal
    w, ws = vertical
    polys = np.stack([hs * tris[:, :, h], ws * tris[:, :, w]], axis=-1)
    colors = np.clip(shade, 0, 1)[:, None] * np.array([0.85, 0.6, 0.35])
    ax.add_collection(
        PolyCollection(polys, facecolors=colors, edgecolors="none")
    )
    ax.set_xlim(polys[..., 0].min() - 1, polys[..., 0].max() + 1)
    ax.set_ylim(polys[..., 1].min() - 1, polys[..., 1].max() + 1)


def _style(ax, title: str, xlabel: str, ylabel: str) -> None:
    from matplotlib.ticker import MultipleLocator

    ax.set(title=title, xlabel=xlabel, ylabel=ylabel, aspect="equal")
    ax.xaxis.set_major_locator(MultipleLocator(2))
    ax.yaxis.set_major_locator(MultipleLocator(2))
    ax.xaxis.set_minor_locator(MultipleLocator(1))
    ax.yaxis.set_minor_locator(MultipleLocator(1))
    ax.grid(which="major", color="k", alpha=0.35, lw=0.6)
    ax.grid(which="minor", color="k", alpha=0.12, lw=0.4)
    ax.set_axisbelow(False)


def _draw_section(ax, mesh) -> None:
    """Vertical cut through the middle (the y = 0 plane), x vs z."""
    section = mesh.section(plane_origin=(0, 0, 0), plane_normal=(0, 1, 0))
    if section is None:
        return
    for entity in section.entities:
        points = section.vertices[entity.points] * 100
        ax.fill(-points[:, 0], points[:, 2], color=(0.85, 0.6, 0.35))
        ax.plot(-points[:, 0], points[:, 2], color="k", lw=0.8)
    ax.autoscale()


def _render_presented(stem: str, size=(480, 640)) -> np.ndarray:
    """The object at the presentation pose, seen from the monkey's side."""
    model = load_rl_model(False)
    data = mujoco.MjData(model)
    pose = presentation_pose()
    PresentationScene(model, data).show(stem, pose)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = np.asarray(pose.position) + (0, 0, 0.05)
    camera.distance, camera.azimuth, camera.elevation = 0.45, -110, -25
    with mujoco.Renderer(model, *size) as renderer:
        renderer.update_scene(data, camera=camera)
        return renderer.render()


def write_sheet(stem: str, mesh_path: Path, out: Path) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mesh = trimesh.load_mesh(mesh_path)
    fig = plt.figure(figsize=(16, 10))
    grid = fig.add_gridspec(2, 3)
    for k, (title, horizontal, vertical, direction) in enumerate(VIEWS):
        ax = fig.add_subplot(grid[k // 3, k % 3])
        _draw_view(ax, mesh, horizontal, vertical, direction)
        _style(ax, title, _axis_label(horizontal), _axis_label(vertical))
    ax = fig.add_subplot(grid[1, 0])
    _draw_section(ax, mesh)
    _style(
        ax,
        "Section through the middle (y = 0)",
        _axis_label((0, -1)),
        _axis_label((2, 1)),
    )
    ax = fig.add_subplot(grid[1, 1:])
    ax.imshow(_render_presented(stem))
    ax.set_axis_off()
    ax.set_title("At the presentation pose, from the monkey's side")
    lo, hi = mesh.bounds * 100
    fig.suptitle(
        f"{stem}: {hi[0] - lo[0]:.1f} × {hi[1] - lo[1]:.1f} × "
        f"{hi[2] - lo[2]:.1f} cm. Object frame: origin at the base centre; "
        "+x faces the monkey, +y is the subject's right, +z up",
        fontsize=13,
    )
    fig.tight_layout()
    path = out / f"{stem}.png"
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for stem, mesh_path in object_meshes().items():
        print(write_sheet(stem, mesh_path, args.out))


if __name__ == "__main__":
    main()

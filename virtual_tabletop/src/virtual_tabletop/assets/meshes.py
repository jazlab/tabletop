"""Convert rig meshes (DAE, STEP, STL) into MuJoCo-loadable STL parts.

MuJoCo reads STL/OBJ/MSH only, and assigns one color per geom. Multi-material
sources (the UR ``.dae`` visuals, the rig ``.step`` assembly) are therefore
split into one STL per color, with transforms baked in. Results are cached
under ``<cache>/meshes`` keyed by the source file's content hash.
"""

import contextlib
import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

from virtual_tabletop import paths

MESH_CACHE_DIR = paths.CACHE_DIR / "meshes"


@dataclass(frozen=True)
class MeshPart:
    """One single-color piece of a source mesh.

    Attributes:
        path: STL file, in the source mesh's own frame and units (meters).
        rgba: Color from the source material, or ``None`` if it had none.
        name: Source node name (per-part splits only), e.g. a STEP part.
    """

    path: Path
    rgba: tuple[float, float, float, float] | None
    name: str | None = None


@contextlib.contextmanager
def _quiet_stdout():
    """Silence C-level stdout (OpenCASCADE prints one line per STEP node)."""
    saved = os.dup(1)
    with open(os.devnull, "w") as devnull:
        os.dup2(devnull.fileno(), 1)
        try:
            yield
        finally:
            os.dup2(saved, 1)
            os.close(saved)


def _file_hash(path: Path) -> str:
    return hashlib.sha256(path.resolve().read_bytes()).hexdigest()[:16]


def _part_color(mesh: trimesh.Trimesh) -> tuple[float, ...] | None:
    visual = mesh.visual
    if visual is None:
        return None
    if visual.kind == "texture":
        color = getattr(visual.material, "main_color", None)
    elif visual.kind in ("face", "vertex"):
        color = visual.main_color
    else:
        return None
    if color is None:
        return None
    return tuple(round(float(c) / 255.0, 4) for c in color)


def _load_parts(path: Path) -> list[tuple[str, trimesh.Trimesh]]:
    """Load ``(node_name, mesh)`` pairs with scene transforms baked in."""
    suffix = path.suffix.lower()
    if suffix == ".stl":
        return [(path.stem, trimesh.load_mesh(path))]
    if suffix in (".step", ".stp"):
        with _quiet_stdout():
            scene = trimesh.load_scene(path)
    else:
        scene = trimesh.load_scene(path)
    parts = []
    for node in scene.graph.nodes_geometry:
        transform, geometry = scene.graph[node]
        mesh = scene.geometry[geometry]
        if len(mesh.faces):
            parts.append((str(node), mesh.copy().apply_transform(transform)))
    return parts


def split_by_color(source: Path, *, by_part: bool = False) -> list[MeshPart]:
    """Split a mesh file into single-color STL parts.

    Args:
        source: Mesh file (``.dae``, ``.step``/``.stp`` or ``.stl``).
        by_part: Keep every sub-mesh separate (named after its node) instead
            of merging all sub-meshes that share a color.

    Returns:
        The parts, in a stable order.
    """
    source = Path(source)
    key = f"{source.stem}-{_file_hash(source)}-v2" + (
        "-parts" if by_part else ""
    )
    out_dir = MESH_CACHE_DIR / key
    index_file = out_dir / "index.json"
    if index_file.exists():
        index = json.loads(index_file.read_text())
        return [
            MeshPart(
                out_dir / p["file"], p["rgba"] and tuple(p["rgba"]), p["name"]
            )
            for p in index
        ]

    groups: dict[str, list[trimesh.Trimesh]] = {}
    colors: dict[str, tuple[float, ...] | None] = {}
    for node, mesh in _load_parts(source):
        color = _part_color(mesh)
        group = node if by_part else str(color)
        if group not in groups:
            groups[group] = []
            colors[group] = color
        groups[group].append(mesh)

    out_dir.mkdir(parents=True, exist_ok=True)
    index = []
    for i, (group, meshes) in enumerate(groups.items()):
        merged = trimesh.util.concatenate(meshes)
        merged.visual = trimesh.visual.ColorVisuals()
        file = f"{i:03d}.stl"
        merged.export(out_dir / file)
        index.append(
            {
                "file": file,
                "rgba": colors[group],
                "name": group if by_part else None,
            }
        )
    index_file.write_text(json.dumps(index, indent=1))
    return split_by_color(source, by_part=by_part)


def mesh_extent(path: Path) -> np.ndarray:
    """Axis-aligned bounding-box size of a mesh file (meters)."""
    return trimesh.load_mesh(path).extents

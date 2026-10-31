"""Read rig geometry from the Commander's ``commander.yaml``.

The Commander builds its MoveIt planning scene from the ``planning_scene``
block of ``tabletop_rig/config/commander.yaml``: box primitives, STEP/STL rig
meshes, and the 10x3 grid of task objects. This module parses that block into
plain dataclasses so the MuJoCo model is generated from the same source of
truth as the rig.

Poses follow the rig's conventions (``tabletop_rig.utils.ros``): positions in
meters in the ``world`` frame; ``rpy`` is static-axis XYZ ("sxyz") in radians;
``orientation`` quaternions are ``[w, x, y, z]``.
"""

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from virtual_tabletop import paths


@dataclass(frozen=True)
class Pose:
    """A world-frame pose.

    Attributes:
        position: ``(x, y, z)`` in meters.
        quat: ``(w, x, y, z)`` unit quaternion (MuJoCo order).
    """

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    @classmethod
    def from_config(cls, cfg: dict[str, Any] | None) -> "Pose":
        """Build a pose from a ``pose_stamped`` mapping in commander.yaml."""
        cfg = cfg or {}
        if "pose" in cfg:
            cfg = {**cfg["pose"], **cfg}
        position = tuple(float(v) for v in cfg.get("position", (0, 0, 0)))
        if "rpy" in cfg and "orientation" in cfg:
            raise ValueError("orientation and rpy cannot both be provided")
        if "rpy" in cfg:
            x, y, z, w = Rotation.from_euler("xyz", cfg["rpy"]).as_quat()
            quat = (w, x, y, z)
        elif "orientation" in cfg:
            quat = np.asarray(cfg["orientation"], dtype=float)
            quat = tuple(quat / np.linalg.norm(quat))
        else:
            quat = (1.0, 0.0, 0.0, 0.0)
        return cls(position=position, quat=tuple(float(q) for q in quat))


@dataclass(frozen=True)
class BoxPrimitive:
    """A box from ``planning_scene.rig.primitives``.

    Attributes:
        name: Collision object id.
        size: Full edge lengths ``(x, y, z)`` in meters (may contain zeros).
        pose: Box center pose.
        rgba: Display color.
    """

    name: str
    size: tuple[float, float, float]
    pose: Pose
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class RigMesh:
    """A mesh from ``planning_scene.rig.meshes``.

    Attributes:
        name: Collision object id.
        path: Mesh file (STEP, STL or DAE).
        pose: Mesh origin pose.
        rgba: Display color.
        subframes: Named frames relative to the mesh origin.
    """

    name: str
    path: Path
    pose: Pose
    rgba: tuple[float, float, float, float]
    subframes: dict[str, Pose] = field(default_factory=dict)


@dataclass(frozen=True)
class GridObject:
    """A task object on the presentation grid.

    Attributes:
        object_id: Object id, e.g. ``"small_object_0"``.
        grid_idx: ``(column, row)`` grid index, the ``"x,y"`` config key.
        mesh_path: Object mesh file.
        pose: Object pose when seated on its mount.
        rgba: Display color.
    """

    object_id: str
    grid_idx: tuple[int, int]
    mesh_path: Path
    pose: Pose
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class RigScene:
    """Everything the planning scene declares."""

    primitives: list[BoxPrimitive]
    meshes: list[RigMesh]
    grid_objects: list[GridObject]


_FIND_PKG = re.compile(r"\$\(find-pkg-share ([A-Za-z0-9_]+)\)")


def _resolve_path(value: str) -> Path:
    """Expand ``$(find-pkg-share pkg)`` against the repository checkout."""
    packages = paths.ros_packages()
    return Path(_FIND_PKG.sub(lambda m: str(packages[m.group(1)]), value))


def _find_key(node: Any, key: str) -> Any:
    """Depth-first search for the first mapping entry named ``key``."""
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for value in node.values():
            found = _find_key(value, key)
            if found is not None:
                return found
    return None


def _rgba(value: Any) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in (value or (0.7, 0.7, 0.7, 1.0)))


def _object_meshes(mesh_dir: Path) -> dict[str, Path]:
    """Map object ids to mesh files, as ``MoveItInterface`` does."""
    meshes = {}
    for pattern in ("*.stl", "*.dae"):
        for path in sorted(mesh_dir.glob(pattern)):
            meshes[path.stem] = path
    return meshes


def load_rig_scene(config_file: Path = paths.COMMANDER_CONFIG) -> RigScene:
    """Parse the planning scene from ``commander.yaml``.

    Args:
        config_file: Commander parameter file.

    Returns:
        The rig primitives, rig meshes and grid objects.
    """
    config = yaml.safe_load(config_file.read_text())
    scene = _find_key(config, "planning_scene")
    if scene is None:
        raise ValueError(f"No planning_scene block in {config_file}")

    primitives = []
    for name, cfg in (scene["rig"].get("primitives") or {}).items():
        if cfg["type"] != "BOX":
            raise NotImplementedError(
                f"Primitive {name}: type {cfg['type']} not supported yet"
            )
        primitives.append(
            BoxPrimitive(
                name=name,
                size=tuple(float(v) for v in cfg["dimensions"]),
                pose=Pose.from_config(cfg.get("pose_stamped")),
                rgba=_rgba(cfg.get("color")),
            )
        )

    meshes = []
    for name, cfg in (scene["rig"].get("meshes") or {}).items():
        meshes.append(
            RigMesh(
                name=name,
                path=_resolve_path(cfg["path"]),
                pose=Pose.from_config(cfg.get("pose_stamped")),
                rgba=_rgba(cfg.get("color")),
                subframes={
                    sub: Pose.from_config(sub_cfg)
                    for sub, sub_cfg in (cfg.get("subframes") or {}).items()
                },
            )
        )

    grid_cfg = scene["grid_objects"]
    mesh_dir = _resolve_path(grid_cfg["mesh_dir"])
    object_meshes = _object_meshes(mesh_dir)
    common = grid_cfg.get("common_kwargs") or {}
    grid_objects = []
    for idx, overrides in grid_cfg["object_kwargs"].items():
        cfg = {**common, **overrides}
        object_id = cfg.get("object_id")
        if object_id is None:
            continue  # empty grid slot
        if object_id not in object_meshes:
            raise ValueError(
                f"Object mesh {object_id} not found in {mesh_dir}"
            )
        x, y = (int(v) for v in idx.split(","))
        grid_objects.append(
            GridObject(
                object_id=object_id,
                grid_idx=(x, y),
                mesh_path=object_meshes[object_id],
                pose=Pose.from_config(cfg["pose_stamped"]),
                rgba=_rgba(cfg.get("color")),
            )
        )

    return RigScene(
        primitives=primitives, meshes=meshes, grid_objects=grid_objects
    )

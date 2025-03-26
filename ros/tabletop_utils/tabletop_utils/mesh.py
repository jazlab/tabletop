import logging
import os
from collections import OrderedDict

import pyfqmr
import pyglet
import trimesh


# Mesh utility functions
def load_geometry(
    path: str, scale: float = 1.0
) -> trimesh.Trimesh | trimesh.Scene:
    """
    Load a mesh from a file and scale it.
    """
    if os.path.splitext(path)[1] == ".stl":
        geometry = trimesh.load_mesh(path)
    elif os.path.splitext(path)[1] == ".dae":
        geometry = trimesh.load_scene(path)
    else:
        raise ValueError(
            f"Unsupported mesh file extension '{os.path.splitext(path)[1]}' for {path}"
        )
    return geometry.apply_scale(scale)


def _simplify_quadratic_decimation_mesh(
    mesh: trimesh.Trimesh,
    target_count: int = 100,
    aggressiveness: int = 7,
    preserve_border: bool = True,
):
    mesh_simplifier = pyfqmr.Simplify()  # type: ignore
    mesh_simplifier.setMesh(mesh.vertices, mesh.faces)
    logging.basicConfig(level=logging.DEBUG)
    logging.getLogger("pyfqmr")
    mesh_simplifier.simplify_mesh(
        target_count=target_count,
        aggressiveness=aggressiveness,
        preserve_border=preserve_border,
        verbose=True,
    )
    vertices, faces, _ = mesh_simplifier.getMesh()
    mesh_simplified = trimesh.Trimesh(vertices=vertices, faces=faces)
    return mesh_simplified


def simplify_quadratic_decimation(
    geometry: trimesh.Trimesh | trimesh.Scene,
    target_count: int = 100,
    aggressiveness: int = 7,
    preserve_border: bool = True,
):
    if isinstance(geometry, trimesh.Scene):
        updates = {
            k: simplify_quadratic_decimation(
                v,
                target_count=target_count,
                aggressiveness=aggressiveness,
                preserve_border=preserve_border,
            )
            for k, v in geometry.geometry.items()  # type: ignore
        }
        scene = geometry.copy()
        scene.geometry.update(updates)
        return scene
    else:
        return _simplify_quadratic_decimation_mesh(
            geometry,  # type: ignore
            target_count,
            aggressiveness,
            preserve_border,
        )


def simplify_bounding_primitive(
    geometry: trimesh.Trimesh | trimesh.Scene,
):
    if isinstance(geometry, trimesh.Scene):
        updates = {
            k: simplify_bounding_primitive(v)
            for k, v in geometry.geometry.items()  # type: ignore
        }
        scene = geometry.copy()
        scene.geometry.update(updates)
        return scene
    else:
        return geometry.bounding_primitive.to_mesh()  # type: ignore


def get_bounding_primitives_mesh_scene(scene: trimesh.Scene):
    return OrderedDict(
        (k, v.bounding_primitive) for k, v in scene.geometry.items()
    )


def simplify_convex_hull(
    geometry: trimesh.Trimesh | trimesh.Scene,
):
    if isinstance(geometry, trimesh.Scene):
        updates = {
            k: simplify_convex_hull(v)
            for k, v in geometry.geometry.items()  # type: ignore
        }
        scene = geometry.copy()
        scene.geometry.update(updates)
        return scene
    else:
        return geometry.convex_hull


def visualize_geometry(geometry: trimesh.Trimesh | trimesh.Scene):
    try:
        # if hasattr(geometry, "lights"):
        #     geometry.lights = []  # type: ignore

        window = geometry.show(start_loop=False)
        pyglet.app.run()
    except KeyboardInterrupt:
        if window is not None:
            window.close()  # type: ignore


def export_geometry(geometry: trimesh.Trimesh | trimesh.Scene, path: str):
    if isinstance(geometry, trimesh.Scene):
        dae_bytes = trimesh.exchange.dae.export_collada(  # type: ignore
            list(geometry.geometry.values())
        )
        with open(path, "wb") as f:
            f.write(dae_bytes)
    else:
        geometry.export(path)

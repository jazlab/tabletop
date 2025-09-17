import argparse
import os
from typing import Any, Optional, TypeVar, cast

import numpy as np
import trimesh
from trimesh.exchange.dae import export_collada

GeometryT = TypeVar("GeometryT", bound=trimesh.Trimesh | trimesh.Scene)


# Mesh utility functions


def copy_geometry(geometry: GeometryT) -> GeometryT:
    """Copy a mesh or scene.

    Args:
        geometry: The mesh or scene to copy.
    """
    if isinstance(geometry, trimesh.Scene):
        return trimesh.Scene(geometry.dump())  # type: ignore
    else:
        return geometry.copy()  # type: ignore


def scale_geometry(geometry: GeometryT, scale: float) -> GeometryT:
    """Scale a mesh or scene (not in-place).

    Args:
        geometry: The mesh or scene to scale.
        scale: The scale to apply to the mesh.
    """
    geometry = copy_geometry(geometry)
    return geometry.apply_scale(scale)


def transform_geometry(geometry: GeometryT, tf: np.ndarray) -> GeometryT:
    """Transform a mesh or scene by a transformation matrix (not in-place).

    Args:
        geometry: The mesh or scene to transform.
        tf: The transformation matrix to apply to the mesh or scene.

    Returns:
        The transformed mesh or scene.
    """
    geometry = copy_geometry(geometry)
    return cast(GeometryT, geometry.apply_transform(tf))


def load_geometry(
    path: str, scale: Optional[float] = None
) -> trimesh.Trimesh | trimesh.Scene:
    """Load a mesh from a file and scale it.

    Args:
        path: The path to the mesh file.
        scale: The scale to apply to the mesh.

    Returns:
        The loaded mesh or scene.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"Mesh path not found: {path}")
    if os.path.isdir(path):
        raise IsADirectoryError(
            f"Mesh path is a directory, not a file: {path}"
        )

    extension = os.path.splitext(path)[1]
    if extension == ".dae":
        geometry = trimesh.load_scene(path)
        if len(geometry.geometry) == 1:
            geometry = cast(trimesh.Trimesh, geometry.to_mesh())
        else:
            # Dump the scene to a new scene to "bake" any metadata
            # into each mesh
            geometry = trimesh.Scene(geometry.dump())
    elif extension == ".stl":
        geometry = trimesh.load_mesh(path)
    else:
        raise ValueError(
            f"Unsupported mesh file extension '{extension}' for {path}"
        )

    if scale is not None:
        geometry = scale_geometry(geometry, scale)

    return geometry


def _simplify_quadratic_decimation_mesh(
    mesh: trimesh.Trimesh,
    target_count: int = 100,
    aggressiveness: int = 7,
    preserve_border: bool = True,
) -> trimesh.Trimesh:
    """Simplify a mesh using quadratic decimation.

    Args:
        mesh: The mesh to simplify.
        target_count: The target number of faces to simplify to.
        aggressiveness: The aggressiveness of the simplification.
        preserve_border: Whether to preserve the border of the mesh.

    Returns:
        The simplified mesh.
    """
    import pyfqmr  # type: ignore

    mesh_simplifier = pyfqmr.Simplify()  # type: ignore
    mesh_simplifier.setMesh(mesh.vertices, mesh.faces)
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
    geometry: GeometryT,
    target_count: int = 100,
    aggressiveness: int = 7,
    preserve_border: bool = True,
) -> GeometryT:
    """Simplify a mesh or scene using quadratic decimation.

    Args:
        geometry: The mesh or scene to simplify.
        target_count: The target number of faces to simplify to.
        aggressiveness: The aggressiveness of the simplification.
        preserve_border: Whether to preserve the border of the mesh.

    Returns:
        The simplified mesh or scene.
    """
    if isinstance(geometry, trimesh.Scene):
        geometry = copy_geometry(geometry)
        updates = {
            k: simplify_quadratic_decimation(
                v,
                target_count=target_count,
                aggressiveness=aggressiveness,
                preserve_border=preserve_border,
            )
            for k, v in geometry.geometry.items()
        }
        geometry.geometry.update(updates)
        return geometry  # type: ignore
    else:
        return _simplify_quadratic_decimation_mesh(
            geometry, target_count, aggressiveness, preserve_border
        )


def simplify_bounding_primitive(geometry: GeometryT) -> GeometryT:
    """Simplify a mesh or scene using bounding primitives.

    Args:
        geometry: The mesh or scene to simplify.

    Returns:
        The simplified mesh or scene.
    """
    if isinstance(geometry, trimesh.Scene):
        geometry = copy_geometry(geometry)
        updates = {
            k: simplify_bounding_primitive(v)
            for k, v in geometry.geometry.items()
        }
        geometry.geometry.update(updates)
        return geometry  # type: ignore
    else:
        return geometry.bounding_primitive.to_mesh()


def simplify_convex_hull(geometry: GeometryT) -> GeometryT:
    """Simplify a mesh or scene using convex hull simplification.

    Args:
        geometry: The mesh or scene to simplify.

    Returns:
        The simplified mesh or scene.
    """
    if isinstance(geometry, trimesh.Scene):
        geometry = copy_geometry(geometry)
        updates = {
            k: simplify_convex_hull(v) for k, v in geometry.geometry.items()
        }
        geometry.geometry.update(updates)
        return geometry  # type: ignore
    else:
        return geometry.convex_hull


def visualize_geometry(
    geometry: trimesh.Trimesh | trimesh.Scene,
    notebook: bool = False,
    axis_scale: float = 0.2,
) -> Any | None:
    """Visualize a mesh or scene.

    Args:
        geometry: The mesh or scene to visualize.
    """
    import pyglet

    geometry = copy_geometry(geometry)
    if isinstance(geometry, trimesh.Trimesh):
        scene = trimesh.Scene([geometry])
    else:
        scene = geometry

    # if hasattr(geometry, "lights"):
    #     geometry.lights = []  # type: ignore
    axis = cast(trimesh.Trimesh, trimesh.creation.axis())
    axis = axis.apply_scale(
        axis_scale * scene.extents.max() / axis.extents.max()
    )
    scene.add_geometry(axis)

    print("Axis colors | x: red, y: green, z: blue")

    if notebook:
        return scene.show()
    else:
        window = scene.show(start_loop=False)
        try:
            pyglet.app.run()
        except KeyboardInterrupt:
            if window is not None:
                window.close()  # type: ignore


def export_geometry(geometry: trimesh.Trimesh | trimesh.Scene, path: str):
    """Export a mesh or scene to a file.

    Args:
        geometry: The mesh or scene to export.
        path: The path to the file to export to.
    """
    if isinstance(geometry, trimesh.Scene):
        dae_bytes = export_collada(geometry.dump())
        with open(path, "wb") as f:
            f.write(dae_bytes)
    else:
        geometry.export(path)


def count_vertices_faces(
    geometry: trimesh.Trimesh | trimesh.Scene,
) -> tuple[int, int]:
    """Count the number of vertices and faces in a mesh or scene.

    Args:
        geometry: The mesh or scene to count the vertices and faces of.
    """
    if isinstance(geometry, trimesh.Scene):
        return (
            sum(g.vertices.shape[0] for g in geometry.geometry.values()),
            sum(g.faces.shape[0] for g in geometry.geometry.values()),
        )
    else:
        return geometry.vertices.shape[0], geometry.faces.shape[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", type=str, help="Mesh file to simplify")
    parser.add_argument(
        "-o",
        "--output-path",
        type=str,
        help="Output file to save the simplified mesh. If not provided, the simplified mesh will be saved in the same directory as the input file.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        help="Scale the mesh by the given factor",
    )
    parser.add_argument(
        "--simplification",
        type=str,
        choices=["convex_hull", "bounding_primitive", "quadric_decimation"],
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Preview the simplified mesh",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="Force overwrite of existing output file",
    )
    args = parser.parse_args()

    # Check if the input file exists
    if not os.path.isfile(args.input_file):
        raise FileNotFoundError(f"File {args.input_file} does not exist")

    # Get the input file basename and extension
    input_basename, input_ext = os.path.splitext(args.input_file)

    # Handle the output path
    if not args.preview:
        if args.output_path is None:
            args.output_path = args.input_file
        elif os.path.isdir(args.output_path):
            args.output_path = os.path.join(
                args.output_path,
                f"{input_basename}_simplified{input_ext}",
            )

        if os.path.isfile(args.output_path) and not args.force:
            raise FileExistsError(
                f"File {args.output_path} already exists. Use --force to overwrite."
            )

    # Check if the scale and simplification method are provided
    if args.scale is None and args.simplification is None:
        raise ValueError(
            "No scale or simplification method provided. Please provide one of the following: --scale, --simplification"
        )

    # Load the mesh
    if input_ext == ".stl":
        print("Loading STL file as a mesh...")
        mesh = trimesh.load_mesh(args.input_file)
    elif input_ext == ".dae":
        print("Loading DAE file as a scene...")
        mesh = trimesh.load_scene(args.input_file)
    else:
        raise ValueError(f"Unsupported file type: {input_ext}")

    # Print the original mesh summary
    print("Original mesh:")
    print(mesh)
    print(f"Extents: {mesh.extents.round(2)}")

    # Preview the original mesh
    if args.preview:
        visualize_geometry(mesh)

    # Scale the mesh if a scale is provided
    if args.scale is not None:
        mesh.apply_scale(args.scale)

    # Simplify the mesh
    match args.simplification:
        case "convex_hull":
            mesh = simplify_convex_hull(mesh)
        case "bounding_primitive":
            mesh = simplify_bounding_primitive(mesh)
        case "quadric_decimation":
            mesh = simplify_quadratic_decimation(mesh)

    # Print the simplified mesh summary
    print("Simplified mesh:")
    print(mesh)
    print(f"Extents: {mesh.extents.round(2)}")

    # Preview the simplified mesh
    if args.preview:
        visualize_geometry(mesh)
    # Export the simplified mesh
    else:
        if args.output_path is None:
            args.output_path = args.input_file
        elif os.path.isdir(args.output_path):
            args.output_path = os.path.join(
                args.output_path,
                f"{input_basename}_simplified{input_ext}",
            )

        if os.path.isfile(args.output_path) and not args.force:
            raise FileExistsError(
                f"File {args.output_path} already exists. Use --force to overwrite."
            )

        mesh.export(args.output_path)


if __name__ == "__main__":
    main()

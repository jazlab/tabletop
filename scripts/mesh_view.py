#!/usr/bin/python3

import argparse

import numpy as np
from transformations import euler_matrix, translation_matrix

from tabletop_py.utils.mesh import (
    count_vertices_faces,
    load_geometry,
    simplify_bounding_primitive,
    simplify_convex_hull,
    simplify_quadratic_decimation,
    transform_geometry,
    visualize_geometry,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=str, help="Path to the mesh file")
    parser.add_argument(
        "--simplification",
        type=str,
        default=None,
        choices=["convex_hull", "bounding_primitive", "quadratic_decimation"],
        help="Simplification method to apply to the mesh",
    )
    parser.add_argument(
        "--scale", type=float, default=None, help="Scale factor"
    )
    parser.add_argument(
        "--rpy",
        type=float,
        nargs=3,
        default=None,
        help="Roll, pitch, yaw of the mesh",
    )
    parser.add_argument(
        "--translation",
        type=float,
        nargs=3,
        default=None,
        help="Translation of the mesh",
    )
    args = parser.parse_args()

    geometry = load_geometry(path=args.path, scale=args.scale)

    print("Summary:")
    print(f"Geometry: {geometry}")
    print(f"Extents: {geometry.extents}")
    num_vertices, num_faces = count_vertices_faces(geometry)
    print(f"Number of vertices: {num_vertices}")
    print(f"Number of faces: {num_faces}")

    if args.simplification == "convex_hull":
        geometry = simplify_convex_hull(geometry)
    elif args.simplification == "bounding_primitive":
        geometry = simplify_bounding_primitive(geometry)
    elif args.simplification == "quadratic_decimation":
        geometry = simplify_quadratic_decimation(geometry)
    elif args.simplification is not None:
        raise ValueError(f"Unsupported simplification: {args.simplification}")

    print(f"Summary after {args.simplification} simplification:")
    print(f"Geometry: {geometry}")
    print(f"Extents: {geometry.extents}")
    num_vertices, num_faces = count_vertices_faces(geometry)
    print(f"Number of vertices: {num_vertices}")
    print(f"Number of faces: {num_faces}")

    if args.rpy is not None or args.translation is not None:
        rotation = (
            euler_matrix(args.rpy[0], args.rpy[1], args.rpy[2], "sxyz")
            if args.rpy is not None
            else np.eye(4)
        )
        translation = (
            translation_matrix(args.translation)
            if args.translation is not None
            else np.eye(4)
        )
        geometry = transform_geometry(geometry, rotation @ translation)

    visualize_geometry(geometry)


if __name__ == "__main__":
    main()

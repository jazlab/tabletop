#!/usr/bin/python3

import argparse
import os

import trimesh
from tabletop_server.utils import (
    simplify_bounding_primitive,
    simplify_convex_hull,
    simplify_quadratic_decimation,
    visualize_geometry,
)


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

#!/usr/bin/python3

import argparse
import os

import trimesh
from tabletop_utils.mesh import visualize_geometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stl_file", type=str)
    args = parser.parse_args()

    if not os.path.isfile(args.stl_file):
        raise FileNotFoundError(f"File {args.stl_file} does not exist")

    if args.stl_file.endswith(".stl"):
        mesh = trimesh.load_mesh(args.stl_file)
    elif args.stl_file.endswith(".dae"):
        mesh = trimesh.load_scene(args.stl_file)
    else:
        raise ValueError(f"Unsupported file type: {args.stl_file}")

    print("Summary:")
    print(mesh)
    print(mesh.extents)

    # mesh = simplify_convex_hull(mesh)
    if isinstance(mesh, trimesh.Scene):
        mesh.lights = []

    visualize_geometry(mesh)


if __name__ == "__main__":
    main()

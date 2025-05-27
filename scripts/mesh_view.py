#!/usr/bin/python3

import argparse

from tabletop_utils.mesh import (
    load_geometry,
    simplify_bounding_primitive,
    simplify_convex_hull,
    simplify_quadratic_decimation,
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
    args = parser.parse_args()

    geometry = load_geometry(path=args.path, scale=args.scale)

    print("Summary:")
    print(geometry)
    print(geometry.extents)

    if args.simplification == "convex_hull":
        geometry = simplify_convex_hull(geometry)
    elif args.simplification == "bounding_primitive":
        geometry = simplify_bounding_primitive(geometry)
    elif args.simplification == "quadratic_decimation":
        geometry = simplify_quadratic_decimation(geometry)
    elif args.simplification is not None:
        raise ValueError(f"Unsupported simplification: {args.simplification}")

    visualize_geometry(geometry)


if __name__ == "__main__":
    main()

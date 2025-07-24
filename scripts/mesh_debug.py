#!/usr/bin/python3

import os

from tabletop_utils.mesh import (
    load_geometry,
    simplify_bounding_primitive,
    simplify_convex_hull,
    visualize_geometry,
)
from tabletop_utils.ros import pose_msg_from_matrix
from tf_transformations import inverse_matrix


def main_mesh():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--filename",
        type=str,
        default=os.path.join(
            os.environ["TABLETOP_DIR"],
            "ros/tabletop_description/meshes/rig_mesh5.stl",
        ),
    )
    args = parser.parse_args()

    mesh = load_geometry(args.filename)

    tf = mesh.bounding_box_oriented.transform
    tf_inv = inverse_matrix(tf)
    mesh_pose = pose_msg_from_matrix(tf_inv)
    print(mesh_pose)
    mesh_primitive = simplify_bounding_primitive(mesh)
    print(mesh_primitive)
    mesh.show()
    # bounding_primitive.show()
    # mesh_simplified = simplify_mesh(mesh)
    # mesh_simplified.show()


def main_scene():
    scene = load_geometry(
        os.path.join(
            os.environ["TABLETOP_DIR"],
            "ros/tabletop_description/meshes/static/rig.dae",
        ),
        scale=1.0,
    )
    # visualize_geometry(scene)
    scene_mesh = scene.to_mesh()  # type: ignore
    visualize_geometry(scene_mesh)
    scene_convex_hull = simplify_convex_hull(scene)
    visualize_geometry(scene_convex_hull)
    scene_convex_hull_mesh = scene_convex_hull.to_mesh()  # type: ignore
    visualize_geometry(scene_convex_hull_mesh)
    scene_bounding_primitives = simplify_bounding_primitive(scene)
    visualize_geometry(scene_bounding_primitives)
    scene_bounding_primitives_mesh = scene_bounding_primitives.to_mesh()  # type: ignore
    visualize_geometry(scene_bounding_primitives_mesh)


if __name__ == "__main__":
    main_scene()

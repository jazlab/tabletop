import logging

import pyfqmr
import trimesh
from tabletop_server.utils import pose_from_matrix
from tf_transformations import inverse_matrix


def load_mesh(filename: str, scale: float = 0.001):
    mesh = trimesh.load_mesh(filename)
    return mesh.apply_scale(scale)


def get_bounding_primitive(mesh: trimesh.Trimesh):
    return mesh.bounding_primitive.to_mesh()


def get_bounding_box_oriented(mesh: trimesh.Trimesh):
    return mesh.bounding_box_oriented


def simplify_mesh(mesh: trimesh.Trimesh):
    mesh_simplifier = pyfqmr.Simplify()  # type: ignore
    mesh_simplifier.setMesh(mesh.vertices, mesh.faces)
    logging.basicConfig(level=logging.DEBUG)
    logging.getLogger("pyfqmr")
    mesh_simplifier.simplify_mesh(
        target_count=100,
        aggressiveness=1,
        preserve_border=False,
        verbose=True,
    )
    vertices, faces, normals = mesh_simplifier.getMesh()
    mesh_simplified = trimesh.Trimesh(vertices=vertices, faces=faces)
    return mesh_simplified


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--filename",
        type=str,
        default="/root/ws/src/tabletop/ros/tabletop_description/meshes/rig_mesh5.stl",
    )
    args = parser.parse_args()

    mesh = load_mesh(args.filename)

    tf = mesh.bounding_box_oriented.transform
    tf_inv = inverse_matrix(tf)
    mesh_pose = pose_from_matrix(tf_inv)
    bounding_primitive = get_bounding_primitive(mesh)
    mesh.show()
    # bounding_primitive.show()
    # mesh_simplified = simplify_mesh(mesh)
    # mesh_simplified.show()


if __name__ == "__main__":
    main()

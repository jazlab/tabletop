#!/usr/bin/python3

import argparse
import pathlib

import trimesh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stl_file", type=pathlib.Path)
    args = parser.parse_args()

    mesh = trimesh.load(args.stl_file)
    mesh.show()


if __name__ == "__main__":
    main()

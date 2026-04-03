import os
from glob import glob

from setuptools import find_packages, setup

package_name = "tabletop_rig"


def nested_data_files(share_path: str, dir: str):
    data_files = []

    for path, _, files in os.walk(dir):
        item = (
            os.path.join(share_path, path),
            [os.path.join(path, f) for f in files],
        )
        data_files.append(item)

    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        *nested_data_files(os.path.join("share", package_name), "config"),
        *nested_data_files(os.path.join("share", package_name), "soundfonts"),
        *nested_data_files(os.path.join("share", package_name), "meshes"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valmiki Kothare",
    maintainer_email="valmiki.kothare.vk@gmail.com",
    description="The rig nodes for the TableTop project",
    license="MIT",
    entry_points={
        "console_scripts": [
            "commander = tabletop_rig.nodes.commander:main",
            "mock_teensy = tabletop_rig.nodes.mock_teensy:main",
            "mock_dashboard_client = tabletop_rig.nodes.mock_dashboard_client:main",
            "mock_robot_state_helper = tabletop_rig.nodes.mock_robot_state_helper:main",
            "eyelink = tabletop_rig.nodes.eyelink:main",
            "flic = tabletop_rig.nodes.flic:main",
            "rosbag_to_csv = tabletop_rig.utils.rosbag:main",
        ],
    },
)

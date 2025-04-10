import os
from glob import glob

from setuptools import find_packages, setup

package_name = "tabletop_server"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valmiki Kothare",
    maintainer_email="valmiki.kothare.vk@gmail.com",
    description="The server nodes for the TableTop project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "example_commander = tabletop_server.nodes.commander:main",
            "mock_teensy = tabletop_server.nodes.mock_teensy:main",
            "mock_dashboard = tabletop_server.nodes.mock_dashboard:main",
            "flic = tabletop_server.nodes.flic:main",
            "flic_client = tabletop_server.flic_client.client_aio:main",
        ],
    },
)

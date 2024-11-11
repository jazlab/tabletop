import os
from glob import glob

from setuptools import find_packages, setup

package_name = "tabletop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valmiki Kothare",
    maintainer_email="valmiki.kothare.vk@gmail.com",
    description="ROS2 package for the TableTop project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control = tabletop.nodes:control",
            "teensy_sensor = tabletop.nodes:teensy_sensor",
            "teensy_control = tabletop.nodes:teensy_control",
        ],
    },
)

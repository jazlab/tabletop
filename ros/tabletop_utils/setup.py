from setuptools import find_packages, setup

package_name = "tabletop_utils"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valmiki Kothare",
    maintainer_email="valmiki.kothare.vk@gmail.com",
    description="Common utilities for the TableTop project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flic_client = tabletop_utils.flic_client:main",
            "rosbag_to_csv = tabletop_utils.rosbag:main",
        ],
    },
)

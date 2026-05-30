import os
from glob import glob

from setuptools import find_packages, setup

package_name = "forest_semantic_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tese",
    maintainer_email="tese@fe.up.pt",
    description="Late fusion between semantic mask and lidar points",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "semantic_point_fusion_node = forest_semantic_fusion.semantic_point_fusion_node:main",
        ],
    },
)


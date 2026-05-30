import os
from glob import glob

from setuptools import find_packages, setup

package_name = "forest_lidar_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tese",
    maintainer_email="tese@fe.up.pt",
    description="YDLidar X4 bringup and scan→PointCloud2 for forest stack",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "laserscan_to_pointcloud2 = forest_lidar_ros2.laserscan_to_pointcloud2:main",
            "fdpo_ydlidar_x4 = forest_lidar_ros2.fdpo_ydlidar_x4_node:main",
        ],
    },
)

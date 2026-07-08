import os
from glob import glob

from setuptools import find_packages, setup

package_name = "forest_vision_detection"

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
    description="Camera-based object detection: Gazebo auto-labeler + YOLO inference",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gz_auto_labeler = forest_vision_detection.gz_auto_labeler_node:main",
            "yolo_detector = forest_vision_detection.yolo_detector_node:main",
            "relabel_offline = forest_vision_detection.relabel_offline:main",
        ],
    },
)

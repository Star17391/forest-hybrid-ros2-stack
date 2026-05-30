import os
from glob import glob

from setuptools import find_packages, setup

package_name = "forest_nicla_vision_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (
            os.path.join("share", package_name, "firmware", "nicla_sensor_node"),
            ["firmware/nicla_sensor_node/nicla_sensor_node.ino"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tese",
    maintainer_email="tese@fe.up.pt",
    description="Nicla Vision USB serial sensor bridge for forest-hybrid-ros2-stack",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "nicla_serial_bridge = forest_nicla_vision_ros2.nicla_serial_bridge:main",
            "nicla_device_probe = forest_nicla_vision_ros2.nicla_device_probe:main",
        ],
    },
)

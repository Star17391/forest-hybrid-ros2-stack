import os
from glob import glob

from setuptools import find_packages, setup

package_name = "forest_sim_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="forest-hybrid",
    maintainer_email="tese@fe.up.pt",
    description="Gazebo-ROS2 bridge nodes for forest-hybrid-ros2-stack",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "marble_pose_from_gz = forest_sim_bridge.marble_pose_from_gz:main",
            "marble_sensor_tf_static = forest_sim_bridge.marble_sensor_tf_static:main",
            "gz_track_odometry_stamp = forest_sim_bridge.gz_track_odometry_stamp:main",
            "twist_to_marble_tracks = forest_sim_bridge.twist_to_marble_tracks:main",
            "keyboard_marble_tracks = forest_sim_bridge.keyboard_marble_tracks:main",
            "forest_cleanup = forest_sim_bridge.cleanup_zombies:main",
            "forest_mission_panel = forest_sim_bridge.mission_panel:main",
            "forest_teleop_panel = forest_sim_bridge.teleop_panel:main",
            "forest_random_explore = forest_sim_bridge.random_explore:main",
            "imu_debug_markers = forest_sim_bridge.imu_debug_markers:main",
            "gz_odom_base_bootstrap = forest_sim_bridge.gz_odom_base_bootstrap:main",
        ],
    },
)

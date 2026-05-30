"""Nicla Vision via ADVR drivers (submodule) with forest topic remaps.

Configure once in repo root: config/forest_nicla_advr_config.h
Then: bash scripts/nicla_advr_apply_config.sh
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    forest_pkg = get_package_share_directory("forest_nicla_vision_ros2")
    params_file = os.path.join(forest_pkg, "config", "nicla_advr_receiver.yaml")
    # Prefix must match FOREST_NICLA_ROS_NAME in config/forest_nicla_advr_config.h
    nicla = "nicla"

    return LaunchDescription(
        [
            Node(
                package="nicla_vision_ros2",
                executable="nicla_receiver",
                name="nicla_receiver",
                output="screen",
                parameters=[params_file],
                remappings=[
                    (f"/{nicla}/camera/image_raw", "/camera/image_raw"),
                    (f"/{nicla}/camera/image_raw/compressed", "/camera/image_raw/compressed"),
                    (f"/{nicla}/camera/camera_info", "/camera/camera_info"),
                    (f"/{nicla}/imu", "/sensors/imu/data"),
                    (f"/{nicla}/tof", "/sensors/tof/range"),
                    (f"/{nicla}/audio", "/sensors/nicla/audio"),
                    (f"/{nicla}/audio_stamped", "/sensors/nicla/audio_stamped"),
                    (f"/{nicla}/audio_info", "/sensors/nicla/audio_info"),
                    (f"/{nicla}/audio_recognized", "/sensors/nicla/audio_recognized"),
                ],
            ),
        ]
    )

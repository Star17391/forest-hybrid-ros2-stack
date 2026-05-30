import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("forest_semantic_fusion")
    params = os.path.join(pkg, "config", "semantic_point_fusion.yaml")
    return LaunchDescription(
        [
            Node(
                package="forest_semantic_fusion",
                executable="semantic_point_fusion_node",
                name="semantic_point_fusion_node",
                output="screen",
                parameters=[params],
            )
        ]
    )


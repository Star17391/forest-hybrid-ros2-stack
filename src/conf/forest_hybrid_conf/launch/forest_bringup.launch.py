"""Bringup base: câmara (usb/CSI), modo de operação, segmentação."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    operation_mode = LaunchConfiguration("operation_mode")
    camera_backend = LaunchConfiguration("camera_backend")
    onnx_model_path = LaunchConfiguration("onnx_model_path")
    model_input_width = LaunchConfiguration("model_input_width")
    model_input_height = LaunchConfiguration("model_input_height")

    camera_pi_launch = os.path.join(
        get_package_share_directory("forest_camera_ros2"),
        "launch",
        "camera_pi.launch.py",
    )
    camera_usb_launch = os.path.join(
        get_package_share_directory("forest_camera_ros2"),
        "launch",
        "camera_usb.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Sincronizar com relógio de simulação",
            ),
            DeclareLaunchArgument(
                "operation_mode",
                default_value="ground",
                description="ground | aerial — em aerial a segmentação não processa imagens",
            ),
            DeclareLaunchArgument(
                "camera_backend",
                default_value="usb",
                description="usb | csi (csi usa camera_ros/libcamera)",
            ),
            DeclareLaunchArgument(
                "onnx_model_path",
                default_value="",
                description="Caminho para o modelo ONNX de segmentação; vazio => máscara 0",
            ),
            DeclareLaunchArgument(
                "model_input_width",
                default_value="512",
                description="Largura de entrada do modelo ONNX",
            ),
            DeclareLaunchArgument(
                "model_input_height",
                default_value="384",
                description="Altura de entrada do modelo ONNX",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_usb_launch),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(PythonExpression(["'", camera_backend, "' == 'usb'"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_pi_launch),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(PythonExpression(["'", camera_backend, "' == 'csi'"])),
            ),
            Node(
                package="forest_robot_supervisor",
                executable="operation_mode_node",
                name="operation_mode_node",
                output="screen",
                parameters=[
                    {"operation_mode": operation_mode, "use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="forest_semantic_segmentation",
                executable="semantic_segmentation_node",
                name="semantic_segmentation_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "onnx_model_path": onnx_model_path,
                        "model_input_width": model_input_width,
                        "model_input_height": model_input_height,
                    }
                ],
            ),
        ]
    )

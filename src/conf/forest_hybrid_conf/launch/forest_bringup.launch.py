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
    enable_camera = LaunchConfiguration("enable_camera")
    camera_backend = LaunchConfiguration("camera_backend")
    enable_operation_mode = LaunchConfiguration("enable_operation_mode")
    operation_mode = LaunchConfiguration("operation_mode")
    onnx_model_path = LaunchConfiguration("onnx_model_path")
    model_input_width = LaunchConfiguration("model_input_width")
    model_input_height = LaunchConfiguration("model_input_height")
    enable_semantic_fusion = LaunchConfiguration("enable_semantic_fusion")

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
                "camera_backend",
                default_value="usb",
                description="usb | csi (csi usa camera_ros/libcamera)",
            ),
            DeclareLaunchArgument(
                "enable_operation_mode",
                default_value="true",
                description="Publicar /system/locomotion_mode via operation_mode_node",
            ),
            DeclareLaunchArgument(
                "operation_mode",
                default_value="ground",
                description="ground | aerial (apenas se enable_operation_mode=true)",
            ),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="true para lançar camada de câmara; false para testes sem câmara",
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
            DeclareLaunchArgument(
                "enable_semantic_fusion",
                default_value="false",
                description="Publicar /perception/semantic_points (late fusion RGB+LiDAR).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_usb_launch),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(
                    PythonExpression(["'", enable_camera, "' == 'true' and '", camera_backend, "' == 'usb'"])
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_pi_launch),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(
                    PythonExpression(["'", enable_camera, "' == 'true' and '", camera_backend, "' == 'csi'"])
                ),
            ),
            Node(
                package="forest_robot_supervisor",
                executable="operation_mode_node",
                name="operation_mode_node",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"operation_mode": operation_mode},
                ],
                condition=IfCondition(
                    PythonExpression(["'", enable_operation_mode, "' == 'true'"])
                ),
            ),
            Node(
                package="forest_planner_ros2",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
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
            Node(
                package="forest_semantic_fusion",
                executable="semantic_point_fusion_node",
                name="semantic_point_fusion_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(
                    PythonExpression(["'", enable_semantic_fusion, "' == 'true'"])
                ),
            ),
        ]
    )

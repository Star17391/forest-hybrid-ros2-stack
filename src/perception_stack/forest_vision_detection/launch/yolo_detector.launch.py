"""Launch the YOLO inference node (camera object detection).

Publishes:
  - /perception/camera/detections        (vision_msgs/Detection2DArray)  -> Tree-SLAM fusion
  - /perception/camera/detections_image  (sensor_msgs/Image, bgr8)       -> RViz overlay

`ultralytics` (+ torch) only lives in the ardupilot venv, not in the system
python that ROS console-scripts are shebanged to. So this launch runs the node
with the VENV interpreter directly (ExecuteProcess on the installed script),
resolved from (in order):
  1. launch arg python_exe
  2. FOREST_YOLO_PYTHON env
  3. ~/venv-ardupilot/bin/python
  4. fallback: system python3 (node starts but stays idle without ultralytics)

Model resolution (in order):
  1. launch arg model_path
  2. FOREST_YOLO_MODEL env
  3. newest forest-vision-training/runs/.../weights/best.pt on disk
  4. _DEFAULT_MODEL below

Usage:
  ros2 launch forest_vision_detection yolo_detector.launch.py
  ros2 launch forest_vision_detection yolo_detector.launch.py \
      model_path:=/abs/best.pt conf_threshold:=0.30
"""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

_DEFAULT_MODEL = str(
    Path.home()
    / "Projetos/Tese/forest-vision-training/runs/detect/runs/detect"
    / "forest_vision_v5/weights/best.pt"
)


def _newest_trained_model() -> str | None:
    root = Path.home() / "Projetos/Tese/forest-vision-training"
    candidates = list(root.glob("runs/**/weights/best.pt"))
    if not candidates:
        return None
    return str(max(candidates, key=lambda p: p.stat().st_mtime))


def _resolve_model(context) -> str:
    explicit = LaunchConfiguration("model_path").perform(context).strip()
    if explicit:
        return explicit
    env = os.environ.get("FOREST_YOLO_MODEL", "").strip()
    if env:
        return env
    return _newest_trained_model() or _DEFAULT_MODEL


def _resolve_python(context) -> str:
    explicit = LaunchConfiguration("python_exe").perform(context).strip()
    if explicit:
        return explicit
    env = os.environ.get("FOREST_YOLO_PYTHON", "").strip()
    if env:
        return env
    venv = Path.home() / "venv-ardupilot" / "bin" / "python"
    if venv.is_file():
        return str(venv)
    return "python3"


def _installed_script() -> str:
    from ament_index_python.packages import get_package_prefix

    prefix = Path(get_package_prefix("forest_vision_detection"))
    return str(prefix / "lib" / "forest_vision_detection" / "yolo_detector")


def _bool(context, key: str) -> str:
    return "true" if LaunchConfiguration(key).perform(context).lower() in (
        "1", "true", "yes",
    ) else "false"


def _setup(context, *_args, **_kwargs):
    model_path = _resolve_model(context)
    python_exe = _resolve_python(context)
    script = _installed_script()

    if not Path(model_path).is_file():
        print(f"[yolo_detector.launch] WARNING: model not found at {model_path}")
    if python_exe == "python3":
        print(
            "[yolo_detector.launch] WARNING: venv python not found; running under "
            "system python3 — ultralytics likely missing → node will be idle."
        )

    params = [
        ("model_path", model_path),
        ("image_topic", LaunchConfiguration("image_topic").perform(context)),
        ("detections_topic", LaunchConfiguration("detections_topic").perform(context)),
        ("overlay_topic", LaunchConfiguration("overlay_topic").perform(context)),
        ("publish_overlay", _bool(context, "publish_overlay")),
        ("conf_threshold", LaunchConfiguration("conf_threshold").perform(context)),
        ("iou_threshold", LaunchConfiguration("iou_threshold").perform(context)),
        ("use_sim_time", _bool(context, "use_sim_time")),
    ]
    ros_args: list[str] = ["--ros-args", "-r", "__node:=yolo_detector"]
    for key, val in params:
        ros_args += ["-p", f"{key}:={val}"]

    return [
        ExecuteProcess(
            cmd=[python_exe, script, *ros_args],
            name="yolo_detector",
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument(
                "python_exe",
                default_value="",
                description="Python interpreter with ultralytics (default: "
                "FOREST_YOLO_PYTHON env, else ~/venv-ardupilot/bin/python).",
            ),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument(
                "detections_topic", default_value="/perception/camera/detections"
            ),
            DeclareLaunchArgument(
                "overlay_topic", default_value="/perception/camera/detections_image"
            ),
            DeclareLaunchArgument("publish_overlay", default_value="true"),
            DeclareLaunchArgument("conf_threshold", default_value="0.35"),
            DeclareLaunchArgument("iou_threshold", default_value="0.45"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )

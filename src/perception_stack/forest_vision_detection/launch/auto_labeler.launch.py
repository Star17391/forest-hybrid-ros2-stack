"""Launch the Gazebo ground-truth auto-labeler node.

Resolves the world SDF from (in priority order):
  1. launch arg world_sdf (explicit absolute path)
  2. launch arg world_name → $FORESTGEN_PATH/worlds/<name>.sdf
  3. FOREST_LAUNCH_OVERRIDES env (world:=<name>) set by `forest up --world`
  4. profile default (forest_realistic_v2_trees_rocks)

This keeps the labeler's parsed world in sync with whatever world the sim layer
actually loaded, so `forest up sim-vision-capture --world X` labels world X.
"""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_WORLD = "forest_realistic_v2_trees_rocks"


def _forestgen_path() -> Path:
    # Matches tools/forest/lib/env.bash default; forest up always exports this.
    return Path(os.environ.get(
        "FORESTGEN_PATH",
        str(Path.home() / "Projetos" / "Gazebo" / "ForestGen"),
    ))


def _world_from_overrides() -> str | None:
    raw = os.environ.get("FOREST_LAUNCH_OVERRIDES", "")
    for part in raw.split(","):
        part = part.strip()
        if part.startswith("world:="):
            name = part.split("world:=", 1)[1].strip()
            # strip optional .sdf extension
            return name[:-4] if name.endswith(".sdf") else name
    return None


def _resolve(context, *args, **kwargs):
    from ament_index_python.packages import get_package_share_directory
    pkg_share = Path(get_package_share_directory("forest_vision_detection"))
    cfg = str(pkg_share / "config" / "auto_labeler.yaml")

    explicit_sdf = LaunchConfiguration("world_sdf").perform(context).strip()
    world_name = LaunchConfiguration("world_name").perform(context).strip()
    output_dir = LaunchConfiguration("output_dir").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    if explicit_sdf:
        sdf = explicit_sdf
        resolved_name = Path(explicit_sdf).stem
    else:
        name = world_name or _world_from_overrides() or _DEFAULT_WORLD
        if name.endswith(".sdf"):
            name = name[:-4]
        resolved_name = name
        sdf = str(_forestgen_path() / "worlds" / f"{name}.sdf")

    # Each world gets its own subdir so multi-world captures don't share a
    # meta.json/poses.csv (relabel_offline assumes one world per dataset dir).
    # Set output_dir explicitly to an absolute leaf to opt out of this.
    if LaunchConfiguration("per_world_subdir").perform(context).lower() in ("true", "1"):
        output_dir = str(Path(output_dir) / resolved_name)

    return [
        Node(
            package="forest_vision_detection",
            executable="gz_auto_labeler",
            name="gz_auto_labeler",
            output="screen",
            parameters=[
                cfg,
                {
                    "world_sdf": sdf,
                    "output_dir": output_dir,
                    "use_sim_time": use_sim_time.lower() in ("true", "1"),
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_out = str(Path.home() / "datasets" / "forest_vision_labels")
    return LaunchDescription([
        DeclareLaunchArgument("world_sdf", default_value="",
                              description="Absolute path to world SDF (overrides world_name)"),
        DeclareLaunchArgument("world_name", default_value="",
                              description="World basename under $FORESTGEN_PATH/worlds (no .sdf)"),
        DeclareLaunchArgument("output_dir", default_value=default_out,
                              description="Root dir for images/ and labels/ output"),
        DeclareLaunchArgument("per_world_subdir", default_value="true",
                              description="Append the world name to output_dir so each "
                                          "world captures into its own dataset folder"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_resolve),
    ])

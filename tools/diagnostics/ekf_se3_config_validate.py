#!/usr/bin/env python3
"""Validação estática da configuração EKF SE(3) dual (sem Gazebo).

Garante alinhamento entre ekf_local/global, imu_sanitize, launch e freeze legacy.
Exit 0 = OK; exit 1 = falhas encontradas.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any

try:
    import yaml  # type: ignore
except ImportError:
    print("ERROR: PyYAML em falta (sudo apt install python3-yaml)", file=sys.stderr)
    sys.exit(2)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: root must be mapping")
    return data


def ekf_params(path: Path) -> dict[str, Any]:
    data = load_yaml(path)
    for key in ("ekf_local", "ekf_filter_node"):
        if key in data and isinstance(data[key], dict):
            params = data[key].get("ros__parameters")
            if isinstance(params, dict):
                return params
    raise ValueError(f"{path}: missing ros__parameters block")


def imu_config_flags(cfg: list[Any]) -> tuple[bool, bool, bool]:
    """Return (gyro_fused_3d, accel_fused_any, roll_pitch_abs_fused).

    roll_pitch_abs (indices 3,4) is the ABSOLUTE orientation. In SE(3)
    (two_d_mode=false) without it — and without accel as a gravity reference —
    roll/pitch integrate from the gyro with NO absolute anchor and drift after
    rotations, tilting the odom→base TF and wrecking point-cloud heights in RViz.
    """
    if len(cfg) != 15:
        raise ValueError(f"imu0_config length {len(cfg)}, expected 15")
    gyro = all(cfg[i] for i in (9, 10, 11))
    accel = any(cfg[i] for i in (12, 13, 14))
    roll_pitch_abs = cfg[3] and cfg[4]
    return gyro, accel, roll_pitch_abs


class CheckResult:
    def __init__(self) -> None:
        self.ok: list[str] = []
        self.fail: list[str] = []

    def add(self, name: str, passed: bool, detail: str = "") -> None:
        msg = f"{name}" + (f" — {detail}" if detail else "")
        (self.ok if passed else self.fail).append(msg)

    def exit_code(self) -> int:
        return 0 if not self.fail else 1


def matrix_size(name: str, flat: list[Any], n: int) -> bool:
    return isinstance(flat, list) and len(flat) == n * n


def check_ekf_local(path: Path, r: CheckResult) -> None:
    p = ekf_params(path)
    r.add("ekf_local.two_d_mode=false", p.get("two_d_mode") is False)
    r.add("ekf_local.world_frame=odom", p.get("world_frame") == "odom")
    r.add("ekf_local.publish_tf=true", p.get("publish_tf") is True)
    r.add(
        "ekf_local.odom0=wheel",
        p.get("odom0") == "/sensors/wheel_odometry",
    )
    cfg = p.get("imu0_config")
    if isinstance(cfg, list):
        gyro, accel, roll_pitch_abs = imu_config_flags(cfg)
        r.add("ekf_local.imu gyro 3D", gyro, "indices 9–11")
        r.add("ekf_local.imu NO accel", not accel, "indices 12–14 must be false")
        # SE(3) attitude anchor: roll/pitch absolutes MUST be fused, otherwise
        # attitude drifts (root cause of the RViz height/rotation bug).
        r.add(
            "ekf_local.imu roll/pitch absolutes fused",
            bool(roll_pitch_abs),
            "indices 3,4 must be true in SE(3) — anchors attitude to gravity",
        )
    else:
        r.add("ekf_local.imu0_config present", False)
    r.add(
        "ekf_local.initial_estimate_covariance 15x15",
        matrix_size("initial", p.get("initial_estimate_covariance", []), 15),
    )
    r.add(
        "ekf_local.process_noise_covariance 15x15",
        matrix_size("process", p.get("process_noise_covariance", []), 15),
    )


def check_authority(path: Path, r: CheckResult) -> None:
    p = load_yaml(path).get("map_odom_authority_node", {}).get("ros__parameters", {})
    r.add("authority.ground_mode in {identity,silent}",
          p.get("ground_mode") in ("identity", "silent"))
    r.add("authority.ardupilot_odom_topic=/ardupilot/local_position_odom",
          p.get("ardupilot_odom_topic") == "/ardupilot/local_position_odom")


def check_imu_sanitize(path: Path, r: CheckResult) -> None:
    text = path.read_text(encoding="utf-8")
    r.add(
        "imu_sanitize accel cov all -1",
        "linear_acceleration_covariance.fill(-1.0)" in text,
        "3 eixos bloqueados para robot_localization",
    )
    r.add(
        "imu_sanitize no single-axis -1 only",
        "linear_acceleration_covariance[0] = -1.0" not in text
        or "fill(-1.0)" in text,
    )


def check_state_estimation_launch(path: Path, r: CheckResult) -> None:
    text = path.read_text(encoding="utf-8")
    r.add('launch ekf_local node name', 'name="ekf_local"' in text)
    r.add("launch map_odom_authority", "map_odom_authority" in text)
    r.add("launch ekf_global REMOVED", "ekf_global" not in text or "removido" in text)
    r.add(
        "launch publish_map_odom_identity arg",
        "publish_map_odom_identity" in text,
    )


def check_sim_gazebo_slam_blocked(path: Path, r: CheckResult) -> None:
    text = path.read_text(encoding="utf-8")
    r.add(
        "sim_gazebo use_slam blocked",
        "use_slam:=true" in text and "LEGACY" in text,
    )


def check_legacy_freeze(root: Path, r: CheckResult) -> None:
    legacy_doc = root / "docs" / "LEGACY_PATHS.md"
    r.add("docs/LEGACY_PATHS.md", legacy_doc.is_file())
    profiles = root / "tools" / "forest" / "profiles" / "legacy"
    for name in ("sim-mvp-nav.yaml", "sim-slam-nav.yaml"):
        p = profiles / name
        if p.is_file():
            data = load_yaml(p)
            r.add(
                f"legacy/{name} status=legacy",
                data.get("status") == "legacy",
            )
        else:
            r.add(f"legacy/{name} exists", False)
    slam_launch = (
        root
        / "src"
        / "localization_mapping_stack"
        / "forest_2d_localization"
        / "launch"
        / "slam_toolbox_online_async.launch.py"
    )
    if slam_launch.is_file():
        t = slam_launch.read_text(encoding="utf-8")
        r.add("slam_toolbox launch FOREST_ALLOW_LEGACY guard", "FOREST_ALLOW_LEGACY" in t)
    else:
        r.add("slam_toolbox launch exists", False)


def check_perception_cylinder_comments(root: Path, r: CheckResult) -> None:
    exp_cpp = (
        root
        / "src"
        / "perception_stack"
        / "forest_3d_perception"
        / "src"
        / "lidar3d_experimental_node.cpp"
    )
    if not exp_cpp.is_file():
        r.add("lidar3d_experimental_node.cpp", False)
        return
    text = exp_cpp.read_text(encoding="utf-8")
    r.add(
        "cylinder fit NOT mislabeled RANSAC in active code path",
        "not RANSAC" in text or "centroid + median radius" in text,
    )
    bad = re.findall(r"Cylinder RANSAC", text)
    r.add(
        "no stale 'Cylinder RANSAC' comments",
        len(bad) == 0,
        f"found {len(bad)}" if bad else "",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate EKF SE3 static configuration")
    parser.add_argument("--repo", type=Path, default=repo_root())
    args = parser.parse_args()
    root: Path = args.repo

    est = root / "src" / "localization_mapping_stack" / "forest_state_estimation"
    local_yaml = est / "config" / "ekf_local.yaml"
    authority_yaml = est / "config" / "map_odom_authority.yaml"
    launch = est / "launch" / "state_estimation.launch.py"
    imu_cpp = root / "src" / "drivers_stack" / "forest_sensors_cpp" / "src" / "imu_sanitize_node.cpp"
    sim_gazebo = root / "src" / "sim_bridge" / "forest_sim_bridge" / "launch" / "sim_gazebo.launch.py"

    r = CheckResult()
    for p, label in (
        (local_yaml, "ekf_local.yaml"),
        (authority_yaml, "map_odom_authority.yaml"),
        (launch, "state_estimation.launch.py"),
        (imu_cpp, "imu_sanitize_node.cpp"),
        (sim_gazebo, "sim_gazebo.launch.py"),
    ):
        if not p.is_file():
            r.add(f"{label} exists", False)
            continue

    if local_yaml.is_file():
        check_ekf_local(local_yaml, r)
    if authority_yaml.is_file():
        check_authority(authority_yaml, r)
    if imu_cpp.is_file():
        check_imu_sanitize(imu_cpp, r)
    if launch.is_file():
        check_state_estimation_launch(launch, r)
    if sim_gazebo.is_file():
        check_sim_gazebo_slam_blocked(sim_gazebo, r)
    check_legacy_freeze(root, r)
    check_perception_cylinder_comments(root, r)

    print("=== EKF SE3 config validation ===")
    for msg in r.ok:
        print(f"  OK  {msg}")
    for msg in r.fail:
        print(f"  FAIL {msg}", file=sys.stderr)
    print(f"\n{len(r.ok)} passed, {len(r.fail)} failed")
    return r.exit_code()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Headless Gazebo probe: enable multicopter + gentle vz, detect tumble (|roll| or |pitch| > limit).

Requires GZ_SIM_RESOURCE_PATH with ForestGen models. No ROS needed.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_WORLD = (
    Path.home() / "Projetos/Gazebo/ForestGen/worlds/mvp_hybrid_flat.sdf"
)
def _quat_to_roll_pitch(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def _parse_gz_model_pose(text: str) -> tuple[float, float, float] | None:
    """Parse ``gz model -m marble_hd2 -p`` (YAML-ish) output."""
    m = re.search(
        r"Pose:\s*\[([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\]",
        text,
    )
    if m:
        x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
        qx, qy, qz, qw = (
            float(m.group(4)),
            float(m.group(5)),
            float(m.group(6)),
            float(m.group(7)),
        )
        roll, pitch = _quat_to_roll_pitch(qx, qy, qz, qw)
        return roll, pitch, z
    # protobuf text from topic echo
    m = re.search(
        r"position:\s*\n\s*x:\s*([-\d.e+]+)\s*\n\s*y:\s*([-\d.e+]+)\s*\n\s*z:\s*([-\d.e+]+)",
        text,
    )
    q = re.search(
        r"orientation:\s*\n\s*x:\s*([-\d.e+]+)\s*\n\s*y:\s*([-\d.e+]+)\s*\n\s*z:\s*([-\d.e+]+)\s*\n\s*w:\s*([-\d.e+]+)",
        text,
    )
    if m and q:
        z = float(m.group(3))
        roll, pitch = _quat_to_roll_pitch(
            float(q.group(1)), float(q.group(2)), float(q.group(3)), float(q.group(4))
        )
        return roll, pitch, z
    return None


def _gz_model_pose(env: dict[str, str], timeout_sec: float) -> str:
    try:
        r = subprocess.run(
            ["gz", "model", "-m", "marble_hd2", "-p"],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            env=env,
        )
        return (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired:
        return ""


def _gz_topic_pub(env: dict[str, str], topic: str, msg_type: str, payload: str) -> None:
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", msg_type, "-p", payload],
        env=env,
        capture_output=True,
        timeout=5,
        check=False,
    )


def run_probe(
    world: Path,
    duration_sec: float,
    max_tilt_deg: float,
    vz_cmd: float,
) -> tuple[bool, str]:
    forestgen = Path.home() / "Projetos/Gazebo/ForestGen"
    env = os.environ.copy()
    paths = [str(forestgen / "models"), str(forestgen / "worlds")]
    if env.get("GZ_SIM_RESOURCE_PATH"):
        paths.append(env["GZ_SIM_RESOURCE_PATH"])
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(paths)

    if not world.is_file():
        return False, f"world not found: {world}"

    sim = subprocess.Popen(
        ["gz", "sim", "-r", "-s", str(int(duration_sec * 500)), str(world)],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(2.5)
    _gz_topic_pub(env, "/marble_hd2/enable", "gz.msgs.Boolean", "data: true")
    time.sleep(0.3)
    _gz_topic_pub(
        env,
        "/marble_hd2/gazebo/command/twist",
        "gz.msgs.Twist",
        f"linear: {{x: 0, y: 0, z: {vz_cmd}}}",
    )

    max_roll = 0.0
    max_pitch = 0.0
    max_z = 0.0
    samples = 0
    deadline = time.monotonic() + duration_sec - 1.0
    detail = ""

    while time.monotonic() < deadline:
        raw = _gz_model_pose(env, 3.0)
        parsed = _parse_gz_model_pose(raw)
        if parsed:
            roll, pitch, z = parsed
            max_roll = max(max_roll, abs(roll))
            max_pitch = max(max_pitch, abs(pitch))
            max_z = max(max_z, z)
            samples += 1
        time.sleep(0.4)

    sim.terminate()
    try:
        sim.wait(timeout=5)
    except subprocess.TimeoutExpired:
        sim.kill()

    tilt_lim = math.radians(max_tilt_deg)
    ok = samples >= 3 and max_roll < tilt_lim and max_pitch < tilt_lim
    detail = (
        f"samples={samples} max|roll|={math.degrees(max_roll):.1f}° "
        f"max|pitch|={math.degrees(max_pitch):.1f}° max_z={max_z:.2f}m "
        f"(limit {max_tilt_deg:.0f}°)"
    )
    return ok, detail


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Gazebo headless aerial stability probe")
    p.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    p.add_argument("--duration", type=float, default=14.0)
    p.add_argument("--max-tilt-deg", type=float, default=35.0)
    p.add_argument("--vz", type=float, default=0.12)
    args = p.parse_args(argv)

    ok, detail = run_probe(args.world, args.duration, args.max_tilt_deg, args.vz)
    print(detail)
    if ok:
        print("PASS: takeoff sample stable (no large roll/pitch)")
        return 0
    print("FAIL: tumble or insufficient pose samples", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())

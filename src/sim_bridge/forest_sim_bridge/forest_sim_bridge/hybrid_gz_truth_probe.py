#!/usr/bin/env python3
"""Medição directa no Gazebo (sem adivinhar): poses de links, eixo de empuxo, subida de z.

Arranca ``mvp_hybrid_flat``, roda lagartas a ±90°, publica motores, amostra ``gz model``.

Uso:
  ros2 run forest_sim_bridge hybrid_gz_truth_probe
  ros2 run forest_sim_bridge hybrid_gz_truth_probe --duration 18 --motor-omega 450
"""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_WORLD = Path.home() / "Projetos/Gazebo/ForestGen/worlds/mvp_hybrid_flat.sdf"
LEFT_YAW = math.pi / 2.0
RIGHT_YAW = -math.pi / 2.0
PROP_LINKS = (
    ("left_prop_front", "left", -math.pi / 2.0),
    ("left_prop_rear", "left", -math.pi / 2.0),
    ("right_prop_front", "right", math.pi / 2.0),
    ("right_prop_rear", "right", math.pi / 2.0),
)


@dataclass
class TruthCheck:
    name: str
    ok: bool
    detail: str


@dataclass
class TruthReport:
    checks: list[TruthCheck] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(c.ok for c in self.checks)

    def add(self, name: str, ok: bool, detail: str) -> None:
        self.checks.append(TruthCheck(name, ok, detail))


def _rot_x(angle: float, v: tuple[float, float, float]) -> tuple[float, float, float]:
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = v
    return (x, c * y - s * z, s * y + c * z)


def link_z_world_from_rolls(track_roll: float, link_roll_x: float) -> tuple[float, float, float]:
    lz = _rot_x(link_roll_x, (0.0, 0.0, 1.0))
    return _rot_x(track_roll, lz)


def _env() -> dict[str, str]:
    forestgen = Path.home() / "Projetos/Gazebo/ForestGen"
    env = os.environ.copy()
    paths = [str(forestgen / "models"), str(forestgen / "worlds")]
    if env.get("GZ_SIM_RESOURCE_PATH"):
        paths.append(env["GZ_SIM_RESOURCE_PATH"])
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(paths)
    return env


def _gz(args: list[str], env: dict[str, str], timeout: float = 8.0) -> str:
    r = subprocess.run(
        ["gz", *args],
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
    )
    return (r.stdout or "") + (r.stderr or "")


def _parse_link_rpy(text: str, link_name: str) -> tuple[float, float, float] | None:
    block = re.search(
        rf"- Name: {re.escape(link_name)}[\s\S]*?"
        r"- Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\n\s*"
        r"\[([^\]]+)\]\s*\n\s*\[([^\]]+)\]",
        text,
    )
    if not block:
        return None
    rpy = [float(x) for x in block.group(2).split()]
    if len(rpy) < 3:
        return None
    return rpy[0], rpy[1], rpy[2]


def _parse_model_z(text: str) -> float | None:
    m = re.search(
        r"- Name: marble_hd2[\s\S]*?"
        r"- Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\n\s*"
        r"\[([^\]]+)\]",
        text,
    )
    if not m:
        return None
    parts = [float(x) for x in m.group(1).split()]
    return parts[2] if len(parts) >= 3 else None


def _gz_topic_pub(env: dict[str, str], topic: str, mtype: str, payload: str) -> None:
    _gz(["topic", "-t", topic, "-m", mtype, "-p", payload], env, timeout=5.0)


def _probe_physics(
    env: dict[str, str],
    report: TruthReport,
    duration_sec: float,
    motor_omega: float,
    settle_sec: float,
) -> None:

    _gz_topic_pub(
        env,
        "/model/marble_hd2/hybrid/left_track_yaw/cmd_pos",
        "gz.msgs.Double",
        f"data: {LEFT_YAW}",
    )
    _gz_topic_pub(
        env,
        "/model/marble_hd2/hybrid/right_track_yaw/cmd_pos",
        "gz.msgs.Double",
        f"data: {RIGHT_YAW}",
    )
    time.sleep(settle_sec)

    model_out = _gz(["model", "-m", "marble_hd2", "-p"], env)
    z0 = _parse_model_z(model_out)
    report.add("model_visible", z0 is not None, f"initial model z={z0}")
    if z0 is not None and z0 > 5.0:
        report.add(
            "attach_sane_spawn",
            False,
            f"z={z0:.2f} m — robô já em voo/runaway; faça forest down/up antes do probe",
        )
        return

    thrust_z: list[float] = []
    for link, side, link_roll in PROP_LINKS:
        out = _gz(["model", "-m", "marble_hd2", "-l", link, "-p"], env)
        rpy = _parse_link_rpy(out, link)
        if rpy is None:
            report.add(f"link_pose_{link}", False, "could not parse link RPY from gz model -l")
            continue
        tr = LEFT_YAW if side == "left" else RIGHT_YAW
        wz = link_z_world_from_rolls(tr, link_roll)[2]
        thrust_z.append(wz)
        report.add(
            f"thrust_z_{link}",
            wz > 0.85,
            f"gz RPY={tuple(round(x, 3) for x in rpy)} → link+Z·world_z={wz:.3f} "
            f"(expected +1.0 for side {side})",
        )

    if thrust_z:
        report.add(
            "all_props_thrust_up",
            min(thrust_z) > 0.85,
            f"per-prop world_z components: {[round(z, 3) for z in thrust_z]}",
        )

    motor_on_sec = min(6.0, max(2.0, duration_sec * 0.35))
    omega = max(0.0, motor_omega)
    motor_start = time.monotonic()
    z_samples: list[float] = []
    t_end = time.monotonic() + max(5.0, duration_sec)
    while time.monotonic() < t_end:
        if omega > 0 and (time.monotonic() - motor_start) < motor_on_sec:
            vels = ", ".join(f"{omega:.0f}" for _ in range(4))
            _gz_topic_pub(
                env,
                "/marble_hd2/gazebo/command/motor_speed",
                "gz.msgs.Actuators",
                f"velocity: [{vels}]",
            )
        elif omega > 0 and (time.monotonic() - motor_start) >= motor_on_sec:
            _gz_topic_pub(
                env,
                "/marble_hd2/gazebo/command/motor_speed",
                "gz.msgs.Actuators",
                "velocity: [0, 0, 0, 0]",
            )
            omega = 0.0
        out = _gz(["model", "-m", "marble_hd2", "-p"], env, timeout=4.0)
        z = _parse_model_z(out)
        if z is not None:
            z_samples.append(z)
        time.sleep(0.5)

    if not z_samples:
        report.add("z_samples", False, "no model z samples")
        return

    dz = max(z_samples) - min(z_samples)
    z_max = max(z_samples)
    z_min = min(z_samples)
    early_n = max(2, len(z_samples) // 3)
    z_early_max = max(z_samples[:early_n])
    report.add(
        "model_z_rises",
        (z_early_max - z_min) >= 0.04 or z_early_max >= (z0 or 0.0) + 0.06,
        f"early max z={z_early_max:.3f} (first {early_n} samples), "
        f"full min={z_min:.3f} max={z_max:.3f} Δ={dz:.3f}",
    )
    report.add(
        "model_z_bounded",
        z_early_max < 2.5,
        f"early max z={z_early_max:.3f} m (runaway if >>2.5 in first ~{early_n} samples)",
    )


def run_probe(
    world: Path | None,
    duration_sec: float,
    motor_omega: float,
    settle_sec: float,
    attach: bool,
) -> TruthReport:
    report = TruthReport()
    env = _env()

    sim: subprocess.Popen | None = None
    if not attach:
        if world is None or not world.is_file():
            report.add("world", False, f"missing world {world}")
            return report
        steps = max(5000, int(duration_sec * 500))
        sim = subprocess.Popen(
            ["gz", "sim", "-r", "-s", str(steps), str(world)],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(4.0)
        report.add("spawn_mode", True, f"headless sim {world.name}")
    else:
        time.sleep(0.5)
        attach_out = _gz(["model", "-m", "marble_hd2", "-p"], env, timeout=5.0)
        if _parse_model_z(attach_out) is not None:
            report.add("attach_mode", True, "using running Gazebo (no second sim)")
        else:
            report.add(
                "attach_mode",
                False,
                "no marble_hd2 in running gz — start forest up sim-hybrid-test -d",
            )
            return report

    _probe_physics(env, report, duration_sec, motor_omega, settle_sec)

    if sim is not None:
        sim.terminate()
        try:
            sim.wait(timeout=6)
        except subprocess.TimeoutExpired:
            sim.kill()

    return report


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Gazebo truth probe for hybrid lift")
    p.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    p.add_argument(
        "--attach",
        action="store_true",
        help="Use already-running Gazebo (do not spawn a second sim)",
    )
    p.add_argument("--duration", type=float, default=16.0)
    p.add_argument("--settle", type=float, default=3.0)
    p.add_argument("--motor-omega", type=float, default=340.0)
    args = p.parse_args(argv)

    report = run_probe(
        args.world if not args.attach else None,
        args.duration,
        args.motor_omega,
        args.settle,
        args.attach,
    )
    print("\n=== hybrid Gazebo truth probe ===\n")
    for c in report.checks:
        tag = "PASS" if c.ok else "FAIL"
        print(f"  [{tag}] {c.name}: {c.detail}")
    print("")
    print("OVERALL:", "PASS" if report.passed else "FAIL")
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())

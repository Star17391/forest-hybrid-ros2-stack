#!/usr/bin/env python3
"""Offline audit: forest_hybrid_robot mass budget, thrust/motor tuning, allocation rank.

Exit 0 if all checks pass; 1 otherwise. Use with ``forest test hybrid-physics``.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

G = 9.81
# X3 reference (Gazebo Fuel model, ~1.5 kg airframe)
X3_MASS_KG = 1.5
X3_MOTOR_K = 8.54858e-06
X3_MOMENT_K = 0.016
X3_MAX_OMEGA = 800.0

DEFAULT_SDF = Path.home() / "Projetos/Gazebo/ForestGen/models/forest_hybrid_robot/model.sdf"


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str


@dataclass
class AuditReport:
    sdf_path: Path
    total_mass_kg: float = 0.0
    checks: list[CheckResult] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(c.ok for c in self.checks)


def _parse_masses(sdf_path: Path) -> tuple[float, list[tuple[str, float]]]:
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"No <model> in {sdf_path}")
    entries: list[tuple[str, float]] = []
    total = 0.0
    for link in model.findall("link"):
        name = link.get("name", "?")
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass_el = inertial.find("mass")
        if mass_el is None or not mass_el.text:
            continue
        m = float(mass_el.text.strip())
        entries.append((name, m))
        total += m
    return total, entries


def _parse_multicopter_params(sdf_path: Path) -> dict[str, float | str]:
    text = sdf_path.read_text(encoding="utf-8")
    out: dict[str, float | str] = {}
    m = re.search(
        r"MulticopterVelocityControl[\s\S]*?<velocityGain>\s*([^<]+)\s*</velocityGain>",
        text,
    )
    if m:
        out["velocity_gain"] = m.group(1).strip()
    m = re.search(r"<forceConstant>\s*([^<]+)\s*</forceConstant>", text)
    if m:
        out["force_constant"] = float(m.group(1).strip())
    m = re.search(r"<motorConstant>\s*([^<]+)\s*</motorConstant>", text)
    if m and "force_constant" not in out:
        out["force_constant"] = float(m.group(1).strip())
    m = re.search(r"<momentConstant>\s*([^<]+)\s*</momentConstant>", text)
    if m:
        out["moment_constant"] = float(m.group(1).strip())
    m = re.search(r"<maxRotVelocity>\s*([^<]+)\s*</maxRotVelocity>", text)
    if m:
        out["max_omega"] = float(m.group(1).strip())
    return out


def _allocation_rank(
    positions: list[tuple[float, float]], directions: list[int], k: float, m_const: float
) -> int:
    try:
        import numpy as np
    except ImportError:
        return -1
    a = np.zeros((4, 4))
    for i, ((x, y), d) in enumerate(zip(positions, directions)):
        arm = math.hypot(x, y)
        ang = math.atan2(y, x)
        a[0, i] = math.sin(ang) * arm * k
        a[1, i] = -math.cos(ang) * arm * k
        a[2, i] = -d * k * m_const
        a[3, i] = k
    return int(np.linalg.matrix_rank(a))


def _rotor_positions_from_sdf(sdf_path: Path) -> list[tuple[float, float]]:
    """Approximate XY of prop joints at spawn (tracks at 0°, shoulders Y=±0.35)."""
    tree = ET.parse(sdf_path)
    model = tree.getroot().find("model")
    if model is None:
        return []
    poses: dict[str, tuple[float, float]] = {}
    shoulder_y = 0.35
    for joint in model.findall("joint"):
        jname = joint.get("name", "")
        if not jname.endswith("_prop_front_joint") and not jname.endswith("_prop_rear_joint"):
            continue
        pose = joint.find("pose")
        if pose is None or pose.text is None or not str(pose.text).strip():
            continue
        parts = pose.text.split()
        x = float(parts[0])
        y = float(parts[1])
        if "left_" in jname:
            y += shoulder_y
        elif "right_" in jname:
            y -= shoulder_y
        poses[jname] = (x, y)
    order = [
        "left_prop_front_joint",
        "right_prop_front_joint",
        "left_prop_rear_joint",
        "right_prop_rear_joint",
    ]
    return [poses[j] for j in order if j in poses]


def run_audit(
    sdf_path: Path,
    max_mass_kg: float,
    min_twr: float,
    max_twr: float,
) -> AuditReport:
    report = AuditReport(sdf_path=sdf_path)
    total, links = _parse_masses(sdf_path)
    report.total_mass_kg = total
    params = _parse_multicopter_params(sdf_path)
    k = float(params.get("force_constant", 0.0))
    m_const = float(params.get("moment_constant", 0.0))
    max_omega = float(params.get("max_omega", 800.0))

    report.checks.append(
        CheckResult(
            "mass_budget",
            total <= max_mass_kg,
            f"total={total:.3f} kg (limit {max_mass_kg:.1f} kg); "
            f"base_link={next((m for n, m in links if n == 'base_link'), 0):.2f} kg",
        )
    )

    if total > 0 and k > 0:
        hover_omega = math.sqrt((total * G) / (4.0 * k))
        max_thrust = 4.0 * k * max_omega**2
        twr = max_thrust / (total * G)
        report.checks.append(
            CheckResult(
                "hover_rpm_plausible",
                hover_omega < 0.85 * max_omega,
                f"hover ω≈{hover_omega:.0f} rad/s ({hover_omega * 9.55:.0f} RPM), "
                f"max ω={max_omega:.0f}",
            )
        )
        report.checks.append(
            CheckResult(
                "thrust_to_weight",
                min_twr <= twr <= max_twr,
                f"TWR max≈{twr:.2f} (want {min_twr:.1f}–{max_twr:.1f}); "
                f"hover needs {total * G:.1f} N, max thrust {max_thrust:.1f} N",
            )
        )
        report.checks.append(
            CheckResult(
                "moment_constant",
                0.005 <= m_const <= 0.08,
                f"momentConstant={m_const} (X3≈{X3_MOMENT_K}; values ≥0.1 cause violent yaw)",
            )
        )
    else:
        report.checks.append(
            CheckResult("motor_params", False, "missing forceConstant in SDF")
        )

    positions = _rotor_positions_from_sdf(sdf_path)
    directions = [1, -1, -1, 1]
    text = sdf_path.read_text(encoding="utf-8")
    if "lift_rotor_fl_joint" in text:
        report.warnings.append(
            "lift_rotor_* no base_link detectado — workaround; empuxo não coincide com hélices nas lagartas"
        )

    if len(positions) == 4 and k > 0:
        rank = _allocation_rank(positions, directions, k, m_const)
        report.checks.append(
            CheckResult(
                "allocation_rank",
                rank == 4,
                f"Lee allocation matrix rank={rank} (need 4)",
            )
        )
    else:
        report.checks.append(
            CheckResult(
                "allocation_rank",
                False,
                f"expected 4 lift rotor joint poses, got {len(positions)}",
            )
        )

    vg = str(params.get("velocity_gain", ""))
    if vg:
        parts = [float(x) for x in vg.split()]
        if parts and max(parts) > 2.0:
            report.warnings.append(
                f"velocityGain {vg} is high for ~{total:.0f} kg — can cause spin/flip on takeoff"
            )

    if total > 15.0:
        report.warnings.append(
            f"Model mass {total:.1f} kg looks like old tracked robot (~36 kg); "
            "multicopter gains tuned for X3 will diverge violently"
        )
    if m_const > 0.08:
        report.warnings.append(
            f"momentConstant {m_const} is high — yaw torque per thrust error can cause spin"
        )

    return report


def _print_report(report: AuditReport, verbose: bool) -> None:
    print(f"SDF: {report.sdf_path}")
    print(f"Total mass (sum of link <mass>): {report.total_mass_kg:.3f} kg")
    print()
    for c in report.checks:
        status = "PASS" if c.ok else "FAIL"
        print(f"  [{status}] {c.name}: {c.detail}")
    if report.warnings:
        print()
        print("Warnings:")
        for w in report.warnings:
            print(f"  ! {w}")
    if verbose:
        print()
        print("Why violent spin on takeoff (typical):")
        print("  - Mass >> 7 kg with X3-like velocityGain → saturation + tumble")
        print("  - momentConstant too large → excessive yaw from small thrust mismatch")
        print("  - Multicopter enabled while legs/tracks still on ground → contact torques")
        print("  - fly_up_velocity_z too high without ramp → step in desired acceleration")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Audit forest_hybrid_robot physics for aerial mode")
    p.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    p.add_argument("--max-mass-kg", type=float, default=7.0)
    p.add_argument("--min-twr", type=float, default=1.8)
    p.add_argument("--max-twr", type=float, default=5.0)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    if not args.sdf.is_file():
        print(f"ERROR: SDF not found: {args.sdf}", file=sys.stderr)
        return 1

    report = run_audit(args.sdf, args.max_mass_kg, args.min_twr, args.max_twr)
    _print_report(report, args.verbose)
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())

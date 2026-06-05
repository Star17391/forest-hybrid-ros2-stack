#!/usr/bin/env python3
"""Diagnose why forest_hybrid_robot does not lift in AERIAL_FLY.

Modes:
  offline — SDF frames, thrust axis, TWR, prop link semantics (no Gazebo).
  live    — sample ROS topics + optional ``gz`` checks (requires ``forest up sim-hybrid-test``).

Exit 0 if all checks pass; 1 otherwise. Use with ``forest test hybrid-aerial-lift``.
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

G = 9.81
DEFAULT_SDF = Path.home() / "Projetos/Gazebo/ForestGen/models/forest_hybrid_robot/model.sdf"
MOTOR_TOPIC = "/marble_hd2/gazebo/command/motor_speed"
LEFT_AERIAL = math.pi / 2.0
RIGHT_AERIAL = -math.pi / 2.0


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    hint: str = ""


@dataclass
class LiftReport:
    mode: str
    checks: list[Check] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(c.ok for c in self.checks)

    def add(self, name: str, ok: bool, detail: str, hint: str = "") -> None:
        self.checks.append(Check(name, ok, detail, hint))


def _rot_x(angle: float, v: tuple[float, float, float]) -> tuple[float, float, float]:
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = v
    return (x, c * y - s * z, s * y + c * z)


def thrust_axis_in_world(track_roll: float, joint_axis: tuple[float, float, float]) -> tuple[float, float, float]:
    """Joint axis in track frame → world (legacy; plugin uses link +Z)."""
    return _rot_x(track_roll, joint_axis)


def thrust_link_z_in_world(track_roll: float, side: str) -> tuple[float, float, float]:
    """MulticopterMotorModel: force along link +Z; SDF link roll L=−90°, R=+90°."""
    link_roll = -math.pi / 2.0 if side == "left" else math.pi / 2.0
    lz = _rot_x(link_roll, (0.0, 0.0, 1.0))
    return _rot_x(track_roll, lz)


def _parse_motor_k(sdf_path: Path) -> float:
    text = sdf_path.read_text(encoding="utf-8")
    vals = [
        float(m.group(1).strip())
        for m in re.finditer(r"<motorConstant>\s*([^<]+)\s*</motorConstant>", text)
    ]
    return max(vals) if vals else 0.0


def _parse_model_mass(sdf_path: Path) -> float:
    tree = ET.parse(sdf_path)
    model = tree.getroot().find("model")
    if model is None:
        return 0.0
    total = 0.0
    for link in model.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass_el = inertial.find("mass")
        if mass_el is not None and mass_el.text:
            total += float(mass_el.text.strip())
    return total


def _prop_frame_semantics_ok(sdf_path: Path) -> tuple[bool, str]:
    tree = ET.parse(sdf_path)
    model = tree.getroot().find("model")
    if model is None:
        return False, "no model"
    bad: list[str] = []
    for name in (
        "left_prop_front",
        "left_prop_rear",
        "right_prop_front",
        "right_prop_rear",
    ):
        link = model.find(f'link[@name="{name}"]')
        if link is None:
            bad.append(f"{name}(missing)")
            continue
        pose = link.find("pose")
        rel = pose.get("relative_to", "") if pose is not None else ""
        if rel.endswith("_track") and pose is not None and pose.text:
            parts = pose.text.split()
            if len(parts) >= 3 and any(abs(float(parts[i])) > 1e-3 for i in range(3)):
                bad.append(name)
        elif rel != f"{name}_joint":
            bad.append(f"{name}(relative_to={rel or 'model'})")
    if bad:
        return False, f"prop link frame error: {', '.join(bad)}"
    return True, "prop links at prop joint origin (not offset on track)"


def _prop_link_thrust_roll_ok(sdf_path: Path) -> tuple[bool, str]:
    text = sdf_path.read_text(encoding="utf-8")
    missing: list[str] = []
    for name, want_roll in (
        ("left_prop_front", "-1.5708"),
        ("left_prop_rear", "-1.5708"),
        ("right_prop_front", "1.5708"),
        ("right_prop_rear", "1.5708"),
    ):
        block = re.search(rf'<link name="{name}">[\s\S]*?</link>', text)
        if not block or want_roll not in block.group(0):
            missing.append(name)
    if missing:
        return (
            False,
            f"link roll ±90° missing for MulticopterMotorModel +Z thrust: {', '.join(missing)}",
        )
    return True, "prop link roll aligns +Z with vertical thrust at drone pose"


def _foot_height_estimate_m(sdf_path: Path, leg_extension_m: float, base_z_m: float) -> float:
    """Rough foot sphere center Z in world (flat ground z=0)."""
    # support_leg joint at base z=0.04, prismatic down to leg_extension, foot at -0.09 on leg link
    return base_z_m + 0.04 - leg_extension_m - 0.09


def run_offline(
    sdf_path: Path,
    mass_kg: float,
    motor_k: float,
    hover_omega: float,
) -> LiftReport:
    report = LiftReport(mode="offline")
    if not sdf_path.is_file():
        report.add("sdf_exists", False, f"missing {sdf_path}")
        return report
    report.add("sdf_exists", True, str(sdf_path))

    ok, detail = _prop_frame_semantics_ok(sdf_path)
    report.add(
        "prop_frame_semantics",
        ok,
        detail,
        hint="Reinicia Gazebo após alterar model.sdf (forest down && forest up).",
    )
    ok2, detail2 = _prop_link_thrust_roll_ok(sdf_path)
    report.add(
        "prop_link_thrust_roll",
        ok2,
        detail2,
        hint="Sem roll no link, empuxo horizontal → pião no RViz e z fixo.",
    )

    mass = mass_kg if mass_kg > 0 else _parse_model_mass(sdf_path)
    k = motor_k if motor_k > 0 else _parse_motor_k(sdf_path)
    report.add("mass_budget", mass > 0 and mass <= 7.5, f"total mass≈{mass:.2f} kg")
    report.add("motor_constant", k > 0, f"motorConstant={k}")

    l_axis = thrust_link_z_in_world(LEFT_AERIAL, "left")
    r_axis = thrust_link_z_in_world(RIGHT_AERIAL, "right")
    l_z = l_axis[2]
    r_z = r_axis[2]
    report.add(
        "thrust_link_z_aerial",
        l_z > 0.85 and r_z > 0.85,
        f"L link +Z @+90°: world {l_axis} (z={l_z:.2f}); "
        f"R link +Z @−90°: world {r_axis} (z={r_z:.2f})",
        hint="Link sem roll ±90° no SDF → empuxo horizontal e pião no RViz.",
    )
    l_wrong = thrust_link_z_in_world(LEFT_AERIAL, "right")
    report.add(
        "prop_link_roll_required",
        l_wrong[2] < 0.5,
        f"L com frame errado (right-style) → z={l_wrong[2]:.2f} (deve ser <0.5 vs +1.0 correto)",
    )

    axis_ground = thrust_link_z_in_world(0.0, "left")
    report.add(
        "thrust_axis_ground_warning",
        abs(axis_ground[2]) < 0.2,
        f"at track 0° thrust axis→world {axis_ground} (|z|={abs(axis_ground[2]):.2f})",
        hint="Esperado: |z|≈0 em solo; se motors spinam em solo → órbita visual.",
    )

    if k > 0 and mass > 0:
        w = hover_omega if hover_omega > 0 else math.sqrt((mass * G) / (4.0 * k))
        thrust_n = 4.0 * k * w * w
        twr = thrust_n / (mass * G)
        report.add(
            "hover_thrust_margin",
            twr >= 0.95,
            f"ω={w:.0f} rad/s → thrust≈{thrust_n:.1f} N, weight={mass * G:.1f} N, TWR≈{twr:.2f}",
            hint="Se TWR<1 ou pernas/esteiras no chão, não há lift.",
        )

    foot_z = _foot_height_estimate_m(sdf_path, 0.17, 0.35)
    report.add(
        "support_legs_ground_contact",
        foot_z > 0.02,
        f"foot center z≈{foot_z:.3f} m (spawn base_z=0.35, legs=0.17 m); "
        "feet above ground → pernas não explicam solo sozinho",
        hint="Verificar também colisão das lagartas/esteiras após rotação ±90°.",
    )

    return report


def _gz_topic_echo_sample(topic: str, timeout_sec: float) -> tuple[bool, str]:
    try:
        r = subprocess.run(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
        out = (r.stdout or "") + (r.stderr or "")
        if "velocity" in out or "Velocity" in out:
            return True, "received Actuators on Gazebo transport"
        return False, f"no actuator message (exit {r.returncode}): {out[:200]}"
    except subprocess.TimeoutExpired:
        return False, f"timeout {timeout_sec}s (Gazebo parado ou tópico sem publisher?)"
    except FileNotFoundError:
        return False, "gz CLI not found"


def _parse_imu_gyro_max(sample_sec: float) -> tuple[bool, float, str]:
    try:
        r = subprocess.run(
            [
                "ros2",
                "topic",
                "echo",
                "/sensors/imu/data_raw",
                "--spin-time",
                str(sample_sec),
            ],
            capture_output=True,
            text=True,
            timeout=sample_sec + 8.0,
        )
        out = r.stdout or ""
        vals = [
            abs(float(m.group(1)))
            for m in re.finditer(
                r"angular_velocity:\s*\n(?:\s*\w+:\s*[-\d.e+]+\s*\n){2}\s*z:\s*([-\d.e+]+)",
                out,
            )
        ]
        if not vals:
            vals = [
                abs(float(m.group(1)))
                for m in re.finditer(r"angular_velocity:[\s\S]*?z:\s*([-\d.e+]+)", out)
            ]
        if not vals:
            return False, 0.0, "no IMU samples (is sim bridge up?)"
        peak = max(vals)
        return True, peak, f"peak |gyro_z|≈{peak:.2f} rad/s over {sample_sec:.0f}s"
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return False, 0.0, str(e)


def _gz_model_z(timeout_sec: float) -> tuple[bool, float, str]:
    try:
        r = subprocess.run(
            ["gz", "model", "-m", "marble_hd2", "-p"],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
        text = (r.stdout or "") + (r.stderr or "")
        m = re.search(
            r"Pose:\s*\[([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)",
            text,
        )
        if m:
            z = float(m.group(3))
            return True, z, f"gz model pose z={z:.3f}"
        return False, 0.0, "could not parse gz model pose"
    except subprocess.TimeoutExpired:
        return False, 0.0, "gz model -p timeout"
    except FileNotFoundError:
        return False, 0.0, "gz CLI not found"


def run_live(
    sample_sec: float,
    min_motor_omega: float,
    min_dz: float,
    spawn_z: float,
) -> LiftReport:
    report = LiftReport(mode="live")
    try:
        import rclpy
        from forest_hybrid_msgs.msg import HybridTransitionStatus
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as e:
        report.add("ros_import", False, str(e))
        return report

    try:
        from actuator_msgs.msg import Actuators
    except ImportError:
        Actuators = None  # type: ignore

    class Sampler(Node):
        def __init__(self) -> None:
            super().__init__("hybrid_aerial_lift_diagnostic")
            self.status: HybridTransitionStatus | None = None
            self.js: JointState | None = None
            self.motor_omegas: list[float] = []
            self.create_subscription(
                HybridTransitionStatus,
                "/forest_gen/hybrid/transition_status",
                self._on_status,
                10,
            )
            self.create_subscription(
                JointState, "/forest_gen/hybrid/joint_states", self._on_js, 10
            )
            if Actuators is not None:
                self.create_subscription(
                    Actuators, "/forest_gen/hybrid/motor_speed", self._on_motor, 10
                )

        def _on_status(self, msg: HybridTransitionStatus) -> None:
            self.status = msg

        def _on_js(self, msg: JointState) -> None:
            self.js = msg

        def _on_motor(self, msg) -> None:
            if msg.velocity:
                self.motor_omegas = [float(v) for v in msg.velocity]

    rclpy.init()
    node = Sampler()
    z_samples: list[float] = []
    t_end = time.monotonic() + sample_sec
    try:
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.status is not None:
                z_samples.append(float(node.status.base_z_m))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    st = node.status
    if st is None:
        report.add(
            "transition_status",
            False,
            "no /forest_gen/hybrid/transition_status",
            hint="forest up sim-hybrid-test -d",
        )
        return report

    report.add(
        "fsm_aerial",
        st.state_name in ("AERIAL_FLY", "AERIAL_HOVER", "AERIAL_READY"),
        f"state={st.state_name} detail={st.detail}",
    )
    report.add(
        "tracks_yaw",
        abs(st.left_track_yaw_rad - LEFT_AERIAL) <= 0.12
        and abs(st.right_track_yaw_rad - RIGHT_AERIAL) <= 0.12,
        f"L={math.degrees(st.left_track_yaw_rad):.1f}° "
        f"R={math.degrees(st.right_track_yaw_rad):.1f}°",
    )

    mean_w = (
        sum(node.motor_omegas) / len(node.motor_omegas) if node.motor_omegas else 0.0
    )
    report.add(
        "motor_speed_ros",
        mean_w >= min_motor_omega,
        f"mean motor ω≈{mean_w:.0f} rad/s (need ≥{min_motor_omega:.0f})",
        hint="Se 0: hybrid_aerial_motor_controller gated ou FSM não em AERIAL_*.",
    )

    dz = (max(z_samples) - min(z_samples)) if z_samples else 0.0
    z_last = z_samples[-1] if z_samples else float(st.base_z_m)
    report.add(
        "base_z_rising",
        dz >= min_dz or z_last >= spawn_z + 0.12,
        f"z samples min={min(z_samples) if z_samples else z_last:.3f} "
        f"max={max(z_samples) if z_samples else z_last:.3f} Δ={dz:.3f} "
        f"(need Δ≥{min_dz} or z≥{spawn_z + 0.12:.2f})",
        hint="Sem subida: empuxo não chega ao base_link (Gazebo plugin) ou preso ao chão.",
    )
    report.add(
        "airborne_flag",
        st.airborne,
        f"airborne={st.airborne} base_z={st.base_z_m:.3f} threshold logic in FSM",
    )

    gz_ok, gz_detail = _gz_topic_echo_sample(MOTOR_TOPIC, 4.0)
    report.add(
        "gz_motor_topic",
        gz_ok,
        gz_detail,
        hint="Se falha: gz.transport não chega ao sim — verifica PLAY e um único gz sim.",
    )

    imu_ok, gyro_z, imu_detail = _parse_imu_gyro_max(min(6.0, sample_sec))
    if imu_ok:
        report.add(
            "imu_spin_stable",
            gyro_z < 8.0,
            imu_detail,
            hint="|gyro| grande → empuxo horizontal (link +Z errado) ou momento dos rotores.",
        )
    else:
        report.add("imu_spin_stable", False, imu_detail)

    pose_ok, gz_z, pose_detail = _gz_model_z(4.0)
    if pose_ok:
        report.add(
            "gz_model_z",
            gz_z >= spawn_z + 0.08,
            pose_detail + f" (spawn≈{spawn_z})",
            hint="Compara com marble_pose_from_gz se divergir.",
        )
    else:
        report.add("gz_model_z", False, pose_detail)

    # Joint-level yaw vs status
    if node.js is not None:
        l_js = r_js = None
        for name, pos in zip(node.js.name, node.js.position):
            if "left_track_yaw_joint" in name:
                l_js = float(pos)
            elif "right_track_yaw_joint" in name:
                r_js = float(pos)
        if l_js is not None and r_js is not None:
            report.add(
                "joint_states_yaw",
                abs(l_js - LEFT_AERIAL) <= 0.12 and abs(r_js - RIGHT_AERIAL) <= 0.12,
                f"JS L={math.degrees(l_js):.1f}° R={math.degrees(r_js):.1f}°",
            )
            mismatch = (
                abs(l_js - st.left_track_yaw_rad) > 0.05
                or abs(r_js - st.right_track_yaw_rad) > 0.05
            )
            report.add(
                "status_matches_joint_states",
                not mismatch,
                f"status L/R={st.left_track_yaw_rad:.3f}/{st.right_track_yaw_rad:.3f} "
                f"js {l_js:.3f}/{r_js:.3f}",
            )

    return report


def _print_report(report: LiftReport) -> None:
    print(f"\n=== hybrid aerial lift diagnostic ({report.mode}) ===\n")
    for c in report.checks:
        tag = "PASS" if c.ok else "FAIL"
        print(f"  [{tag}] {c.name}: {c.detail}")
        if not c.ok and c.hint:
            print(f"         → {c.hint}")
    print("")
    if report.passed:
        print("OVERALL: PASS")
    else:
        fails = [c.name for c in report.checks if not c.ok]
        print(f"OVERALL: FAIL ({len(fails)} check(s): {', '.join(fails)})")


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Diagnose hybrid robot no-lift in AERIAL_FLY")
    p.add_argument("--mode", choices=("offline", "live", "all"), default="all")
    p.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    p.add_argument("--mass-kg", type=float, default=5.84)
    p.add_argument("--motor-k", type=float, default=1.0e-4)
    p.add_argument("--hover-omega", type=float, default=0.0)
    p.add_argument("--live-sample-sec", type=float, default=12.0)
    p.add_argument("--min-motor-omega", type=float, default=250.0)
    p.add_argument("--min-dz", type=float, default=0.04)
    p.add_argument("--spawn-z", type=float, default=0.35)
    args = p.parse_args(list(argv) if argv is not None else None)

    reports: list[LiftReport] = []
    if args.mode in ("offline", "all"):
        reports.append(
            run_offline(args.sdf, args.mass_kg, args.motor_k, args.hover_omega)
        )
    if args.mode in ("live", "all"):
        reports.append(
            run_live(
                args.live_sample_sec,
                args.min_motor_omega,
                args.min_dz,
                args.spawn_z,
            )
        )

    for r in reports:
        _print_report(r)

    return 0 if all(r.passed for r in reports) else 1


if __name__ == "__main__":
    sys.exit(main())

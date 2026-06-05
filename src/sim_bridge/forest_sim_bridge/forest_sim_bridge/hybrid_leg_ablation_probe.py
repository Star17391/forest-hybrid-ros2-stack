#!/usr/bin/env python3
"""Regista evidências para ablação das pernas (testes 1–3).

Dispara ``to_aerial``, amostra ``transition_status`` e ``pose_fused``, detecta
queda (Δz negativo na fase de pernas), runaway (z alto) e lift (AERIAL_FLY).

Uso (stack já up com perfil de ablação):
  ros2 run forest_sim_bridge hybrid_leg_ablation_probe --experiment baseline
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

import rclpy
from forest_hybrid_msgs.msg import HybridTransitionStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


@dataclass
class Sample:
    t: float
    state: str
    base_z: float
    leg_ext: float
    pose_z: float | None
    airborne: bool


@dataclass
class AblationReport:
    experiment: str
    samples: list[Sample] = field(default_factory=list)
    states_seen: set[str] = field(default_factory=set)

    @property
    def z_pose_min(self) -> float | None:
        vals = [s.pose_z for s in self.samples if s.pose_z is not None]
        return min(vals) if vals else None

    @property
    def z_pose_max(self) -> float | None:
        vals = [s.pose_z for s in self.samples if s.pose_z is not None]
        return max(vals) if vals else None

    @property
    def z_base_max(self) -> float:
        return max((s.base_z for s in self.samples), default=0.0)

    @staticmethod
    def _z(s: Sample) -> float:
        return s.pose_z if s.pose_z is not None else s.base_z

    def fall_during_legs(self) -> tuple[bool, str]:
        leg_samples = [
            s
            for s in self.samples
            if s.state in ("LEGS_EXTENDING", "TRACKS_ROTATING", "TRANSITION_LOCK")
        ]
        if len(leg_samples) < 2:
            return False, "insufficient samples in pre-fly states"
        z0 = self._z(leg_samples[0])
        z_min = min(self._z(s) for s in leg_samples)
        drop = z0 - z_min
        if drop >= 0.08:
            return True, f"base_z drop {drop:.3f} m (from {z0:.3f} to {z_min:.3f})"
        return False, f"base_z drop {drop:.3f} m (< 0.08 m threshold)"

    def runaway(self, z_limit: float = 10.0) -> tuple[bool, str]:
        zm = self.z_base_max
        zp = self.z_pose_max
        if zm >= z_limit:
            return True, f"base_z_m max={zm:.2f} m"
        if zp is not None and zp >= z_limit:
            return True, f"pose_fused z max={zp:.2f} m"
        return False, f"max base_z={zm:.2f} pose_z={zp}"

    def reached_aerial_fly(self) -> bool:
        return "AERIAL_FLY" in self.states_seen or "AERIAL_HOVER" in self.states_seen


class HybridLegAblationProbe(Node):
    def __init__(self, experiment: str, duration_sec: float, fall_z_drop: float) -> None:
        super().__init__("hybrid_leg_ablation_probe")
        self._experiment = experiment
        self._duration = duration_sec
        self._fall_drop = fall_z_drop
        self._report = AblationReport(experiment=experiment)
        self._last_pose_z: float | None = None
        self._last_leg_cmd: float | None = None
        self._t0 = time.monotonic()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_req = self.create_publisher(
            String, "/forest_gen/hybrid/transition_request", qos
        )
        self.create_subscription(
            Float64, "/forest_gen/hybrid/support_leg_fl_cmd", self._on_leg_cmd, 10
        )
        self.create_subscription(
            HybridTransitionStatus,
            "/forest_gen/hybrid/transition_status",
            self._on_status,
            10,
        )
        self.create_subscription(PoseStamped, "/state/pose_fused", self._on_pose, 10)

    def _on_pose(self, msg: PoseStamped) -> None:
        self._last_pose_z = float(msg.pose.position.z)

    def _on_leg_cmd(self, msg: Float64) -> None:
        self._last_leg_cmd = float(msg.data)

    def _on_status(self, msg: HybridTransitionStatus) -> None:
        self._report.states_seen.add(msg.state_name)
        self._report.samples.append(
            Sample(
                t=time.monotonic() - self._t0,
                state=msg.state_name,
                base_z=float(msg.base_z_m),
                leg_ext=float(msg.leg_extension_m),
                pose_z=self._last_pose_z,
                airborne=bool(msg.airborne),
            )
        )

    def _wait_for_fsm(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            if "hybrid_transition_manager" in self.get_node_names():
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _wait_transition_subscribers(self, timeout_sec: float) -> int:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            n = self._pub_req.get_subscription_count()
            if n > 0:
                return n
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._pub_req.get_subscription_count()

    def _fire_to_aerial(self, attempts: int = 5) -> None:
        req = String()
        req.data = "to_aerial"
        for i in range(attempts):
            self._pub_req.publish(req)
            self.get_logger().info(f"Published to_aerial ({i + 1}/{attempts})")
            for _ in range(4):
                rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> int:
        print(f"\n=== hybrid leg ablation: {self._experiment} ===\n")

        if not self._wait_for_fsm(15.0):
            print("  [FAIL] /hybrid_transition_manager não está no grafo ROS")
            print("  Ver sim.log — stack pode ter morrido (gz/bridge SIGINT)")
            return 1

        subs = self._wait_transition_subscribers(10.0)
        print(f"  transition_request subscribers: {subs}")
        if subs < 1:
            print("  [FAIL] FSM não subscreveu transition_request — to_aerial não chega")
            return 1

        self._fire_to_aerial()

        end = time.monotonic() + self._duration
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        fall, fall_detail = self._report.fall_during_legs()
        rw, rw_detail = self._report.runaway()
        fly = self._report.reached_aerial_fly()

        print(f"  states: {', '.join(sorted(self._report.states_seen))}")
        print(f"  samples: {len(self._report.samples)}")
        print(f"  fall_pre_fly: {fall} — {fall_detail}")
        print(f"  runaway: {rw} — {rw_detail}")
        print(f"  aerial_fly: {fly}")
        if self._report.z_pose_min is not None:
            print(
                f"  pose_fused z: min={self._report.z_pose_min:.3f} "
                f"max={self._report.z_pose_max:.3f}"
            )
        print(f"  base_z_m max: {self._report.z_base_max:.3f}")
        if self._last_leg_cmd is not None:
            print(f"  last support_leg_fl_cmd: {self._last_leg_cmd:.3f} m")
        else:
            print("  last support_leg_fl_cmd: (nunca recebido — pernas não comandadas?)")

        print("\n  Timeline (state, t, base_z, leg_ext, pose_z):")
        last_state = ""
        for s in self._report.samples:
            if s.state != last_state:
                pz = f"{s.pose_z:.3f}" if s.pose_z is not None else "?"
                print(
                    f"    {s.t:6.1f}s {s.state:18s} base_z={s.base_z:.3f} "
                    f"leg={s.leg_ext:.3f} pose_z={pz}"
                )
                last_state = s.state

        print("\n  Interpretação:")
        if fall:
            print("    • Queda detectada na fase pernas/pré-voo (métrica base_z).")
        else:
            print("    • Sem queda forte na fase pernas (métrica base_z).")
        if rw:
            print("    • Runaway vertical — investigar motores/FSM cmd_z em seguida.")
        print("")
        return 0


def main() -> None:
    p = argparse.ArgumentParser(description="Hybrid leg ablation evidence recorder")
    p.add_argument(
        "--experiment",
        required=True,
        help="baseline | test1_no_leg_deploy | test2_slow_fsm | test3_no_leg_cmds",
    )
    p.add_argument("--duration", type=float, default=45.0)
    p.add_argument("--fall-z-drop", type=float, default=0.08)
    args = p.parse_args()

    rclpy.init()
    node = HybridLegAblationProbe(args.experiment, args.duration, args.fall_z_drop)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()

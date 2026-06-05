#!/usr/bin/env python3
"""Diagnóstico de runaway vertical: quem comanda motores e vz durante AERIAL_FLY.

Requer stack em AERIAL_FLY (ou dispara to_aerial e espera).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

import rclpy
from forest_hybrid_msgs.msg import HybridTransitionStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

MOTOR_TOPIC = "/marble_hd2/gazebo/command/motor_speed"


def _gz_motor_sample(timeout_sec: float = 3.0) -> str:
    try:
        r = subprocess.run(
            [
                "gz",
                "topic",
                "-e",
                "-t",
                MOTOR_TOPIC,
                "-n",
                "1",
            ],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
        return (r.stdout or "") + (r.stderr or "")
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return str(e)


class HybridRunawayProbe(Node):
    def __init__(self, sample_sec: float, wait_aerial_sec: float) -> None:
        super().__init__("hybrid_runaway_probe")
        self._sample_sec = sample_sec
        self._wait_aerial = wait_aerial_sec
        self._status: HybridTransitionStatus | None = None
        self._aerial_cmd: Twist | None = None
        self._pub_req = self.create_publisher(
            String, "/forest_gen/hybrid/transition_request", 10
        )
        self.create_subscription(
            HybridTransitionStatus,
            "/forest_gen/hybrid/transition_status",
            self._on_status,
            10,
        )
        self.create_subscription(
            Twist, "/forest_gen/hybrid/aerial_cmd_vel", self._on_aerial_cmd, 10
        )

    def _on_status(self, msg: HybridTransitionStatus) -> None:
        self._status = msg

    def _on_aerial_cmd(self, msg: Twist) -> None:
        self._aerial_cmd = msg

    def run(self) -> int:
        print("\n=== hybrid runaway probe ===\n")
        req = String()
        req.data = "to_aerial"
        self._pub_req.publish(req)

        deadline = time.monotonic() + self._wait_aerial
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._status and self._status.state_name in (
                "AERIAL_FLY",
                "AERIAL_HOVER",
            ):
                break

        if not self._status:
            print("  [FAIL] no transition_status")
            return 1

        print(
            f"  FSM: {self._status.state_name} base_z_m={self._status.base_z_m:.3f} "
            f"airborne={self._status.airborne} "
            f"multicopter_enabled={self._status.multicopter_enabled}"
        )

        z_max = self._status.base_z_m
        t_end = time.monotonic() + self._sample_sec
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._status:
                z_max = max(z_max, float(self._status.base_z_m))

        cmd = self._aerial_cmd
        if cmd:
            print(
                f"  aerial_cmd_vel: vx={cmd.linear.x:.3f} vy={cmd.linear.y:.3f} "
                f"vz={cmd.linear.z:.3f} az={cmd.angular.z:.3f}"
            )
        else:
            print("  aerial_cmd_vel: (no sample)")

        print(
            "\n  Comandantes de empuxo esperados:"
            "\n    • hybrid_aerial_motor_controller → gz.transport motor_speed"
            "\n    • hybrid_transition_manager → aerial_cmd_vel (vz fly_up em AERIAL_FLY)"
            "\n    • MulticopterMotorModel SDF plugins (se ω>0 no tópico)"
            f"\n    • stock Lee: {'ON' if self._status.multicopter_enabled else 'OFF'}"
        )

        gz_out = _gz_motor_sample()
        omegas: list[float] = []
        for line in gz_out.splitlines():
            line = line.strip()
            if line.startswith("velocity:"):
                try:
                    omegas.append(float(line.split(":", 1)[1].strip()))
                except ValueError:
                    pass
        if omegas:
            w_max = max(omegas)
            w_avg = sum(omegas) / len(omegas)
            print(f"\n  gz {MOTOR_TOPIC}: velocities={omegas}")
            print(f"    max ω={w_max:.1f} rad/s (hover ref ~338)")
            if w_max > 500:
                print("    [WARN] ω muito alta — risco runaway / TWR>>1")
        else:
            print(f"\n  gz motor sample: (empty or timeout)\n    {gz_out[:200]}")

        print(f"\n  base_z_m max durante {self._sample_sec:.0f}s: {z_max:.3f}")
        if z_max >= 10.0:
            print("  [FAIL] runaway vertical (base_z_m >= 10 m)")
            return 1
        print("  [PASS] sem runaway na métrica base_z_m (limite 10 m)")
        return 0


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--sample-sec", type=float, default=20.0)
    p.add_argument("--wait-aerial-sec", type=float, default=30.0)
    args = p.parse_args()
    rclpy.init()
    node = HybridRunawayProbe(args.sample_sec, args.wait_aerial_sec)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()

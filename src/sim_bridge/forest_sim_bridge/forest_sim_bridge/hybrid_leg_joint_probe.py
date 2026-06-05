#!/usr/bin/env python3
"""Amostra joint_states das pernas + cmd durante to_aerial (evidência física vs FSM)."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from forest_hybrid_msgs.msg import HybridTransitionStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class HybridLegJointProbe(Node):
    def __init__(self, duration_sec: float) -> None:
        super().__init__("hybrid_leg_joint_probe")
        self._t_end = time.monotonic() + duration_sec
        self._state = ""
        self._leg_js: list[tuple[str, float]] = []
        self._last_cmd: float | None = None
        self.create_subscription(
            HybridTransitionStatus,
            "/forest_gen/hybrid/transition_status",
            self._on_status,
            10,
        )
        self.create_subscription(JointState, "/forest_gen/hybrid/joint_states", self._on_js, 10)
        self.create_subscription(
            Float64, "/forest_gen/hybrid/support_leg_fl_cmd", self._on_cmd, 10
        )
        self._pub = self.create_publisher(
            String, "/forest_gen/hybrid/transition_request", 10
        )

    def _on_status(self, msg: HybridTransitionStatus) -> None:
        if msg.state_name != self._state:
            self._state = msg.state_name
            legs = ", ".join(f"{n}={p:.3f}" for n, p in self._leg_js) if self._leg_js else "?"
            cmd = f"{self._last_cmd:.3f}" if self._last_cmd is not None else "?"
            print(f"  → {self._state}  leg_cmd={cmd}  joints: {legs}")

    def _on_cmd(self, msg: Float64) -> None:
        self._last_cmd = float(msg.data)

    def _on_js(self, msg: JointState) -> None:
        self._leg_js = []
        for name, pos in zip(msg.name, msg.position):
            if "support_leg" in name:
                self._leg_js.append((name, float(pos)))

    def run(self) -> int:
        print("\n=== hybrid leg joint probe ===\n")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        req = String()
        req.data = "to_aerial"
        for _ in range(5):
            self._pub.publish(req)
            rclpy.spin_once(self, timeout_sec=0.05)

        while time.monotonic() < self._t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return 0


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=20.0)
    args = p.parse_args()
    rclpy.init()
    node = HybridLegJointProbe(args.duration)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

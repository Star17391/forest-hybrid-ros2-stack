#!/usr/bin/env python3
"""Audita fonte da pose no RViz vs Gazebo (sem adivinhar frame_id).

Pose_V via ros_gz_bridge não preenche child_frame_id — testa o picker estável
e /state/pose_fused durante voo.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from forest_sim_bridge.gz_world_tf_pick import GzMarblePosePicker


class HybridPoseSourceAudit(Node):
    def __init__(self, sample_sec: float) -> None:
        super().__init__("hybrid_pose_source_audit")
        self._t_end = time.monotonic() + sample_sec
        self._picker = GzMarblePosePicker("marble_hd2", "marble_hd2/base_link")
        self._fused_prev_yaw: float | None = None
        self._fused_spin_peak = 0.0
        self._fused_z: list[float] = []
        self._gz_labels: set[str] = set()

        self.create_subscription(TFMessage, "/forest_gen/gz/world_tf", self._on_dyn, 10)
        self.create_subscription(TFMessage, "/forest_gen/gz/world_tf_full", self._on_full, 10)
        self.create_subscription(PoseStamped, "/state/pose_fused", self._on_fused, 10)

    def _on_gz(self, msg: TFMessage, label: str) -> None:
        picked = self._picker.pick_xy_yaw(msg)
        if picked is not None:
            self._gz_labels.add(f"{label}:{self._picker.last_label}")

    def _on_dyn(self, msg: TFMessage) -> None:
        self._on_gz(msg, "dynamic")

    def _on_full(self, msg: TFMessage) -> None:
        self._on_gz(msg, "full")
        named = sum(1 for t in msg.transforms if t.child_frame_id)
        if named == 0 and len(msg.transforms) > 0:
            self._gz_labels.add(f"full:{len(msg.transforms)}_unnamed")

    def _on_fused(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if self._fused_prev_yaw is not None:
            dy = yaw - self._fused_prev_yaw
            while dy > math.pi:
                dy -= 2.0 * math.pi
            while dy < -math.pi:
                dy += 2.0 * math.pi
            self._fused_spin_peak = max(self._fused_spin_peak, abs(dy) / 0.05)
        self._fused_prev_yaw = yaw
        self._fused_z.append(float(msg.pose.position.z))

    def run(self) -> int:
        while time.monotonic() < self._t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

        print("\n=== hybrid pose source audit ===\n")
        ok = True

        if not self._gz_labels:
            print("  [FAIL] no Gazebo TF samples")
            return 1

        for line in sorted(self._gz_labels):
            print(f"  gz picker: {line}")

        if self._fused_z:
            z_min, z_max = min(self._fused_z), max(self._fused_z)
            print(
                f"  pose_fused z: min={z_min:.3f} max={z_max:.3f} "
                f"peak_yaw_rate≈{self._fused_spin_peak:.1f} rad/s"
            )
            if self._fused_spin_peak > 5.0:
                print(
                    "  [FAIL] pose_fused gira como pião — índice errado em Pose_V "
                    "(hélice?)"
                )
                ok = False
            else:
                print("  [PASS] pose_fused estável em yaw")
        else:
            print("  [FAIL] no /state/pose_fused")
            ok = False

        print(
            "\n  Nota: ros_gz_bridge Pose_V não traz child_frame_id; "
            "marble_pose_from_gz usa índice estável (z≈spawn, baixo spin)."
        )
        print("\nOVERALL:", "PASS" if ok else "FAIL")
        return 0 if ok else 1


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--sample-sec", type=float, default=8.0)
    args = p.parse_args()
    rclpy.init()
    node = HybridPoseSourceAudit(args.sample_sec)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Check EKF TF health: detect NaN odom->base_link and static sensor frames."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class EkfTfHealth(Node):
    def __init__(self, duration: float) -> None:
        super().__init__("ekf_tf_health")
        self._duration = duration
        self._t0 = time.monotonic()
        self._nan_count = 0
        self._ok_count = 0
        self._frames_seen: set[str] = set()
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, 10)

    def _check_transform(self, t) -> bool:
        tr = t.transform.translation
        rot = t.transform.rotation
        vals = [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w]
        if not all(math.isfinite(v) for v in vals):
            return False
        n = rot.x * rot.x + rot.y * rot.y + rot.z * rot.z + rot.w * rot.w
        return n > 1e-6

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            child = t.child_frame_id
            parent = t.header.frame_id
            self._frames_seen.add(f"{parent} -> {child}")
            if child == "marble_hd2/base_link" and parent == "odom":
                if self._check_transform(t):
                    self._ok_count += 1
                else:
                    self._nan_count += 1

    def _on_tf_static(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._frames_seen.add(f"{t.header.frame_id} -> {t.child_frame_id}")

    def run(self) -> int:
        while time.monotonic() - self._t0 < self._duration:
            rclpy.spin_once(self, timeout_sec=0.2)
        print(f"\n=== EKF TF health ({self._duration:.0f}s) ===")
        print(f"  odom -> marble_hd2/base_link OK:   {self._ok_count}")
        print(f"  odom -> marble_hd2/base_link NaN:   {self._nan_count}")
        if self._nan_count > 0:
            print("  VERDICT: EKF publishing invalid TF — fix wheel odom / EKF inputs")
            return 1
        if self._ok_count == 0:
            print("  VERDICT: No valid odom->base_link seen (stack not ready?)")
            return 2
        print("  VERDICT: TF chain OK for base_link")
        for f in sorted(self._frames_seen):
            if "marble" in f or "laser" in f or "odom" in f or "map" in f:
                print(f"    {f}")
        return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=10.0)
    args = p.parse_args()
    rclpy.init()
    node = EkfTfHealth(args.duration)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Mostra z de cada índice em world_tf_full — prova se o latch (ex. 16) segue o modelo."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class HybridPoseIndicesProbe(Node):
    def __init__(self, sample_sec: float, latch_hint: int | None) -> None:
        super().__init__("hybrid_pose_indices_probe")
        self._t_end = time.monotonic() + sample_sec
        self._latch = latch_hint
        self._by_idx: dict[int, list[float]] = {}
        self.create_subscription(
            TFMessage, "/forest_gen/gz/world_tf_full", self._on_full, 10
        )

    def _on_full(self, msg: TFMessage) -> None:
        for idx, tf in enumerate(msg.transforms):
            z = float(tf.transform.translation.z)
            self._by_idx.setdefault(idx, []).append(z)

    def run(self) -> int:
        while time.monotonic() < self._t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        print("\n=== hybrid pose indices (world_tf_full z) ===\n")
        if not self._by_idx:
            print("  [FAIL] no TFMessage samples")
            return 1

        for idx in sorted(self._by_idx):
            zs = self._by_idx[idx]
            z_min, z_max = min(zs), max(zs)
            dz = z_max - z_min
            mark = " <-- latch?" if self._latch is not None and idx == self._latch else ""
            flag = " STALE" if dz < 0.02 and z_max > 1.0 else ""
            if dz >= 0.5:
                flag = " MOVES"
            print(
                f"  index[{idx:2d}]: z min={z_min:8.3f} max={z_max:8.3f} "
                f"Δ={dz:.3f} n={len(zs)}{mark}{flag}"
            )

        print(
            "\n  Se o índice latch tem Δ≈0 mas outro índice tem MOVES durante voo,"
            "\n  pose_fused/FSM usam a entidade errada (prova de desacoplamento)."
        )
        return 0


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--sample-sec", type=float, default=15.0)
    p.add_argument("--latch-index", type=int, default=16)
    args = p.parse_args()
    rclpy.init()
    node = HybridPoseIndicesProbe(args.sample_sec, args.latch_index)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()

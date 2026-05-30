#!/usr/bin/env python3
"""Audita a árvore TF esperada (sim MARBLE).

Usa QoS TRANSIENT_LOCAL em /tf_static (senão perde map->odom publicado no arranque)
e confirma com tf2 lookup no fim.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


REQUIRED_STATIC = [
    ("map", "odom"),
    ("marble_hd2/base_link", "laser"),
]

REQUIRED_DYNAMIC = [
    ("odom", "marble_hd2/base_link"),
]

TF_STATIC_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class TfFrameAudit(Node):
    def __init__(self, duration: float) -> None:
        from rclpy.parameter import Parameter

        super().__init__(
            "tf_frame_audit",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()
        self._static: set[tuple[str, str]] = set()
        self._dynamic: set[tuple[str, str]] = set()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            TFMessage, "/tf_static", self._on_static, TF_STATIC_QOS
        )
        self.create_subscription(TFMessage, "/tf", self._on_dynamic, 50)

    def _on_static(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._static.add((t.header.frame_id, t.child_frame_id))

    def _on_dynamic(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._dynamic.add((t.header.frame_id, t.child_frame_id))

    def _tf2_can_transform(self, parent: str, child: str) -> bool:
        try:
            if self._tf_buffer.can_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            ):
                return True
        except Exception:
            pass
        return False

    def _report(self) -> int:
        print(f"\n=== tf_frame_audit ({self._duration:.0f}s) ===")
        print("Static transforms seen (/tf_static, TRANSIENT_LOCAL):")
        for p, c in sorted(self._static):
            print(f"  {p} -> {c}")

        print("Dynamic transforms seen (/tf, sample):")
        for p, c in sorted(self._dynamic):
            if "marble" in p or "marble" in c or p in ("odom", "map") or c == "laser":
                print(f"  {p} -> {c}")

        print("tf2 lookup (authoritative):")
        checks = [
            ("map", "odom"),
            ("odom", "marble_hd2/base_link"),
            ("marble_hd2/base_link", "laser"),
        ]
        tf2_ok: dict[tuple[str, str], bool] = {}
        for parent, child in checks:
            ok = self._tf2_can_transform(parent, child)
            tf2_ok[(parent, child)] = ok
            status = "OK" if ok else "FAIL"
            print(f"  {parent} -> {child}: {status}")

        bad = []
        for p, c in REQUIRED_STATIC:
            if (p, c) not in self._static and not tf2_ok.get((p, c), False):
                bad.append(f"MISSING: {p} -> {c}")
        if not any(
            p == "odom" and c == "marble_hd2/base_link" for p, c in self._dynamic
        ) and not tf2_ok.get(("odom", "marble_hd2/base_link"), False):
            bad.append("MISSING: odom -> marble_hd2/base_link (EKF?)")

        if ("base_link", "laser") in self._static:
            print(
                "WARN: static base_link -> laser (extrinsics YAML não carregou; "
                "esperado marble_hd2/base_link -> laser)"
            )

        for b in bad:
            print(f"ERRO: {b}", file=sys.stderr)

        if bad:
            print("\nCadeia esperada: map -> odom -> marble_hd2/base_link -> laser")
            return 1

        if ("map", "odom") not in self._static and tf2_ok.get(("map", "odom")):
            print(
                "NOTA: map->odom visível no tf2 mas não na lista /tf_static — "
                "normal com static_transform_publisher + subscrição tardia."
            )
        print("\nOK: cadeia TF essencial válida (tf2)")
        return 0

    def run(self) -> int:
        while time.monotonic() - self._t0 < self._duration:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._report()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=12.0)
    args = p.parse_args()
    rclpy.init()
    node = TfFrameAudit(args.duration)
    try:
        sys.exit(node.run())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

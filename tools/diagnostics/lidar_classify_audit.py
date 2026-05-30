#!/usr/bin/env python3
"""Auditoria Fase 1: distribuição de labels em /perception/lidar/points_labeled."""

from __future__ import annotations

import argparse
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


LABELS = {
    0: "invalid",
    1: "ground",
    2: "other",
    3: "hole",
    4: "obstacle",
}


class LidarClassifyAudit(Node):
    def __init__(self, duration: float) -> None:
        from rclpy.parameter import Parameter

        super().__init__(
            "lidar_classify_audit",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()
        self._counts = {k: 0 for k in LABELS}
        self._msgs = 0

        self.create_subscription(
            PointCloud2,
            "/perception/lidar/points_labeled",
            self._on_cloud,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._msgs += 1
        for label in _iter_labels(msg):
            self._counts[label] = self._counts.get(label, 0) + 1
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            raise SystemExit(0)

    def _report(self) -> None:
        total = sum(self._counts.values())
        invalid = self._counts.get(0, 0)
        classified = total - invalid
        print(f"\n=== lidar_classify_audit ({self._duration:.0f}s) ===")
        print(f"Messages: {self._msgs}  points: {total}  (classified beams: {classified})")
        if total == 0:
            print("ERRO: sem pontos — sim up + classify node?", file=sys.stderr)
            return
        if classified == 0:
            print(
                "ERRO: 100% label=invalid — reinicia sim após rebuild forest_lidar_preprocess_cpp",
                file=sys.stderr,
            )
            return
        for lid, name in LABELS.items():
            c = self._counts.get(lid, 0)
            pct_all = 100.0 * c / total
            pct_cls = 100.0 * c / classified if classified else 0.0
            if lid != 0:
                print(
                    f"  label {lid} ({name:8s}): {c:6d}  "
                    f"({pct_all:5.1f}% all, {pct_cls:5.1f}% of classified)"
                )
            else:
                print(f"  label {lid} ({name:8s}): {c:6d}  ({pct_all:5.1f}% all)")
        ground = self._counts.get(1, 0)
        hazard = self._counts.get(3, 0) + self._counts.get(4, 0)
        other = self._counts.get(2, 0)
        print(f"\n  ground / classified: {100.0 * ground / classified:.1f}%")
        print(f"  hazard / classified: {100.0 * hazard / classified:.1f}% (hole+obstacle)")
        print(f"  other / classified:  {100.0 * other / classified:.1f}%")


def _iter_labels(msg: PointCloud2):
    label_offset = None
    point_step = msg.point_step
    for field in msg.fields:
        if field.name == "label":
            label_offset = field.offset
            break
    if label_offset is None:
        return
    data = msg.data
    for i in range(msg.width):
        off = i * point_step + label_offset
        yield data[off]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=10.0)
    args = p.parse_args()
    rclpy.init()
    node = LidarClassifyAudit(args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

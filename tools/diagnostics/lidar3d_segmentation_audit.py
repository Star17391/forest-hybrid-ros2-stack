#!/usr/bin/env python3
"""Audit Fase 1 — 3D segmentation output quality.

Reports per-frame: n_ground, n_trunks, n_obstacles, n_total, ratios.
Aggregates over duration and gives a PASS/FAIL verdict.

Pré-requisito: sim lidar3d com forest_3d_perception a correr.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


@dataclass
class Stats:
    count: int = 0
    total_pts: int = 0


class SegAudit(Node):
    def __init__(self, duration: float):
        super().__init__(
            "lidar3d_seg_audit",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()

        self._ground = Stats()
        self._trunks = Stats()
        self._obstacles = Stats()
        self._input = Stats()

        qos = qos_profile_sensor_data

        self.create_subscription(PointCloud2, "/sensors/lidar/points", self._on_input, qos)
        self.create_subscription(PointCloud2, "/perception/lidar3d/ground", self._on_ground, qos)
        self.create_subscription(PointCloud2, "/perception/lidar3d/trunks", self._on_trunks, qos)
        self.create_subscription(PointCloud2, "/perception/lidar3d/obstacles", self._on_obstacles, qos)

        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Seg audit: {duration:.0f}s...")

    def _n_pts(self, msg: PointCloud2) -> int:
        return msg.width * msg.height

    def _on_input(self, msg: PointCloud2):
        self._input.count += 1
        self._input.total_pts += self._n_pts(msg)

    def _on_ground(self, msg: PointCloud2):
        self._ground.count += 1
        self._ground.total_pts += self._n_pts(msg)

    def _on_trunks(self, msg: PointCloud2):
        self._trunks.count += 1
        self._trunks.total_pts += self._n_pts(msg)

    def _on_obstacles(self, msg: PointCloud2):
        self._obstacles.count += 1
        self._obstacles.total_pts += self._n_pts(msg)

    def _tick(self):
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            rclpy.shutdown()

    def _report(self):
        print(f"\n{'='*60}")
        print(f"  SEGMENTATION AUDIT ({self._duration:.0f}s)")
        print(f"{'='*60}")

        print(f"\n  Input clouds: {self._input.count} ({self._input.total_pts} pts total)")
        print(f"  Ground:       {self._ground.count} msgs ({self._ground.total_pts} pts)")
        print(f"  Trunks:       {self._trunks.count} msgs ({self._trunks.total_pts} pts)")
        print(f"  Obstacles:    {self._obstacles.count} msgs ({self._obstacles.total_pts} pts)")

        seg_total = self._ground.total_pts + self._trunks.total_pts + self._obstacles.total_pts
        if seg_total > 0:
            g_pct = 100 * self._ground.total_pts / seg_total
            t_pct = 100 * self._trunks.total_pts / seg_total
            o_pct = 100 * self._obstacles.total_pts / seg_total
            print(f"\n  Ratios: ground={g_pct:.1f}%  trunks={t_pct:.1f}%  obstacles={o_pct:.1f}%")

        print(f"\n  VERDICT:", end=" ")
        issues = []
        if self._input.count == 0:
            issues.append("no input clouds")
        if self._ground.count == 0 and self._input.count > 3:
            issues.append("no ground segmented")
        if seg_total > 0 and self._ground.total_pts / seg_total < 0.1:
            issues.append("ground < 10% (expected dominant on flat world)")

        if issues:
            print(f"FAIL — {'; '.join(issues)}")
            sys.exit(1)
        else:
            print("PASS")
            if seg_total == 0 and self._input.count > 0:
                print("  (Note: seg outputs empty — check if lidar3d_segmentation_node is running)")

        print(f"{'='*60}\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=30.0)
    args = p.parse_args()

    rclpy.init()
    node = SegAudit(args.duration)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Auditoria LiDAR 3D sim — /sensors/lidar/points (Airy-class gpu_lidar)."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points


class Lidar3dPipelineAudit(Node):
    def __init__(self, duration: float) -> None:
        from rclpy.parameter import Parameter

        super().__init__(
            "lidar3d_pipeline_audit",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()
        self._msgs = 0
        self._points = 0
        self._near = 0
        self._finite = 0
        self._frame = ""

        self.create_subscription(
            PointCloud2,
            "/sensors/lidar/points",
            self._on_cloud,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._msgs += 1
        self._frame = msg.header.frame_id
        for x, y, z in read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            self._points += 1
            r = math.sqrt(x * x + y * y + z * z)
            if math.isfinite(r):
                self._finite += 1
                if r <= 8.0:
                    self._near += 1
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            raise SystemExit(0)

    def _report(self) -> None:
        print(f"\n=== lidar3d_pipeline_audit ({self._duration:.0f}s) ===")
        print(f"Messages: {self._msgs}  frame: {self._frame!r}")
        print(f"Points (finite): {self._finite}  total read: {self._points}")
        print(f"Points r<=8m: {self._near}")
        ok = self._msgs >= 3 and self._finite >= 5000
        if not ok:
            print(
                "ERRO: poucos dados 3D — sim com --lidar3d, Gazebo PLAY, gpu_lidar render?",
                file=sys.stderr,
            )
            sys.exit(1)
        print("OK: nuvem 3D activa")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=25.0)
    args = parser.parse_args()
    rclpy.init()
    node = Lidar3dPipelineAudit(args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

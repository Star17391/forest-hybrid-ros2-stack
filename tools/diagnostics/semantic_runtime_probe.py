#!/usr/bin/env python3
"""Simple runtime probe for semantic mask and semantic points topics."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("semantic_runtime_probe")
        qos = rclpy.qos.qos_profile_sensor_data
        self.mask_count = 0
        self.points_count = 0
        self.t0 = time.time()
        self.create_subscription(Image, "/perception/semantic_mask", self._on_mask, qos)
        self.create_subscription(PointCloud2, "/perception/semantic_points", self._on_points, qos)
        self.timer = self.create_timer(5.0, self._report)

    def _on_mask(self, _msg: Image) -> None:
        self.mask_count += 1

    def _on_points(self, _msg: PointCloud2) -> None:
        self.points_count += 1

    def _report(self) -> None:
        dt = max(1.0e-6, time.time() - self.t0)
        self.get_logger().info(
            f"semantic_mask_hz={self.mask_count / dt:.2f} semantic_points_hz={self.points_count / dt:.2f}"
        )


def main() -> None:
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


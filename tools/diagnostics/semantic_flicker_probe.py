#!/usr/bin/env python3
"""Estimate temporal flicker of semantic mask stream."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FlickerProbe(Node):
    def __init__(self) -> None:
        super().__init__("semantic_flicker_probe")
        self.prev: np.ndarray | None = None
        self.frames = 0
        self.switch_sum = 0.0
        self.t0 = time.time()
        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Image, "/perception/semantic_mask", self._cb, qos)
        self.create_timer(5.0, self._report)

    def _cb(self, msg: Image) -> None:
        if msg.encoding != "mono8":
            return
        cur = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        if self.prev is not None and self.prev.shape == cur.shape:
            switch_ratio = float(np.count_nonzero(self.prev != cur)) / float(cur.size)
            self.switch_sum += switch_ratio
        self.prev = cur.copy()
        self.frames += 1

    def _report(self) -> None:
        dt = max(1.0e-6, time.time() - self.t0)
        avg_flicker = self.switch_sum / max(1, self.frames - 1)
        fps = self.frames / dt
        self.get_logger().info(f"semantic_fps={fps:.2f} avg_switch_ratio={avg_flicker:.4f}")


def main() -> None:
    rclpy.init()
    node = FlickerProbe()
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


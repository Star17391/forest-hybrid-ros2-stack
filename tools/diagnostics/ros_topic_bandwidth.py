#!/usr/bin/env python3
"""Measure ROS topic rate, message size, and bandwidth (requires running stack)."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def _sizeof_pointcloud2(msg) -> int:
    return len(msg.data) + 200


def _sizeof_laserscan(msg) -> int:
    return len(msg.ranges) * 4 + len(msg.intensities) * 4 + 128


class TopicStats(Node):
    def __init__(self, topics: list[str], duration: float) -> None:
        super().__init__("ros_topic_bandwidth")
        self._duration = duration
        self._t0 = time.monotonic()
        self._stats: dict[str, dict] = {}
        for topic in topics:
            self._stats[topic] = {"count": 0, "bytes": 0, "last_stamp": None}
            try:
                msg_type = self._guess_type(topic)
            except Exception as exc:
                self.get_logger().warning(f"Skip {topic}: {exc}")
                continue
            self.create_subscription(
                msg_type,
                topic,
                lambda m, t=topic: self._cb(t, m),
                qos_profile_sensor_data,
            )

    def _guess_type(self, topic: str):
        names_types = self.get_topic_names_and_types()
        for name, types in names_types:
            if name == topic and types:
                return get_message(types[0])
        raise RuntimeError(f"topic not found: {topic}")

    def _cb(self, topic: str, msg) -> None:
        st = self._stats[topic]
        st["count"] += 1
        if hasattr(msg, "data"):
            st["bytes"] += _sizeof_pointcloud2(msg)
        elif hasattr(msg, "ranges"):
            st["bytes"] += _sizeof_laserscan(msg)
        else:
            st["bytes"] += 256
        if hasattr(msg, "header"):
            st["last_stamp"] = str(msg.header.stamp.sec)

    def run(self) -> int:
        while time.monotonic() - self._t0 < self._duration:
            rclpy.spin_once(self, timeout_sec=0.2)
        elapsed = time.monotonic() - self._t0
        print(f"\n=== topic bandwidth ({elapsed:.1f}s) ===")
        print(f"{'topic':<45} {'hz':>8} {'KB/s':>10} {'avg_B':>10} {'n':>6}")
        for topic, st in sorted(self._stats.items()):
            n = st["count"]
            if n == 0:
                print(f"{topic:<45} {'—':>8} {'—':>10} {'—':>10} {0:>6}")
                continue
            hz = n / elapsed
            bps = st["bytes"] / elapsed
            avg = st["bytes"] / n
            print(f"{topic:<45} {hz:8.2f} {bps/1024:10.1f} {avg:10.0f} {n:6d}")
        return 0


DEFAULT_TOPICS = [
    "/clock",
    "/scan",
    "/sensors/lidar/scan",
    "/perception/lidar/points_labeled",
    "/perception/lidar/scan_ground",
    "/tf",
    "/tf_static",
]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--topics", nargs="*", default=DEFAULT_TOPICS)
    args = p.parse_args()
    rclpy.init()
    node = TopicStats(args.topics, args.duration)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

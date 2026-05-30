"""Helpers for diagnostics with use_sim_time."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
def header_stamp_sec(msg) -> float:
    h = msg.header.stamp
    return h.sec + h.nanosec * 1e-9


def clock_now_sec(node: Node) -> float:
    t = node.get_clock().now()
    return t.nanoseconds * 1e-9


def latency_ms(msg, node: Node) -> float:
    """Stamp → recepção no relógio ROS (sim ou wall, conforme use_sim_time)."""
    return (clock_now_sec(node) - header_stamp_sec(msg)) * 1000.0

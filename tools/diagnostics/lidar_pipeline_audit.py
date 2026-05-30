#!/usr/bin/env python3
"""Auditoria LiDAR sim — executar com Gazebo em PLAY e stack a correr.

  source install/setup.bash
  python3 scripts/diagnostics/lidar_pipeline_audit.py
"""

from __future__ import annotations

import math
import subprocess
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _run(cmd: list[str]) -> str:
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT, timeout=8)
    except Exception as exc:
        return f"(failed: {exc})"


def _analyze_scan(msg: LaserScan, label: str) -> dict:
    finite = [r for r in msg.ranges if math.isfinite(r)]
    at_max = sum(1 for r in finite if abs(r - msg.range_max) < 0.05)
    at_min = sum(1 for r in finite if abs(r - msg.range_min) < 0.05)
    mid = [r for r in finite if msg.range_min + 0.1 < r < msg.range_max - 0.1]
    # Unique values (2 cm resolution): flat line in RViz often = few distinct ranges.
    rounded = [round(r, 2) for r in finite]
    uniq = len(set(rounded))
    return {
        "label": label,
        "frame": msg.header.frame_id,
        "n": len(msg.ranges),
        "finite": len(finite),
        "at_max": at_max,
        "pct_at_max": round(100.0 * at_max / len(finite), 1) if finite else None,
        "at_min": at_min,
        "mid_varied": len(mid),
        "unique_ranges_2cm": uniq,
        "min_r": round(min(finite), 3) if finite else None,
        "max_r": round(max(finite), 3) if finite else None,
        "mean_r": round(sum(finite) / len(finite), 3) if finite else None,
    }


class _OneShot(Node):
    def __init__(self) -> None:
        super().__init__("lidar_pipeline_audit")
        self.samples: dict[str, dict] = {}
        self._done = 0
        self._targets = [
            ("/scan", "raw_gazebo_bridge"),
            ("/sensors/lidar/scan", "preprocessed"),
        ]
        for topic, _ in self._targets:
            self.create_subscription(
                LaserScan, topic, lambda m, t=topic: self._cb(m, t), qos_profile_sensor_data
            )

    def _cb(self, msg: LaserScan, topic: str) -> None:
        if topic in self.samples:
            return
        label = next(lbl for tpath, lbl in self._targets if tpath == topic)
        self.samples[topic] = _analyze_scan(msg, label)
        self._done += 1


def main() -> int:
    print("=== LiDAR pipeline audit (forest-hybrid) ===\n")

    print("--- ROS nodes (lidar-related) ---")
    print(_run(["ros2", "node", "list"]))
    print()

    print("--- Topic publishers ---")
    for t in ("/scan", "/sensors/lidar/scan", "/sensors/lidar/points", "/perception/lidar/points_labeled"):
        print(f"\n[{t}]")
        print(_run(["ros2", "topic", "info", t, "-v"]))

    print("\n--- Gazebo gz topics (laser) ---")
    gz_list = _run(["gz", "topic", "-l"])
    for line in gz_list.splitlines():
        if "laser" in line.lower() or "lidar" in line.lower() or "scan" in line.lower():
            print(line)

    print("\n--- Waiting up to 8s for LaserScan samples (PLAY + bridge) ---")
    rclpy.init()
    node = _OneShot()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0 and node._done < len(node._targets):
        rclpy.spin_once(node, timeout_sec=0.5)

    all_nan = False
    if not node.samples:
        print("NO DATA: nenhum LaserScan recebido em /scan ou /sensors/lidar/scan")
        print("Verificar: Gazebo PLAY, ros_gz_bridge, lazy bridge (RViz subscreve /scan)")
    else:
        for topic, stats in node.samples.items():
            if stats.get("finite", 0) == 0:
                all_nan = True
                print(
                    f"\n[BLOCKER] {topic}: 0 ranges finitos (só NaN). "
                    "SLAM não funciona até haver dados válidos. "
                    "Comparar /scan vs /sensors/lidar/scan; gz topic no Gazebo."
                )

    print("\n--- Scan statistics ---")
    for topic, stats in node.samples.items():
        print(f"\n{topic} ({stats['label']}):")
        for k, v in stats.items():
            if k != "label":
                print(f"  {k}: {v}")

    raw = node.samples.get("/scan")
    pre = node.samples.get("/sensors/lidar/scan")

    if raw:
        if raw["finite"] == 0:
            print("\n[CONFIRMED] /scan sem ranges finitos — sensor/bridge não entrega dados.")
        elif raw["pct_at_max"] is not None and raw["pct_at_max"] > 80:
            print(
                f"\n[CONFIRMED] {raw['pct_at_max']}% dos feixes em range_max ({raw['max_r']} m) — "
                "a 'linha' no RViz é o círculo/anel de misses, não obstáculos distantes."
            )
        elif raw["unique_ranges_2cm"] is not None and raw["unique_ranges_2cm"] <= 5:
            print(
                f"\n[CONFIRMED] Apenas {raw['unique_ranges_2cm']} distâncias distintas — "
                "scan dominado por um único tipo de hit (ex.: intersecção com o solo no plano inclinado)."
            )
        elif raw["mid_varied"] >= 20:
            print("\n[OK] /scan tem muitos alcances intermédios — obstáculos/solo variados.")

    if raw and pre:
        same_max = raw.get("pct_at_max") == pre.get("pct_at_max")
        if raw["mid_varied"] >= 20 and pre["mid_varied"] < 10:
            print(
                "\n[CONFIRMED] Regressão no preprocess: /scan OK mas /sensors/lidar/scan "
                "perde hits (laserscan_preprocess_node ou IMU leveling)."
            )
        elif same_max and raw.get("unique_ranges_2cm") == pre.get("unique_ranges_2cm"):
            print("\n[OK] /scan e /sensors/lidar/scan estatisticamente iguais — preprocess não é a causa.")
        else:
            print("\n--- /scan vs preprocess ---")
            for k in ("pct_at_max", "unique_ranges_2cm", "min_r", "mean_r"):
                print(f"  {k}: raw={raw.get(k)}  pre={pre.get(k)}")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if (not node.samples or all_nan) else 0


if __name__ == "__main__":
    sys.exit(main())

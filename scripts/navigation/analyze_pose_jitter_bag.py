#!/usr/bin/env python3
"""Analisa saltos em pose_fused vs Gazebo a partir de um rosbag de diagnóstico."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def yaw_from_q(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_series(bag: Path, topic: str, model_index: int = 1):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise SystemExit(f"Tópico ausente no bag: {topic}")

    rows = []
    while reader.has_next():
        name, data, t = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, get_message(topics[name]))
        if topic == "/state/pose_fused":
            p = msg.pose.position
            q = msg.pose.orientation
            rows.append((t * 1e-9, p.x, p.y, p.z, yaw_from_q(q.x, q.y, q.z, q.w)))
        elif topic.startswith("/forest_gen/gz/world_tf"):
            if model_index >= len(msg.transforms):
                continue
            tr = msg.transforms[model_index].transform
            p = tr.translation
            q = tr.rotation
            rows.append((t * 1e-9, p.x, p.y, p.z, yaw_from_q(q.x, q.y, q.z, q.w)))
    return rows


def jitter_stats(rows):
    if len(rows) < 2:
        return {}
    dpos = []
    dyaw = []
    for a, b in zip(rows, rows[1:]):
        dt = b[0] - a[0]
        if dt <= 0.0:
            continue
        dpos.append(math.hypot(b[1] - a[1], b[2] - a[2]) / dt)
        dyaw.append(abs(b[4] - a[4]) / dt)
    if not dpos:
        return {}
    dpos.sort()
    dyaw.sort()
    return {
        "samples": len(rows),
        "duration_s": rows[-1][0] - rows[0][0],
        "max_speed_mps": max(dpos),
        "p95_speed_mps": dpos[int(0.95 * (len(dpos) - 1))],
        "max_yaw_rate_rps": max(dyaw),
        "p95_yaw_rate_rps": dyaw[int(0.95 * (len(dyaw) - 1))],
    }


def compare(fused, gz):
    if not fused or not gz:
        return
    import bisect

    def nearest(series, t):
        ts = [r[0] for r in series]
        i = bisect.bisect_left(ts, t)
        i = min(max(i, 0), len(series) - 1)
        return series[i]

    t0 = max(fused[0][0], gz[0][0])
    t1 = min(fused[-1][0], gz[-1][0])
    print("\nAmostras alinhadas pose_fused vs Gazebo[idx]:")
    print("  t(s)   fused(x,y,yaw°)              gz(x,y,yaw°)                 |Δpos|  Δyaw°")
    worst = 0.0
    for dt in range(0, int(t1 - t0) + 1, 2):
        t = t0 + dt
        f = nearest(fused, t)
        g = nearest(gz, t)
        dpos = math.hypot(f[1] - g[1], f[2] - g[2])
        dyaw = math.degrees(abs(f[4] - g[4]))
        worst = max(worst, dpos)
        print(
            f"  {dt:4d}   ({f[1]:6.2f},{f[2]:6.2f},{math.degrees(f[4]):6.1f})"
            f"   ({g[1]:6.2f},{g[2]:6.2f},{math.degrees(g[4]):6.1f})"
            f"   {dpos:6.3f}  {dyaw:6.1f}"
        )
    print(f"\nPior |Δpos| amostrado: {worst:.3f} m")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="Pasta do rosbag mcap")
    parser.add_argument("--model-index", type=int, default=1)
    args = parser.parse_args()

    fused = load_series(args.bag, "/state/pose_fused")
    gz = load_series(args.bag, "/forest_gen/gz/world_tf_full", args.model_index)

    print(f"Bag: {args.bag}")
    for label, rows in [("pose_fused", fused), ("gazebo[idx]", gz)]:
        stats = jitter_stats(rows)
        print(f"\n{label}: {stats}")
    compare(fused, gz)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

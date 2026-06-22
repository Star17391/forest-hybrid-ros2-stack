#!/usr/bin/env python3
"""Extrai uma nuvem PointCloud2 de um bag e caracteriza-a (raio/z) ou despeja xyz.

Uso:
  extract_cloud.py <bag_dir> <topic> stats        # estatísticas radiais/z
  extract_cloud.py <bag_dir> <topic> dump <out>   # despeja "x y z" (último frame)
"""
import struct
import sys

import numpy as np
import rclpy.serialization as rs
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def last_cloud_xyz(bag_dir, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    typ = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in typ:
        return None
    cls = get_message(typ[topic])
    last = None
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t == topic:
            last = rs.deserialize_message(data, cls)
    if last is None:
        return None
    # parse XYZ (assume float32 x,y,z nos offsets do PointField)
    off = {f.name: f.offset for f in last.fields}
    step = last.point_step
    n = last.width * last.height
    buf = bytes(last.data)
    xs, ys, zs = [], [], []
    ox, oy, oz = off["x"], off["y"], off["z"]
    for i in range(n):
        b = i * step
        xs.append(struct.unpack_from("<f", buf, b + ox)[0])
        ys.append(struct.unpack_from("<f", buf, b + oy)[0])
        zs.append(struct.unpack_from("<f", buf, b + oz)[0])
    return np.array(xs), np.array(ys), np.array(zs)


def main():
    bag, topic, mode = sys.argv[1], sys.argv[2], sys.argv[3]
    r = last_cloud_xyz(bag, topic)
    if r is None:
        print("(tópico ausente ou vazio)")
        return
    x, y, z = r
    if mode == "dump":
        with open(sys.argv[4], "w") as f:
            for a, b, c in zip(x, y, z):
                f.write(f"{a:.4f} {b:.4f} {c:.4f}\n")
        print(f"despejados {len(x)} pts -> {sys.argv[4]}")
        return
    # stats
    cx, cy = np.median(x), np.median(y)
    rad = np.hypot(x - cx, y - cy)
    print(f"n={len(x)}  centro=({cx:.2f},{cy:.2f})")
    print(f"  z:    min={z.min():.2f} max={z.max():.2f} span={z.max()-z.min():.2f}")
    print(f"  raio: med={np.median(rad):.3f} p90={np.percentile(rad,90):.3f} "
          f"max={rad.max():.3f}  (DBH~2*raio)")
    # distribuição radial em bandas de z (mostra base alargada / ramos)
    print("  perfil por z:")
    for lo in np.arange(np.floor(z.min()*10)/10, z.max(), 0.30):
        m = (z >= lo) & (z < lo + 0.30)
        if m.sum() == 0:
            continue
        rr = np.hypot(x[m]-cx, y[m]-cy)
        print(f"    z[{lo:4.1f},{lo+0.3:4.1f}) n={m.sum():3d} "
              f"raio med={np.median(rr):.3f} max={rr.max():.3f}")


if __name__ == "__main__":
    main()

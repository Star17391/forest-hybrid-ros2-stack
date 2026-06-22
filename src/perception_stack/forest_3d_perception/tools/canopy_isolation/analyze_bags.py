#!/usr/bin/env python3
"""Lê os bags trunk_range_t<t>_d<d> e resume a deteção por espécie x distância.

Para cada bag (mundo de 1 árvore isolada à distância d em +X):
  - quantos frames com >=1 landmark, e nº mediano de landmarks/frame
    (deve ser 1; >1 = deteção espúria);
  - do landmark DOMINANTE (o mais próximo do x esperado = d): mediana de
    DBH, confiança, altura, e DBH stddev;
  - erro do DBH vs GT (secção do mesh, batch.gt_diameter).

Uso:  analyze_bags.py [bags_dir]
"""
import glob
import os
import re
import statistics as st
import sys

import rclpy.serialization as rs
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

GT = {1: 0.484, 2: 0.503, 3: 0.406, 4: 0.515, 5: 0.237, 6: 0.551}
LANDMARKS = "/perception/lidar/tree_landmarks"


def read_landmarks(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    typ = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if LANDMARKS not in typ:
        return []
    msg_cls = get_message(typ[LANDMARKS])
    frames = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == LANDMARKS:
            frames.append(rs.deserialize_message(data, msg_cls))
    return frames


def summarize(bag_dir, tid, d):
    frames = read_landmarks(bag_dir)
    counts = [len(f.trees) for f in frames]
    nonzero = [c for c in counts if c > 0]
    # landmark dominante por frame = o mais próximo de (x=d, y=0)
    dbh, conf, hgt, sd = [], [], [], []
    for f in frames:
        if not f.trees:
            continue
        best = min(f.trees, key=lambda tr: (tr.base.x - d) ** 2 + tr.base.y ** 2)
        dbh.append(best.diameter)
        conf.append(best.confidence)
        hgt.append(best.height)
        sd.append(best.diameter_stddev)
    med = lambda v: st.median(v) if v else float("nan")
    gt = GT.get(tid, float("nan"))
    mdbh = med(dbh)
    return dict(
        tid=tid, d=d, n_frames=len(frames),
        n_det=len(nonzero), n_per=med(counts) if counts else 0,
        dbh=mdbh, err=mdbh - gt, gt=gt, conf=med(conf),
        hgt=med(hgt), sd=med(sd),
    )


def main():
    bdir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(__file__), "bags")
    rows = []
    for p in sorted(glob.glob(os.path.join(bdir, "trunk_range_t*_d*"))):
        m = re.search(r"trunk_range_t(\d+)_d(\d+)$", p)
        if not m:
            continue
        rows.append(summarize(p, int(m.group(1)), int(m.group(2))))
    rows.sort(key=lambda r: (r["tid"], r["d"]))
    print(f"{'tree':>4} {'d':>3} {'GT':>5} | {'frames':>6} {'det':>4} {'n/fr':>4} "
          f"| {'DBH':>6} {'err':>7} {'sd':>5} | {'conf':>5} {'hgt':>5}")
    print("-" * 78)
    for r in rows:
        flag = "" if abs(r["err"]) < 0.05 else ("  <-- ERRO" if r["dbh"] == r["dbh"] else "  <-- SEM FIT")
        if r["n_per"] and r["n_per"] > 1:
            flag += " [MULTI-CLUSTER]"
        print(f"{r['tid']:>4} {r['d']:>3} {r['gt']:>5.2f} | {r['n_frames']:>6} "
              f"{r['n_det']:>4} {r['n_per']:>4.0f} | {r['dbh']:>6.3f} {r['err']:>+7.3f} "
              f"{r['sd']:>5.2f} | {r['conf']:>5.2f} {r['hgt']:>5.2f}{flag}")


if __name__ == "__main__":
    main()

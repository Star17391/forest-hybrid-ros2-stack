#!/usr/bin/env python3
"""Mostra a CONVERGÊNCIA do DBH multi-vista ao longo do tempo, de um bag mv_*.

Lê /slam/tree_map e segue o landmark dominante (o que mais perto fica de (d,0) no
1.º frame, depois por uid). Imprime DBH, nº de observações e sigma_DBH (da covariância)
em função do tempo — para ver se com o movimento o DBH converge para o GT.

Uso:  analyze_multiview.py <bag_dir> <gt_dbh> [d_esperado=4]
"""
import math
import sys

import rclpy.serialization as rs
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

TREE_MAP = "/slam/tree_map"


def main():
    bag, gt = sys.argv[1], float(sys.argv[2])
    d = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    typ = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = get_message(typ[TREE_MAP])
    t0 = None
    track_uid = None
    print(f"{'t[s]':>6} {'uid':>5} {'nobs':>5} {'DBH':>6} {'err':>7} {'sigma':>6} {'conf':>5}")
    print("-" * 48)
    last_print = -1.0
    while reader.has_next():
        topic, data, tns = reader.read_next()
        if topic != TREE_MAP:
            continue
        m = rs.deserialize_message(data, cls)
        ts = tns * 1e-9
        if t0 is None:
            t0 = ts
        rel = ts - t0
        if not m.trees:
            continue
        if track_uid is None:
            best = min(m.trees, key=lambda tr: (tr.position.x - d) ** 2 + tr.position.y ** 2)
            track_uid = best.uid
        sel = [tr for tr in m.trees if tr.uid == track_uid]
        if not sel:
            continue
        tr = sel[0]
        if rel - last_print < 1.0:   # 1 linha por segundo
            continue
        last_print = rel
        sig = math.sqrt(max(tr.covariance[8], 0.0))
        print(f"{rel:>6.1f} {tr.uid:>5} {tr.n_observations:>5} {tr.diameter:>6.3f} "
              f"{tr.diameter - gt:>+7.3f} {sig:>6.3f} {tr.confidence:>5.2f}")


if __name__ == "__main__":
    main()

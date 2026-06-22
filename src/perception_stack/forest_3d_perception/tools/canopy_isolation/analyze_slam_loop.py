#!/usr/bin/env python3
"""Valida o Tree-SLAM end-to-end de um bag de LAÇO FECHADO (multi-árvore).

Reporta por componente:
  TRACKER       — nº de landmarks ao longo do tempo, n_observations por uid,
                  estabilidade da posição, classes, quais ficaram dormentes.
  BACKEND       — pose_covariance_trace (qualidade da pose) ao longo do tempo;
                  deriva/refino das posições dos landmarks.
  RELOCALIZADOR — ao fechar o laço, há uids DUPLICADOS (dois landmarks no mesmo
                  sítio com uid diferente)? Se sim, o loop closure FALHOU.

Uso: analyze_slam_loop.py <bag_dir>
"""
import math
import sys

import rclpy.serialization as rs
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

TREE_MAP = "/slam/tree_map"
STATUS = "/slam/status"
DUP_DIST = 0.6  # m: dois landmarks mais perto que isto = a MESMA árvore


def reader_for(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    return r, {t.name: t.type for t in r.get_all_topics_and_types()}


def main():
    bag = sys.argv[1]
    r, typ = reader_for(bag)
    map_cls = get_message(typ[TREE_MAP]) if TREE_MAP in typ else None
    st_cls = get_message(typ[STATUS]) if STATUS in typ else None

    t0 = None
    status_rows = []          # (t, mode, n_lm, cov_trace, owns)
    map_snaps = []            # (t, {uid: (x,y,diam,conf,nobs,cls,last_seen)})
    while r.has_next():
        topic, data, tns = r.read_next()
        ts = tns * 1e-9
        if t0 is None:
            t0 = ts
        rel = ts - t0
        if topic == STATUS and st_cls:
            m = rs.deserialize_message(data, st_cls)
            status_rows.append((rel, m.mode, m.n_landmarks_tracked,
                                m.pose_covariance_trace, m.owns_map_to_odom))
        elif topic == TREE_MAP and map_cls:
            m = rs.deserialize_message(data, map_cls)
            d = {}
            for tr in m.trees:
                ls = tr.last_seen.sec + tr.last_seen.nanosec * 1e-9
                d[tr.uid] = (tr.position.x, tr.position.y, tr.diameter,
                             tr.confidence, tr.n_observations, tr.semantic_class, ls)
            map_snaps.append((rel, d))

    dur = (map_snaps[-1][0] if map_snaps else (status_rows[-1][0] if status_rows else 0.0))
    print(f"=== BAG {bag}  ({dur:.0f}s, {len(map_snaps)} mapas, {len(status_rows)} status) ===\n")

    # ---------------- BACKEND: pose cov + nº landmarks ----------------
    print("## BACKEND — qualidade da pose (cov trace) e nº de landmarks")
    if status_rows:
        print(f"{'t[s]':>6} {'mode':>5} {'n_lm':>5} {'cov_trace':>10} {'owns':>5}")
        step = max(1, len(status_rows) // 12)
        for i in range(0, len(status_rows), step):
            t, mo, nl, cov, ow = status_rows[i]
            print(f"{t:6.1f} {mo:5d} {nl:5d} {cov:10.5f} {str(ow):>5}")
    else:
        print("  (sem /slam/status no bag)")

    # ---------------- TRACKER: ciclo de vida ----------------
    print("\n## TRACKER — ciclo de vida dos landmarks")
    if map_snaps:
        # uids ao longo do tempo
        print(f"{'t[s]':>6} {'n_uids':>7}  uids vistos")
        step = max(1, len(map_snaps) // 10)
        for i in range(0, len(map_snaps), step):
            t, d = map_snaps[i]
            uids = sorted(d.keys())
            shown = ",".join(str(u) for u in uids[:10]) + ("..." if len(uids) > 10 else "")
            print(f"{t:6.1f} {len(uids):7d}  {shown}")
        # estado final por uid
        tf, df = map_snaps[-1]
        tmax = max((v[6] for v in df.values()), default=0.0)
        print(f"\n  Estado final ({len(df)} landmarks):")
        print(f"  {'uid':>4} {'x':>7} {'y':>7} {'diam':>6} {'conf':>5} {'nobs':>5} {'cls':>4} {'dormente?':>9}")
        for uid in sorted(df):
            x, y, dm, cf, no, cl, ls = df[uid]
            dormant = "sim" if (tmax - ls) > 3.0 else "-"
            print(f"  {uid:>4} {x:7.2f} {y:7.2f} {dm:6.3f} {cf:5.2f} {no:5d} {cl:>4} {dormant:>9}")
    else:
        print("  (sem /slam/tree_map no bag)")

    # ---------------- RELOCALIZADOR: duplicados no fim ----------------
    print("\n## RELOCALIZADOR — duplicação ao fechar o laço")
    if map_snaps:
        tf, df = map_snaps[-1]
        items = list(df.items())
        dups = []
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                ua, (xa, ya, *_), = items[i][0], items[i][1]
                ub, (xb, yb, *_), = items[j][0], items[j][1]
                dd = math.hypot(xa - xb, ya - yb)
                if dd < DUP_DIST:
                    dups.append((ua, ub, dd))
        print(f"  landmarks finais: {len(df)}")
        if dups:
            print(f"  !! {len(dups)} PARES duplicados (< {DUP_DIST} m, uid diferente) "
                  f"-> loop closure NÃO re-associou:")
            for ua, ub, dd in dups[:10]:
                print(f"     uid {ua} ~ uid {ub}  (dist {dd:.2f} m)")
        else:
            print(f"  OK: nenhum par duplicado -> revisita re-associou aos uids existentes.")
        # pico vs fim de n_uids (loop closure pode FUNDIR -> n desce)
        peak = max(len(d) for _, d in map_snaps)
        print(f"  nº de landmarks: pico={peak}, fim={len(df)}"
              + ("  (desceu -> houve fusão/merge)" if len(df) < peak else ""))
    else:
        print("  (sem /slam/tree_map no bag)")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Mede a cobertura em AZIMUTE das árvores detetadas (teste do caminho B).

Mundo surround: 16 árvores a 4 e 8 m a rodear o robô. Sem o caminho B, só as
FRONTAIS (|az| < ~90°, com solo) são detetadas. Com o caminho B, devem aparecer
também as de TRÁS/LADO (|az| > 90°). Lê /perception/lidar/tree_landmarks (frame
do robô → az = atan2(y,x)) e o /slam/tree_map, e classifica por setor.

Uso: analyze_surround.py <bag_dir>
"""
import math
import sys

import rclpy.serialization as rs
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

LM = "/perception/lidar/tree_landmarks"
MAP = "/slam/tree_map"


def sector(az_deg):
    a = abs(az_deg)
    if a <= 60:
        return "FRENTE (|az|<=60)"
    if a <= 120:
        return "LADO (60-120)"
    return "TRAS (>120)"


def main():
    bag = sys.argv[1]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    typ = {t.name: t.type for t in r.get_all_topics_and_types()}
    lm_cls = get_message(typ[LM]) if LM in typ else None
    map_cls = get_message(typ[MAP]) if MAP in typ else None

    # tree_landmarks: junta deteções de todos os frames por setor (perceção bruta).
    per_frame_best = {}  # sector -> max nº de árvores nesse setor num único frame
    last_lm = None
    map_last = None
    while r.has_next():
        topic, data, tns = r.read_next()
        if topic == LM and lm_cls:
            m = rs.deserialize_message(data, lm_cls)
            counts = {}
            for tr in m.trees:
                az = math.degrees(math.atan2(tr.base.y, tr.base.x))
                s = sector(az)
                counts[s] = counts.get(s, 0) + 1
            for s, c in counts.items():
                per_frame_best[s] = max(per_frame_best.get(s, 0), c)
            last_lm = m
        elif topic == MAP and map_cls:
            map_last = rs.deserialize_message(data, map_cls)

    print("== PERCEÇÃO (/perception/lidar/tree_landmarks) — máx árvores por setor num frame ==")
    for s in ["FRENTE (|az|<=60)", "LADO (60-120)", "TRAS (>120)"]:
        print(f"   {s:22} {per_frame_best.get(s, 0)}")
    if last_lm:
        print(f"   (último frame: {len(last_lm.trees)} deteções)")

    if map_last:
        print(f"\n== SLAM (/slam/tree_map) — {len(map_last.trees)} landmarks no mapa ==")
        print(f"   {'uid':>4} {'az°':>6} {'dist':>5} {'diam':>6} {'cls':>4} {'cov_xx':>8} {'cov_zz':>8}")
        for tr in sorted(map_last.trees, key=lambda t: math.atan2(t.position.y, t.position.x)):
            az = math.degrees(math.atan2(tr.position.y, tr.position.x))
            dist = math.hypot(tr.position.x, tr.position.y)
            cov = tr.covariance
            print(f"   {tr.uid:>4} {az:6.0f} {dist:5.1f} {tr.diameter:6.3f} "
                  f"{tr.semantic_class:>4} {cov[0]:8.4f} {cov[8]:8.4f}")
        back = sum(1 for tr in map_last.trees
                   if abs(math.degrees(math.atan2(tr.position.y, tr.position.x))) > 90)
        print(f"\n   landmarks na metade de TRÁS (|az|>90): {back}  "
              + ("<- caminho B a funcionar!" if back > 0 else "<- nenhum (caminho B sem efeito)"))


if __name__ == "__main__":
    main()

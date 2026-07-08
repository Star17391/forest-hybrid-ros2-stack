#!/usr/bin/env python3
"""Capturador/avaliador do Tree-SLAM (NÃO lança processos).

Subscreve /slam/tree_map e, a CADA mensagem, reavalia contra a GT do mundo e
reescreve o JSON de resultado — assim mesmo que seja morto (SIGINT/SIGTERM ao
acabar o bag), o último estado fica gravado. A orquestração (lançar o nó +
tocar o bag) fica a cargo do bash, que NÃO mistura fds de processos ROS com o
shell (ver slam_replay.sh). Determinístico: mesma entrada → mesmo resultado.

Uso (normalmente via slam_replay.sh):
  python3 slam_capture_eval.py --world forest_flat_trees \
      --out result.json --tag baseline [--listen-secs 0]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slam_race import evaluate, load_gt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="forest_flat_trees")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", default="run")
    ap.add_argument("--match-gate", type=float, default=1.5)
    ap.add_argument("--listen-secs", type=float, default=0.0,
                    help="0 = ouve até receber sinal (morto pelo bash)")
    args = ap.parse_args()

    trees, rocks = load_gt(args.world)
    gt_tree_xy = [(x, y) for _, _, x, y, _ in trees]
    gt_rock_xy = list(rocks)

    import rclpy
    from rclpy.node import Node
    from forest_hybrid_msgs.msg import TrackedTreeLandmarkArray

    def write_result(last, n_msgs):
        if last is None:
            return
        trunks = [{"uid": int(t.uid), "x": t.position.x, "y": t.position.y,
                   "confidence": float(t.confidence),
                   "n_obs": int(t.n_observations),
                   "diameter": float(t.diameter)}
                  for t in last.trees if t.semantic_class == 3]
        rock_tracks = [{"uid": int(t.uid), "x": t.position.x, "y": t.position.y,
                        "confidence": float(t.confidence),
                        "n_obs": int(t.n_observations)}
                       for t in last.trees if t.semantic_class == 6]
        m_tree = evaluate(gt_tree_xy, trunks, match_gate=args.match_gate)
        m_rock = (evaluate(gt_rock_xy, rock_tracks, match_gate=args.match_gate)
                  if gt_rock_xy else None)
        result = {"tag": args.tag, "world": args.world, "n_map_msgs": n_msgs,
                  "trees": m_tree, "rocks": m_rock,
                  "n_trunk_tracks": len(trunks),
                  "n_rock_tracks": len(rock_tracks),
                  "trunk_tracks": trunks, "rock_tracks": rock_tracks,
                  "gt_tree_xy": gt_tree_xy, "gt_rock_xy": gt_rock_xy}
        tmp = args.out + ".tmp"
        with open(tmp, "w") as f:
            json.dump(result, f, indent=2)
        os.replace(tmp, args.out)

    class Capture(Node):
        def __init__(self):
            super().__init__("slam_capture_eval")
            self.last = None
            self.n_msgs = 0
            self.create_subscription(
                TrackedTreeLandmarkArray, "/slam/tree_map", self._cb, 10)

        def _cb(self, m):
            self.last = m
            self.n_msgs += 1
            write_result(m, self.n_msgs)  # persiste a cada msg

    rclpy.init()
    cap = Capture()
    print(f"[{args.tag}] capturador a ouvir /slam/tree_map "
          f"(GT {len(gt_tree_xy)} troncos)…", flush=True)
    import time
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(cap, timeout_sec=0.2)
            if args.listen_secs > 0 and (time.time() - t0) > args.listen_secs:
                break
    except KeyboardInterrupt:
        pass
    finally:
        write_result(cap.last, cap.n_msgs)
        print(f"[{args.tag}] terminou: {cap.n_msgs} mapas recebidos → {args.out}",
              flush=True)
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

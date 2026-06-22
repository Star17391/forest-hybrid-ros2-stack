#!/usr/bin/env python3
"""Avaliação da estabilização MULTI-VIEW do DBH no Tree-SLAM (camada forest_tree_slam).

Mede o DBH acumulado por track (uid) em /slam/tree_map — não o DBH por-frame da
perceção. O gate relevante aqui é o CV global do DBH de cada landmark ao longo do
tempo (multi-view), não o frame-a-frame da perceção.

Auto-conduz o robô (reta + rotação + arco) para exercitar vistas de ângulos diferentes.

Uso:
  forest diag tree-slam-dbh --world forest_gentle_trees_rocks --duration 40
  (requer: forest up sim-tree-slam -d --world forest_gentle_trees_rocks)
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from forest_hybrid_msgs.msg import TrackedTreeLandmarkArray

CMD_VEL_TOPIC = "/forest_gen/cmd_vel"
TREE_MAP_TOPIC = "/slam/tree_map"


def load_gt_trees(sdf_path: Path) -> list[tuple[str, float, float]]:
    text = sdf_path.read_text(encoding="utf-8", errors="ignore")
    trees: list[tuple[str, float, float]] = []
    pat = re.compile(r"<name>\s*(tree_\d+)\s*</name>\s*<pose>([^<]+)</pose>", re.DOTALL)
    for m in pat.finditer(text):
        parts = m.group(2).split()
        if len(parts) >= 2:
            trees.append((m.group(1), float(parts[0]), float(parts[1])))
    return trees


class TreeSlamDbhEval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("tree_slam_dbh_eval")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args
        self.gt = load_gt_trees(Path(args.world))
        if not self.gt:
            self.get_logger().error(f"0 troncos GT em {args.world}")
            raise SystemExit(2)

        self.uid_dbhs: dict[int, list[float]] = defaultdict(list)
        self.uid_positions: dict[int, list[tuple[float, float]]] = defaultdict(list)
        self.uid_obs: dict[int, int] = {}
        self.frames = 0

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TrackedTreeLandmarkArray, TREE_MAP_TOPIC, self._on_map, qos)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_timer(args.report_every, self._report)

    def _on_map(self, msg: TrackedTreeLandmarkArray) -> None:
        self.frames += 1
        for t in msg.trees:
            uid = int(t.uid)
            self.uid_dbhs[uid].append(float(t.diameter))
            self.uid_positions[uid].append((float(t.position.x), float(t.position.y)))
            self.uid_obs[uid] = int(t.n_observations)

    def _drive(self, lin: float, ang: float) -> None:
        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        self.cmd_pub.publish(tw)

    def run(self) -> int:
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.args.duration:
            if self.args.drive:
                el = time.monotonic() - t0
                frac = el % 12.0
                if frac < 4.0:
                    self._drive(0.3, 0.0)
                elif frac < 6.0:
                    self._drive(0.0, 0.0)
                elif frac < 10.0:
                    self._drive(0.0, 0.5)
                else:
                    self._drive(0.15, 0.3)
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.args.drive:
            for _ in range(5):
                self._drive(0.0, 0.0)
                time.sleep(0.02)
        return self.finish()

    def _match_gt(self, uid: int) -> int:
        pos = self.uid_positions.get(uid)
        if not pos:
            return -1
        mx = statistics.mean(p[0] for p in pos)
        my = statistics.mean(p[1] for p in pos)
        best_i, best_d = -1, self.args.match_radius
        for i, (_, gx, gy) in enumerate(self.gt):
            d = math.hypot(mx - gx, my - gy)
            if d < best_d:
                best_i, best_d = i, d
        return best_i

    def _compute(self):
        min_samples = self.args.min_samples
        min_obs = self.args.min_observations
        dbh_tol = self.args.dbh_tol_pct / 100.0

        cvs: list[float] = []
        stable_uids: list[int] = []
        matched_gt: set[int] = set()

        for uid, samples in self.uid_dbhs.items():
            if len(samples) < min_samples:
                continue
            if self.uid_obs.get(uid, 0) < min_obs:
                continue
            mean_d = statistics.mean(samples)
            if mean_d < 1e-3:
                continue
            cv = statistics.pstdev(samples) / mean_d
            cvs.append(cv)
            if cv <= dbh_tol:
                stable_uids.append(uid)
            gt_i = self._match_gt(uid)
            if gt_i >= 0:
                matched_gt.add(gt_i)

        med_cv = statistics.median(cvs) if cvs else float("nan")
        frac_stable = (len(stable_uids) / len(cvs)) if cvs else 0.0
        recall_tracks = len(matched_gt) / len(self.gt) if self.gt else 0.0
        return cvs, med_cv, frac_stable, stable_uids, recall_tracks

    def _report(self) -> None:
        cvs, med_cv, frac_stable, stable_uids, recall_tracks = self._compute()
        self.get_logger().info(
            f"[{self.frames} frames] tracks={len(self.uid_dbhs)} "
            f"com CV={len(cvs)} med_cv={med_cv*100:.1f}% "
            f"estáveis={len(stable_uids)} recall_gt={recall_tracks:.2f}")

    def finish(self) -> int:
        cvs, med_cv, frac_stable, stable_uids, recall_tracks = self._compute()
        tol = self.args.dbh_tol_pct
        gate_cv = (not cvs or med_cv * 100 <= tol)
        gate_stable = frac_stable >= self.args.stable_frac_target

        print("\n" + "=" * 64)
        print("  AVALIAÇÃO DBH MULTI-VIEW — Tree-SLAM (/slam/tree_map)")
        print("=" * 64)
        print(f"  frames processados      : {self.frames}")
        print(f"  tracks únicos (uid)     : {len(self.uid_dbhs)}")
        print(f"  tracks com ≥{self.args.min_samples} amostras : {len(cvs)}")
        if cvs:
            print(f"  DBH CV global (mediana) : {med_cv*100:.1f}%  (gate ≤{tol:.0f}%)  "
                  f"{'PASS' if gate_cv else 'FAIL'}")
            print(f"  tracks com CV ≤ {tol:.0f}%   : {len(stable_uids)}/{len(cvs)}  "
                  f"({frac_stable*100:.0f}%)  alvo ≥{self.args.stable_frac_target*100:.0f}%  "
                  f"{'PASS' if gate_stable else 'FAIL'}")
        else:
            print("  DBH estabilidade        : n/a (poucas amostras — aumenta --duration)")
        print(f"  recall GT (por posição) : {recall_tracks:.2f}")
        print("-" * 64)
        overall = gate_cv and gate_stable and len(cvs) > 0
        print(f"  GATE GLOBAL             : {'PASS ✓' if overall else 'FAIL ✗'}")
        print("=" * 64)
        return 0 if overall else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="DBH multi-view do Tree-SLAM vs GT")
    ap.add_argument("--world", required=True, help="Caminho ou nome do .sdf")
    ap.add_argument("--topic", default=TREE_MAP_TOPIC)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--drive", dest="drive", action="store_true", default=True)
    ap.add_argument("--no-drive", dest="drive", action="store_false")
    ap.add_argument("--match-radius", type=float, default=1.5)
    ap.add_argument("--min-samples", type=int, default=8,
                    help="amostras mínimas de /slam/tree_map por uid")
    ap.add_argument("--min-observations", type=int, default=5,
                    help="n_observations mínimo no landmark")
    ap.add_argument("--dbh-tol-pct", type=float, default=10.0)
    ap.add_argument("--stable-frac-target", type=float, default=0.5,
                    help="fração mínima de tracks com CV≤tol para PASS")
    ap.add_argument("--report-every", type=float, default=5.0)
    args = ap.parse_args()

    if not Path(args.world).exists():
        import os
        base = os.environ.get("FORESTGEN_PATH", "")
        cand = Path(base) / "worlds" / args.world
        cand2 = cand.with_suffix(".sdf") if cand.suffix != ".sdf" else cand
        if cand2.exists():
            args.world = str(cand2)
        else:
            print(f"ERRO: mundo não encontrado: {args.world}", file=sys.stderr)
            return 2

    rclpy.init()
    node = TreeSlamDbhEval(args)
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = node.finish()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())

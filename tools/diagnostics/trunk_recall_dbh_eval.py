#!/usr/bin/env python3
"""Avaliação ISOLADA da perceção de troncos (Agente 2) — sem Tree-SLAM.

Mede o GATE da camada de perceção contra o ground-truth do mundo Gazebo:
  - RECALL    : fração de troncos GT (visíveis no range) que são detectados.
  - PRECISÃO  : fração de detecções que casam com um tronco GT (resto = falsos +).
  - DBH ±%    : estabilidade do DBH do MESMO tronco frame-a-frame (gate: ±10%).

Como: subscreve /perception/lidar/tree_landmarks (base_link, por-frame), transforma
cada base para o frame do mundo via TF, e associa ao tronco GT mais próximo. Os GT
saem do .sdf (`<name>tree_N</name><pose>x y z ...</pose>`). NÃO faz tracking para o
output da perceção — a associação é só para a métrica (o harness é que tem estado,
a perceção continua stateless).

Uso:
  ros2 run ... NÃO — corre direto (a sim tem de estar a publicar):
    python3 trunk_recall_dbh_eval.py --world <forest_rugged_trees_rocks.sdf> \
            --duration 40 --recall-target 0.7
  Ou via CLI:  forest diag trunks --world forest_rugged_trees_rocks
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401  (regista do_transform_point)

from forest_hybrid_msgs.msg import TreeLandmarkArray


def load_gt_trees(sdf_path: Path) -> list[tuple[str, float, float]]:
    """Extrai (name, x, y) dos troncos GT do .sdf (include com name=tree_N)."""
    text = sdf_path.read_text(encoding="utf-8", errors="ignore")
    trees: list[tuple[str, float, float]] = []
    pat = re.compile(r"<name>\s*(tree_\d+)\s*</name>\s*<pose>([^<]+)</pose>", re.DOTALL)
    for m in pat.finditer(text):
        parts = m.group(2).split()
        if len(parts) >= 2:
            trees.append((m.group(1), float(parts[0]), float(parts[1])))
    return trees


class TrunkEval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("trunk_recall_dbh_eval")
        self.world_frame = args.world_frame
        self.match_r = args.match_radius
        self.range_max = args.range_max
        self.min_seen = args.min_seen_frac
        self.recall_target = args.recall_target
        self.dbh_tol = args.dbh_tol_pct / 100.0

        self.gt = load_gt_trees(Path(args.world))
        if not self.gt:
            self.get_logger().error(f"0 troncos GT em {args.world} — verifica o caminho")
            raise SystemExit(2)
        self.get_logger().info(
            f"GT: {len(self.gt)} troncos | world_frame={self.world_frame} "
            f"match_r={self.match_r}m range={self.range_max}m alvo_recall={self.recall_target}")

        # Estado por tronco GT (índice = posição em self.gt).
        self.gt_seen_frames = [0] * len(self.gt)   # frames em que esteve no range
        self.gt_hit_frames = [0] * len(self.gt)    # frames em que foi detectado
        self.gt_dbhs: list[list[float]] = [[] for _ in self.gt]
        self.frames = 0
        self.det_total = 0
        self.det_matched = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            TreeLandmarkArray, args.topic, self.on_landmarks, qos)
        self.create_timer(args.report_every, self.report)
        if args.duration > 0:
            self.create_timer(args.duration, self.finish)

    def robot_xy_in_world(self, stamp) -> tuple[float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.robot_frame, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def on_landmarks(self, msg: TreeLandmarkArray) -> None:
        self.robot_frame = msg.header.frame_id
        # Pose do robô no mundo (origem do frame base no world).
        rob = self.robot_xy_in_world(msg.header.stamp)
        if rob is None:
            return
        self.frames += 1

        # 1) Quais GT estão no range deste frame (visíveis).
        in_range = []
        for i, (_, gx, gy) in enumerate(self.gt):
            d = math.hypot(gx - rob[0], gy - rob[1])
            if d <= self.range_max:
                self.gt_seen_frames[i] += 1
                in_range.append(i)

        # 2) Transforma cada detecção TRONCO-dominante para o mundo e associa ao GT.
        hit_this_frame: set[int] = set()
        for t in msg.trees:
            scores = [float(t.class_scores[0]), float(t.class_scores[1]), float(t.class_scores[2])]
            if max(scores) < 0.05 or scores.index(max(scores)) != 0:
                continue  # emissão multi-classe: recall de tronco só conta argmax=tronco
            self.det_total += 1
            ps = PointStamped()
            ps.header = msg.header
            ps.point.x, ps.point.y, ps.point.z = t.base.x, t.base.y, t.base.z
            try:
                pw = self.tf_buffer.transform(ps, self.world_frame, timeout=rclpy.duration.Duration(seconds=0.05))
            except Exception:
                continue
            best_i, best_d = -1, self.match_r
            for i in in_range:
                _, gx, gy = self.gt[i]
                d = math.hypot(pw.point.x - gx, pw.point.y - gy)
                if d < best_d:
                    best_i, best_d = i, d
            if best_i >= 0:
                self.det_matched += 1
                if best_i not in hit_this_frame:
                    self.gt_hit_frames[best_i] += 1
                    hit_this_frame.add(best_i)
                self.gt_dbhs[best_i].append(float(t.diameter))

    def _compute(self):
        # Um GT conta como "visível" se esteve no range em frames suficientes.
        min_frames = max(1, int(self.min_seen * max(self.frames, 1)))
        visible = [i for i in range(len(self.gt)) if self.gt_seen_frames[i] >= min_frames]
        detected = [i for i in visible if self.gt_hit_frames[i] > 0]
        recall = len(detected) / len(visible) if visible else 0.0
        precision = self.det_matched / self.det_total if self.det_total else 0.0
        # Estabilidade do DBH, duas leituras:
        #  - CV global: dispersão ao longo de TODA a observação (ângulos/distâncias
        #    diferentes) — exigente, relevante para o SLAM que vê o tronco de vários sítios.
        #  - frame-a-frame: mediana de |Δ|/dbh entre frames CONSECUTIVOS — o gate literal.
        cvs = []
        f2f = []
        for i in detected:
            d = self.gt_dbhs[i]
            if len(d) >= 3 and statistics.mean(d) > 1e-3:
                cvs.append(statistics.pstdev(d) / statistics.mean(d))
            for k in range(1, len(d)):
                if d[k - 1] > 1e-3:
                    f2f.append(abs(d[k] - d[k - 1]) / d[k - 1])
        med_cv = statistics.median(cvs) if cvs else float("nan")
        med_f2f = statistics.median(f2f) if f2f else float("nan")
        frac_stable = (sum(1 for c in cvs if c <= self.dbh_tol) / len(cvs)) if cvs else 0.0
        return visible, detected, recall, precision, med_cv, frac_stable, cvs, med_f2f

    def report(self) -> None:
        visible, detected, recall, precision, med_cv, frac_stable, cvs, med_f2f = self._compute()
        self.get_logger().info(
            f"[{self.frames} frames] visíveis={len(visible)} detectados={len(detected)} "
            f"recall={recall:.2f} precisão={precision:.2f} "
            f"DBH frame-a-frame={med_f2f*100:.1f}% CV-global={med_cv*100:.1f}%")

    def finish(self) -> None:
        visible, detected, recall, precision, med_cv, frac_stable, cvs, med_f2f = self._compute()
        print("\n" + "=" * 64)
        print("  AVALIAÇÃO DA PERCEÇÃO DE TRONCOS (isolada, sem SLAM)")
        print("=" * 64)
        print(f"  frames processados      : {self.frames}")
        print(f"  troncos GT no mundo     : {len(self.gt)}")
        print(f"  troncos GT visíveis     : {len(visible)}  (no range em ≥{self.min_seen*100:.0f}% dos frames)")
        print(f"  troncos detectados      : {len(detected)}")
        print(f"  RECALL                  : {recall:.2f}   (alvo ≥ {self.recall_target:.2f})  "
              f"{'PASS' if recall >= self.recall_target else 'FAIL'}")
        print(f"  PRECISÃO                : {precision:.2f}   ({self.det_matched}/{self.det_total} detecções casadas)")
        if cvs:
            print(f"  DBH FRAME-A-FRAME       : {med_f2f*100:.1f}%  (gate literal ±{self.dbh_tol*100:.0f}%)  "
                  f"{'PASS' if med_f2f <= self.dbh_tol else 'FAIL'}")
            print(f"  DBH CV global (obs.)    : {med_cv*100:.1f}%  (variação ao longo de ângulos/distâncias)")
            print(f"  troncos com CV ≤ {self.dbh_tol*100:.0f}%    : {frac_stable:.2f}")
        else:
            print(f"  DBH estabilidade        : n/a (poucas amostras)")
        # Gate literal usa o frame-a-frame (o que o prompt pede); CV global é diagnóstico.
        gate = recall >= self.recall_target and (not cvs or med_f2f <= self.dbh_tol)
        print("-" * 64)
        print(f"  GATE GLOBAL             : {'PASS ✓' if gate else 'FAIL ✗'}")
        print("=" * 64)
        rclpy.shutdown()


def main() -> int:
    ap = argparse.ArgumentParser(description="Avaliação recall + estabilidade DBH da perceção de troncos")
    ap.add_argument("--world", required=True, help="Caminho do .sdf (ou nome em FORESTGEN_PATH/worlds)")
    ap.add_argument("--topic", default="/perception/lidar/tree_landmarks")
    ap.add_argument("--world-frame", default="map")
    ap.add_argument("--match-radius", type=float, default=1.5, help="raio de associação detecção↔GT [m]")
    ap.add_argument("--range-max", type=float, default=12.0, help="alcance em que um GT conta como visível [m]")
    ap.add_argument("--min-seen-frac", type=float, default=0.2, help="fração de frames no range p/ ser 'visível'")
    ap.add_argument("--recall-target", type=float, default=0.7)
    ap.add_argument("--dbh-tol-pct", type=float, default=10.0)
    ap.add_argument("--report-every", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=40.0, help="0 = corre até Ctrl-C")
    args = ap.parse_args()

    # Resolver nome de mundo via FORESTGEN_PATH se não for um caminho existente.
    if not Path(args.world).exists():
        import os
        base = os.environ.get("FORESTGEN_PATH", "")
        cand = Path(base) / "worlds" / args.world
        cand2 = cand.with_suffix(".sdf") if cand.suffix != ".sdf" else cand
        if cand2.exists():
            args.world = str(cand2)
        else:
            print(f"ERRO: mundo não encontrado: {args.world} (nem em {cand2})", file=sys.stderr)
            return 2

    rclpy.init()
    node = TrunkEval(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())

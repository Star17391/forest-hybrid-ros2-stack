#!/usr/bin/env python3
"""Audit experimental LiDAR pipeline — localiza onde a funnel quebra.

Verifica:
  - nó lidar3d_experimental_node activo
  - tópicos reais vs nomes errados (/ground_points não existe)
  - debug_stats JSON (status, contagens por etapa)
  - hz e pontos por tópico
  - /perception/lidar/tree_landmarks (contrato SLAM): DBH, cov, estabilidade inter-frame

Testes manuais (param debug_stage no nó):
  1 = só voxel publicado em ground
  2 = só CSF (ground + non_ground)
  3 = só clustering (CSF bypass, voxel -> cluster)
  0/4 = pipeline completa

Uso:
  forest up sim-lidar3d-experimental -d --world forest_rugged_trees_rocks
  forest test lidar3d-experimental 60
  # ou:
  python3 tools/diagnostics/lidar3d_experimental_pipeline_audit.py --duration 60
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass, field

import rclpy
from forest_hybrid_msgs.msg import TreeLandmarkArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


TOPIC_TREE_LANDMARKS = "/perception/lidar/tree_landmarks"

TOPICS_REAL = {
    "input": "/sensors/lidar/points",
    "ground": "/perception/lidar3d/experimental/ground",
    "non_ground": "/perception/lidar3d/experimental/non_ground",
    "clusters": "/perception/lidar3d/experimental/clusters",
    "tree_candidates": "/perception/lidar3d/experimental/stem_candidates",
    "debug_stats": "/perception/lidar3d/experimental/debug_stats",
    "dbg_voxel": "/perception/lidar3d/experimental/debug/stage_voxel",
    "dbg_ground": "/perception/lidar3d/experimental/debug/stage_ground",
    "dbg_non_ground": "/perception/lidar3d/experimental/debug/stage_non_ground",
    "dbg_clusters": "/perception/lidar3d/experimental/debug/stage_clusters",
}

TOPICS_WRONG = [
    "/ground_points",
    "/non_ground_points",
    "/perception/lidar3d/ground",
    "/perception/lidar3d/trunks",
]


@dataclass
class TopicStats:
    msgs: int = 0
    pts: int = 0
    last_width: int = 0


@dataclass
class DebugStatsAgg:
    count: int = 0
    statuses: dict[str, int] = field(default_factory=dict)
    last: dict | None = None


@dataclass
class TrunkSample:
    x: float
    y: float
    diameter: float


@dataclass
class TreeLandmarkAudit:
    msgs: int = 0
    total_trees: int = 0
    max_trees_per_frame: int = 0
    diameters: list[float] = field(default_factory=list)
    diameter_stddevs: list[float] = field(default_factory=list)
    confidences: list[float] = field(default_factory=list)
    frames_with_trees: int = 0
    trees_with_cov: int = 0
  trees_with_diameter_sigma: int = 0
  trees_with_class_scores: int = 0
  class_score_sum_bad: int = 0
  dom_trunk: int = 0
  dom_rock: int = 0
  dom_obstacle: int = 0
  frame_id_ok: int = 0
    last_frame_id: str = ""
    last_tree_count: int = 0
    dbh_rel_changes: list[float] = field(default_factory=list)
    _prev_trunks: list[TrunkSample] = field(default_factory=list)

    def on_msg(self, msg: TreeLandmarkArray) -> None:
        self.msgs += 1
        n = len(msg.trees)
        self.total_trees += n
        self.max_trees_per_frame = max(self.max_trees_per_frame, n)
        self.last_tree_count = n
        self.last_frame_id = msg.header.frame_id

        if msg.header.frame_id and "base_link" in msg.header.frame_id:
            self.frame_id_ok += 1

        if n == 0:
            self._prev_trunks = []
            return

        self.frames_with_trees += 1
        cur: list[TrunkSample] = []

        for t in msg.trees:
            d = float(t.diameter)
            self.diameters.append(d)
            self.diameter_stddevs.append(float(t.diameter_stddev))
            self.confidences.append(float(t.confidence))
            if float(t.diameter_stddev) > 1e-6:
                self.trees_with_diameter_sigma += 1
            scores = [float(t.class_scores[0]), float(t.class_scores[1]), float(t.class_scores[2])]
            ssum = sum(scores)
            if ssum > 0.05:
                self.trees_with_class_scores += 1
            if abs(ssum - 1.0) > 0.08 or min(scores) < -0.01:
                self.class_score_sum_bad += 1
            dom = scores.index(max(scores))
            if dom == 0:
                self.dom_trunk += 1
            elif dom == 1:
                self.dom_rock += 1
            else:
                self.dom_obstacle += 1
            cov = list(t.base_covariance)
            if len(cov) >= 9 and (cov[0] > 0 or cov[4] > 0 or cov[8] > 0):
                self.trees_with_cov += 1
            cur.append(
                TrunkSample(
                    x=float(t.base.x),
                    y=float(t.base.y),
                    diameter=d,
                )
            )

        if self._prev_trunks:
            self._match_dbh_stability(cur)
        self._prev_trunks = cur

    def _match_dbh_stability(
        self, cur: list[TrunkSample], match_radius_m: float = 0.45
    ) -> None:
        prev = self._prev_trunks
        used: set[int] = set()
        for p in prev:
            best_j = -1
            best_d2 = match_radius_m * match_radius_m
            for j, c in enumerate(cur):
                if j in used:
                    continue
                d2 = (p.x - c.x) ** 2 + (p.y - c.y) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_j = j
            if best_j < 0:
                continue
            used.add(best_j)
            c = cur[best_j]
            mean_d = 0.5 * (p.diameter + c.diameter)
            if mean_d < 0.05:
                continue
            rel = abs(p.diameter - c.diameter) / mean_d
            if math.isfinite(rel):
                self.dbh_rel_changes.append(rel)

    def summary(self) -> dict:
        n_d = len(self.diameters)
        mean_dbh = sum(self.diameters) / n_d if n_d else 0.0
        mean_sigma = sum(self.diameter_stddevs) / n_d if n_d else 0.0
        mean_conf = sum(self.confidences) / n_d if n_d else 0.0
        dbh_changes = self.dbh_rel_changes
        mean_rel = sum(dbh_changes) / len(dbh_changes) if dbh_changes else 0.0
        max_rel = max(dbh_changes) if dbh_changes else 0.0
        pct_cov = (100.0 * self.trees_with_cov / n_d) if n_d else 0.0
        pct_sigma = (100.0 * self.trees_with_diameter_sigma / n_d) if n_d else 0.0
        pct_scores = (100.0 * self.trees_with_class_scores / n_d) if n_d else 0.0
        return {
            "msgs": self.msgs,
            "total_trees": self.total_trees,
            "max_trees_per_frame": self.max_trees_per_frame,
            "last_tree_count": self.last_tree_count,
            "frames_with_trees": self.frames_with_trees,
            "mean_dbh_m": round(mean_dbh, 4),
            "mean_diameter_stddev_m": round(mean_sigma, 4),
            "mean_confidence": round(mean_conf, 3),
            "pct_with_base_covariance": round(pct_cov, 1),
            "pct_with_diameter_stddev": round(pct_sigma, 1),
            "pct_with_class_scores": round(pct_scores, 1),
            "class_score_sum_bad": self.class_score_sum_bad,
            "dom_trunk": self.dom_trunk,
            "dom_rock": self.dom_rock,
            "dom_obstacle": self.dom_obstacle,
            "frame_id_last": self.last_frame_id,
            "frame_id_base_link_ok": self.frame_id_ok,
            "dbh_inter_frame_pairs": len(dbh_changes),
            "dbh_rel_change_mean": round(mean_rel, 4),
            "dbh_rel_change_max": round(max_rel, 4),
            "dbh_gate_10pct_pairs": sum(1 for r in dbh_changes if r <= 0.10),
        }


class ExpPipelineAudit(Node):
    def __init__(self, duration: float):
        super().__init__(
            "lidar3d_exp_pipeline_audit",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._duration = duration
        self._t0 = time.monotonic()
        self._topic_stats: dict[str, TopicStats] = {
            topic: TopicStats() for topic in TOPICS_REAL.values()
        }
        self._debug = DebugStatsAgg()
        self._trees = TreeLandmarkAudit()
        self._failed = False

        qos = qos_profile_sensor_data
        for key, topic in TOPICS_REAL.items():
            if key == "debug_stats":
                self.create_subscription(String, topic, self._on_debug, 10)
            else:
                self.create_subscription(
                    PointCloud2, topic, lambda m, t=topic: self._on_cloud(m, t), qos
                )

        self.create_subscription(
            TreeLandmarkArray, TOPIC_TREE_LANDMARKS, self._on_tree_landmarks, 10
        )

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"Experimental pipeline audit ({duration:.0f}s) + {TOPIC_TREE_LANDMARKS}…"
        )

    def _n_pts(self, msg: PointCloud2) -> int:
        return int(msg.width) * int(msg.height)

    def _on_cloud(self, msg: PointCloud2, topic: str) -> None:
        st = self._topic_stats.get(topic)
        if st is None:
            return
        st.msgs += 1
        n = self._n_pts(msg)
        st.pts += n
        st.last_width = n

    def _on_debug(self, msg: String) -> None:
        self._debug.count += 1
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._debug.last = data
        status = str(data.get("status", "unknown"))
        self._debug.statuses[status] = self._debug.statuses.get(status, 0) + 1

    def _on_tree_landmarks(self, msg: TreeLandmarkArray) -> None:
        self._trees.on_msg(msg)

    def _tick(self) -> None:
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            rclpy.shutdown()

    def _report_tree_landmarks(self) -> None:
        print("\n--- tree_landmarks (contrato SLAM /perception/lidar/tree_landmarks) ---")
        s = self._trees.summary()
        if self._trees.msgs == 0:
            print("  (zero mensagens — pipeline não publica ou tópico errado)")
            self._failed = True
            print("  >>> FALHA Fase-0: tree_landmarks ausente")
            return

        print(json.dumps(s, indent=2))
        if self._trees.total_trees == 0:
            print("  WARN: mensagens OK mas zero troncos — mover robô / ajustar classify?")
        if s["pct_with_base_covariance"] < 50.0 and self._trees.total_trees > 0:
            print("  WARN: base_covariance pouco preenchida (<50%)")
        if s["pct_with_diameter_stddev"] < 50.0 and self._trees.total_trees > 0:
            print("  WARN: diameter_stddev pouco preenchido (<50%)")
        if self._trees.total_trees > 0 and s["class_score_sum_bad"] > 0:
            print(f"  >>> FALHA Fase-1: class_scores inválidos ({s['class_score_sum_bad']})")
            self._failed = True
        elif self._trees.total_trees > 0 and s["pct_with_class_scores"] < 95.0:
            print("  >>> FALHA Fase-1: class_scores não preenchido em ≥95% das deteções")
            self._failed = True
        elif self._trees.total_trees > 0 and s["dom_rock"] == 0:
            print("  WARN: zero deteções rocha-dominantes (mundo sem rochas visíveis?)")
        if s["dbh_inter_frame_pairs"] >= 5:
            stable = s["dbh_gate_10pct_pairs"]
            total = s["dbh_inter_frame_pairs"]
            pct = 100.0 * stable / total
            print(
                f"  DBH inter-frame (±10% gate): {stable}/{total} pares estáveis ({pct:.0f}%)"
            )
            if pct < 50.0:
                print("  WARN: DBH instável frame-a-frame (alvo gate ≥50% pares ≤10%)")
        else:
            print(
                "  DBH inter-frame: poucos pares (mover robô ou aumentar --duration)"
            )

    def _report(self) -> None:
        print("\n" + "=" * 70)
        print("  EXPERIMENTAL LiDAR PIPELINE AUDIT (Fase 0)")
        print("=" * 70)

        self._check_node()
        self._check_wrong_topics()
        print("\n--- Tópicos reais (contagens) ---")
        for key, topic in TOPICS_REAL.items():
            st = self._topic_stats[topic]
            print(f"  {topic}")
            print(f"    msgs={st.msgs}  pts_total={st.pts}  last_width={st.last_width}")

        self._report_tree_landmarks()

        print("\n--- debug_stats (último frame) ---")
        print(f"  msgs received: {self._debug.count}")
        print(f"  status histogram: {dict(self._debug.statuses)}")
        if self._debug.last:
            print(json.dumps(self._debug.last, indent=2))
        else:
            print("  (nenhum JSON — nó não publica ou tópico errado)")

        print("\n--- DIAGNÓSTICO (evidência) ---")
        self._diagnose()
        if self._failed:
            print("\n>>> AUDIT FAILED")
        else:
            print("\n>>> AUDIT OK (Fase 0 — infra de medição)")

    def _check_node(self) -> None:
        print("\n--- Nós ROS ---")
        try:
            out = subprocess.check_output(["ros2", "node", "list"], text=True, timeout=8)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"  ERROR ros2 node list: {e}")
            return
        nodes = [ln.strip() for ln in out.splitlines() if ln.strip()]
        exp = "/lidar3d_experimental_node" in nodes
        leg = "/lidar3d_segmentation_node" in nodes
        print(f"  lidar3d_experimental_node: {'SIM' if exp else 'NAO'}")
        print(f"  lidar3d_segmentation_node: {'SIM' if leg else 'NAO (ok se csf-only)'}")
        if not exp:
            self._failed = True
            print("  >>> FALHA: nó experimental não está a correr.")
            print("      Use: forest up sim-lidar3d-experimental -d --world forest_rugged_trees_rocks")
            print("      ou: forest up sim-lidar3d-test -d --lidar3d --lidar3d-experimental")
            self._hint_sim_log_crash()

    def _hint_sim_log_crash(self) -> None:
        """If forest session log shows CSF segfault, tell user to rebuild."""
        import glob
        import os

        patterns = ["/run/user/*/forest/sessions/*/sim.log", "/tmp/forest/sessions/*/sim.log"]
        logs: list[str] = []
        for pat in patterns:
            logs.extend(glob.glob(pat))
        logs = sorted(logs, key=os.path.getmtime, reverse=True)
        for log_path in logs[:3]:
            try:
                with open(log_path, encoding="utf-8", errors="replace") as f:
                    tail = f.read()[-12000:]
            except OSError:
                continue
            if "lidar3d_experimental_node" in tail and "exit code -11" in tail:
                print("  >>> EVIDÊNCIA: crash CSF (SIGSEGV) no sim.log:")
                print(f"      {log_path}")
                print("      Rebuild: colcon build --packages-select forest_3d_perception")
                print("      Depois: source install/setup.bash && forest down --force && forest up ...")
                return

    def _check_wrong_topics(self) -> None:
        print("\n--- Tópicos que NÃO existem nesta pipeline ---")
        try:
            listed = subprocess.check_output(["ros2", "topic", "list"], text=True, timeout=8)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            listed = ""
        for t in TOPICS_WRONG:
            present = t in listed
            print(f"  {t}: {'PUBLICADO (inesperado)' if present else 'não existe (esperado)'}")

    def _diagnose(self) -> None:
        issues: list[str] = []
        inp = self._topic_stats[TOPICS_REAL["input"]]
        dbg = self._debug.last or {}
        status = str(dbg.get("status", ""))

        if inp.msgs == 0:
            issues.append(
                "ROOT: sem input em /sensors/lidar/points — Gazebo PLAY? bridge lidar3d?"
            )
        if self._debug.count == 0 and inp.msgs > 0:
            issues.append(
                "ROOT: input OK mas zero debug_stats — nó experimental não corre ou enabled:=false"
            )
        if self._trees.msgs == 0 and inp.msgs > 0:
            issues.append(
                f"ROOT: sem {TOPIC_TREE_LANDMARKS} — verificar publish_tree_landmarks no nó"
            )
        if status == "tf_fail":
            issues.append(
                "ROOT: TF laser -> marble_hd2/base_link falha — verificar TF tree (forest diag tf-audit)"
            )
        elif status == "crop_too_few":
            issues.append(
                f"ROOT: crop remove quase tudo (n_crop={dbg.get('n_crop',0)}) — "
                "ajustar min_range/max_range/min_z/max_z"
            )
        elif status == "disabled":
            issues.append("ROOT: enabled:=false no lidar3d_experimental_node")
        elif status == "ok":
            ng = int(dbg.get("n_non_ground", 0))
            g = int(dbg.get("n_ground", 0))
            vz = int(dbg.get("n_voxel", 0))
            nc = int(dbg.get("n_clusters", 0))
            if vz > 0 and g == 0 and ng == 0:
                issues.append("ROOT: CSF devolveu 0 ground e 0 non-ground — CSF params ou crash interno")
            elif vz > 0 and ng > 0 and nc == 0:
                issues.append(
                    f"ROOT: clustering produziu 0 clusters com {ng} non-ground pts — "
                    "min_cluster_size/tolerance"
                )
            elif self._topic_stats[TOPICS_REAL["ground"]].msgs == 0:
                issues.append(
                    "ROOT: status ok no JSON mas tópico ground sem mensagens — QoS/subscriber?"
                )

        if not issues:
            g_msgs = self._topic_stats[TOPICS_REAL["ground"]].msgs
            if g_msgs > 0 and self._trees.msgs > 0:
                print(
                    "  Pipeline + tree_landmarks activos — gate DBH/recall medido acima"
                )
            elif g_msgs > 0:
                print("  Pipeline publica dados — tree_landmarks ainda vazio")
            else:
                issues.append("Indeterminado — correr com debug_stage:=1/2/3 isolado")

        if issues:
            self._failed = True
        for i, line in enumerate(issues, 1):
            print(f"  [{i}] {line}")

        print("\n--- Testes binários (ros2 param) ---")
        print("  Test1 voxel:  ros2 param set /lidar3d_experimental_node debug_stage 1")
        print("  Test2 CSF:    ros2 param set /lidar3d_experimental_node debug_stage 2")
        print("  Test3 cluster: ros2 param set /lidar3d_experimental_node debug_stage 3")
        print("  Test4 full:   ros2 param set /lidar3d_experimental_node debug_stage 0")
        print("\n--- Comando gate perceção (Agente 2) ---")
        print("  forest up sim-lidar3d-experimental -d --world forest_rugged_trees_rocks")
        print("  forest test lidar3d-experimental 60")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    node = ExpPipelineAudit(args.duration)
    failed = False
    try:
        rclpy.spin(node)
        failed = node._failed
    except KeyboardInterrupt:
        failed = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

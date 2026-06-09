#!/usr/bin/env python3
"""Audit experimental LiDAR pipeline — localiza onde a funnel quebra.

Verifica:
  - nó lidar3d_experimental_node activo
  - tópicos reais vs nomes errados (/ground_points não existe)
  - debug_stats JSON (status, contagens por etapa)
  - hz e pontos por tópico

Testes manuais (param debug_stage no nó):
  1 = só voxel publicado em ground
  2 = só CSF (ground + non_ground)
  3 = só clustering (CSF bypass, voxel -> cluster)
  0/4 = pipeline completa

Uso:
  forest up sim-lidar3d-experimental -d --world forest_rugged_trees_rocks
  python3 tools/diagnostics/lidar3d_experimental_pipeline_audit.py --duration 20
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


TOPICS_REAL = {
    "input": "/sensors/lidar/points",
    "ground": "/perception/lidar3d/experimental/ground",
    "non_ground": "/perception/lidar3d/experimental/non_ground",
    "clusters": "/perception/lidar3d/experimental/clusters",
    "tree_candidates": "/perception/lidar3d/experimental/tree_candidates",
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
        self._failed = False

        qos = qos_profile_sensor_data
        for key, topic in TOPICS_REAL.items():
            if key == "debug_stats":
                self.create_subscription(String, topic, self._on_debug, 10)
            else:
                self.create_subscription(
                    PointCloud2, topic, lambda m, t=topic: self._on_cloud(m, t), qos
                )

        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Experimental pipeline audit ({duration:.0f}s)...")

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

    def _tick(self) -> None:
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            rclpy.shutdown()

    def _report(self) -> None:
        print("\n" + "=" * 70)
        print("  EXPERIMENTAL LiDAR PIPELINE AUDIT")
        print("=" * 70)

        self._check_node()
        self._check_wrong_topics()
        print("\n--- Tópicos reais (contagens) ---")
        for key, topic in TOPICS_REAL.items():
            st = self._topic_stats[topic]
            print(f"  {topic}")
            print(f"    msgs={st.msgs}  pts_total={st.pts}  last_width={st.last_width}")

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
            print("\n>>> AUDIT OK")

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
            print("      Use: forest up sim-lidar3d-experimental -d")
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
            if g_msgs > 0:
                print("  Pipeline publica dados — se RViz vazio, ver Fixed Frame=map e tópicos Exp/*")
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

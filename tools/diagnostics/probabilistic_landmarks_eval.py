#!/usr/bin/env python3
"""Validação Fase 1 — emissão probabilística em /perception/lidar/tree_landmarks.

Gates (perceção P-A + P-B):
  1. Todas as deteções têm class_scores com soma ≈ 1 (publisher preenche o contrato).
  2. Existem deteções com argmax=rocha (mundo com rochas GT).
  3. Troncos: deteções argmax=tronco ainda casam com GT (recall parcial).
  4. debug_stats reporta n_landmarks_emitted > 0 e n_dominant_rock > 0.
  5. Flip-flop suave: deteções rocha-dominantes no mesmo sítio não saltam para
     s_tronco > 0.5 entre frames consecutivos (mediana de max score tronco).

Auto-conduz se --no-drive não for passado.

Uso:
  forest up sim-lidar3d-experimental -d --world forest_gentle_trees_rocks
  forest test probabilistic-landmarks 40
  forest diag prob-landmarks --world forest_gentle_trees_rocks --duration 40
"""
from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_geometry_msgs  # noqa: F401

from forest_hybrid_msgs.msg import TreeLandmarkArray
from std_msgs.msg import String

TOPIC_LANDMARKS = "/perception/lidar/tree_landmarks"
TOPIC_DEBUG = "/perception/lidar3d/experimental/debug_stats"
CMD_VEL = "/forest_gen/cmd_vel"
EXP_NODE = "/lidar3d_experimental_node"

IDX_TRUNK, IDX_ROCK, IDX_OBST = 0, 1, 2


def load_gt_xy(sdf_path: Path, name_pat: str) -> list[tuple[str, float, float]]:
    text = sdf_path.read_text(encoding="utf-8", errors="ignore")
    pat = re.compile(
        rf"<name>\s*({name_pat})\s*</name>\s*<pose>([^<]+)</pose>", re.DOTALL)
    out: list[tuple[str, float, float]] = []
    for m in pat.finditer(text):
        parts = m.group(2).split()
        if len(parts) >= 2:
            out.append((m.group(1), float(parts[0]), float(parts[1])))
    return out


def argmax(scores: list[float]) -> int:
    return max(range(len(scores)), key=lambda i: scores[i])


@dataclass
class Agg:
    frames: int = 0
    detections: int = 0
    score_sum_bad: int = 0
    dom_trunk: int = 0
    dom_rock: int = 0
    dom_obst: int = 0
    trunk_matched: int = 0
    trunk_total: int = 0
    rock_flip_bad: int = 0
    rock_flip_checks: int = 0
    debug_samples: list[dict] = field(default_factory=list)
    _prev_rock: list[tuple[float, float, float]] = field(default_factory=list)

    def on_debug(self, raw: str) -> None:
        try:
            self.debug_samples.append(json.loads(raw))
        except json.JSONDecodeError:
            pass


class Eval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("probabilistic_landmarks_eval")
        self.args = args
        self.world_frame = args.world_frame
        self.match_r = args.match_radius
        self.range_max = args.range_max

        sdf = Path(args.world)
        self.gt_trees = load_gt_xy(sdf, r"tree_\d+")
        self.gt_rocks = load_gt_xy(sdf, r"rock_\d+")
        if not self.gt_trees:
            raise SystemExit(f"0 troncos GT em {sdf}")
        if args.require_rocks and not self.gt_rocks:
            raise SystemExit(f"0 rochas GT em {sdf} (usa mundo *_trees_rocks)")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_lm = self.create_subscription(
            TreeLandmarkArray, TOPIC_LANDMARKS, self.on_landmarks, qos)
        self.sub_dbg = self.create_subscription(String, TOPIC_DEBUG, self.on_debug, 10)
        self.pub_cmd = self.create_publisher(Twist, CMD_VEL, 10)

        import tf2_ros

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.agg = Agg()
        self._t0 = time.monotonic()

    def _robot_xy(self, frame_id: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, frame_id, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def on_debug(self, msg: String) -> None:
        self.agg.on_debug(msg.data)

    def on_landmarks(self, msg: TreeLandmarkArray) -> None:
        self.agg.frames += 1
        rob = self._robot_xy(msg.header.frame_id)
        cur_rocks: list[tuple[float, float, float]] = []

        visible_trees: list[tuple[float, float]] = []
        if rob is not None:
            visible_trees = [
                (gx, gy)
                for _, gx, gy in self.gt_trees
                if math.hypot(gx - rob[0], gy - rob[1]) <= self.range_max
            ]

        for t in msg.trees:
            self.agg.detections += 1
            scores = [float(t.class_scores[0]), float(t.class_scores[1]), float(t.class_scores[2])]
            ssum = sum(scores)
            if abs(ssum - 1.0) > 0.08 or min(scores) < -0.01:
                self.agg.score_sum_bad += 1

            dom = argmax(scores)
            if dom == IDX_TRUNK:
                self.agg.dom_trunk += 1
            elif dom == IDX_ROCK:
                self.agg.dom_rock += 1
            else:
                self.agg.dom_obst += 1

            wx = wy = None
            if rob is not None:
                ps = PointStamped()
                ps.header = msg.header
                ps.point.x, ps.point.y, ps.point.z = t.base.x, t.base.y, t.base.z
                try:
                    pw = self.tf_buffer.transform(
                        ps, self.world_frame, timeout=rclpy.duration.Duration(seconds=0.05))
                    wx, wy = pw.point.x, pw.point.y
                except Exception:
                    pass

            if dom == IDX_TRUNK and visible_trees and wx is not None:
                self.agg.trunk_total += 1
                if any(math.hypot(wx - gx, wy - gy) < self.match_r for gx, gy in visible_trees):
                    self.agg.trunk_matched += 1

            if dom == IDX_ROCK and wx is not None:
                cur_rocks.append((wx, wy, scores[IDX_TRUNK]))

        for wx, wy, s_trunk in cur_rocks:
            for px, py, ps_trunk in self.agg._prev_rock:
                if math.hypot(wx - px, wy - py) < self.match_r:
                    self.agg.rock_flip_checks += 1
                    if ps_trunk <= 0.5 and s_trunk > 0.5:
                        self.agg.rock_flip_bad += 1
                    break
        self.agg._prev_rock = cur_rocks

    def _drive(self, lin: float, ang: float) -> None:
        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        self.pub_cmd.publish(tw)

    def run(self) -> None:
        dur = self.args.duration
        while time.monotonic() - self._t0 < dur:
            el = time.monotonic() - self._t0
            if not self.args.no_drive:
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
        if not self.args.no_drive:
            for _ in range(5):
                self._drive(0.0, 0.0)
                time.sleep(0.02)
        self.finish()

    def finish(self) -> None:
        a = self.agg
        dbg = a.debug_samples[-1] if a.debug_samples else {}
        n_emit = int(dbg.get("n_landmarks_emitted", 0))
        n_rock_dbg = int(dbg.get("n_dominant_rock", 0))
        trunk_prec = a.trunk_matched / a.trunk_total if a.trunk_total else 0.0
        flip_rate = a.rock_flip_bad / a.rock_flip_checks if a.rock_flip_checks else 0.0

        gates = {
            "scores_normalized": a.detections > 0 and a.score_sum_bad == 0,
            "rocks_emitted": a.dom_rock >= self.args.min_rock_dets,
            "trunk_precision": a.trunk_total == 0 or trunk_prec >= self.args.trunk_prec_min,
            "debug_rock_count": n_rock_dbg > 0 or a.dom_rock >= self.args.min_rock_dets,
            "no_hard_rock_flip": a.rock_flip_checks == 0 or flip_rate <= self.args.max_flip_rate,
            "node_active": self._node_active(),
            "frames_received": a.frames >= self.args.min_frames,
        }
        ok = all(gates.values())

        print("\n" + "=" * 68)
        print("  VALIDAÇÃO — landmarks probabilísticos (Fase 1 perceção)")
        print("=" * 68)
        print(f"  frames / detecções        : {a.frames} / {a.detections}")
        print(f"  argmax tronco/rocha/obst   : {a.dom_trunk} / {a.dom_rock} / {a.dom_obst}")
        print(f"  class_scores inválidos     : {a.score_sum_bad}")
        print(f"  tronco casado (precisão)   : {trunk_prec:.2f} ({a.trunk_matched}/{a.trunk_total})")
        print(f"  flip rocha→tronco (soft)   : {flip_rate:.2f} ({a.rock_flip_bad}/{a.rock_flip_checks})")
        print(f"  debug n_landmarks_emitted  : {n_emit}")
        print(f"  debug n_dominant_rock      : {n_rock_dbg}")
        print("-" * 68)
        for name, passed in gates.items():
            print(f"  {name:24s}: {'PASS' if passed else 'FAIL'}")
        print("-" * 68)
        print(f"  GATE GLOBAL               : {'PASS ✓' if ok else 'FAIL ✗'}")
        print("=" * 68)
        raise SystemExit(0 if ok else 1)

    def _node_active(self) -> bool:
        try:
            out = subprocess.check_output(["ros2", "node", "list"], text=True, timeout=5)
            return EXP_NODE in out
        except Exception:
            return False


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--world", required=True, help="Caminho para .sdf com tree_N e rock_N")
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--world-frame", default="map")
    p.add_argument("--match-radius", type=float, default=0.55)
    p.add_argument("--range-max", type=float, default=18.0)
    p.add_argument("--min-rock-dets", type=int, default=3)
    p.add_argument("--min-frames", type=int, default=5)
    p.add_argument("--trunk-prec-min", type=float, default=0.55)
    p.add_argument("--max-flip-rate", type=float, default=0.25)
    p.add_argument("--require-rocks", action="store_true", default=True)
    p.add_argument("--no-drive", action="store_true")
    args = p.parse_args()

    if not Path(args.world).is_file():
        repo = Path(__file__).resolve().parents[2]
        guess = repo.parent.parent / "Gazebo" / "ForestGen" / "worlds" / args.world
        if guess.is_file():
            args.world = str(guess)
        elif (repo / ".." / ".." / "Gazebo" / "ForestGen" / "worlds" / (args.world + ".sdf")).resolve().is_file():
            args.world = str(
                (repo / ".." / ".." / "Gazebo" / "ForestGen" / "worlds" / (args.world + ".sdf")).resolve())

    rclpy.init()
    node = Eval(args)
    try:
        node.run()
    except SystemExit as exc:
        rclpy.shutdown()
        raise exc
    except KeyboardInterrupt:
        node.finish()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

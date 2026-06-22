#!/usr/bin/env python3
"""Testa a assimetria FRENTE vs ATRÁS da deteção de troncos (LiDAR inclinado).

Hipótese: com o LiDAR inclinado, atrás do robô os raios sobem e NÃO há retornos
de chão; a region growing (ancorada ao chão) não consegue semear → árvores atrás
(quadrantes 3 e 4, x<0 em base_link) não são detetadas mesmo estando visíveis.

Método controlado: roda o robô no SÍTIO (mesma cena, todos os bearings). Para cada
direção (8 setores de 45°) conta: pontos de CHÃO, pontos de NÃO-CHÃO e DETEÇÕES de
tronco. Se atrás houver não-chão (a árvore está lá) mas ~0 chão e ~0 deteções, o
problema é a falta de chão atrás, não falta de alvo.

Uso:
  forest up sim-tree-slam -d --world forest_rugged_trees_rocks
  python3 rear_detection_eval.py --duration 50
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from forest_hybrid_msgs.msg import TreeLandmarkArray

CMD_VEL = "/forest_gen/cmd_vel"
GROUND = "/perception/lidar3d/experimental/ground"
NON_GROUND = "/perception/lidar3d/experimental/non_ground"
LANDMARKS = "/perception/lidar/tree_landmarks"
NSECT = 8  # setores de 45°


def sector(x: float, y: float) -> int:
    a = math.atan2(y, x)  # bearing em base_link; 0 = frente, ±pi = atrás
    return int(((a + math.pi) / (2 * math.pi) * NSECT)) % NSECT


SECTOR_LABEL = {
    0: "ATRÁS  (±180°)", 1: "tras-dir", 2: "DIREITA(-90°)", 3: "frente-dir",
    4: "FRENTE (0°)", 5: "frente-esq", 6: "ESQ   (+90°)", 7: "tras-esq",
}


class RearEval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rear_detection_eval")
        self.args = args
        self.t0 = time.monotonic()
        self.g = [0] * NSECT      # pontos de chão por setor (somados nos frames)
        self.ng = [0] * NSECT     # pontos não-chão por setor
        self.det = [0] * NSECT    # deteções de tronco por setor
        self.g_frames = 0
        self.ng_frames = 0
        self.create_subscription(PointCloud2, GROUND, self._on_ground, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, NON_GROUND, self._on_non_ground,
                                 qos_profile_sensor_data)
        self.create_subscription(TreeLandmarkArray, LANDMARKS, self._on_lm, 10)
        self.cmd = self.create_publisher(Twist, CMD_VEL, 10)
        self.create_timer(0.1, self._tick)

    def _bin_cloud(self, msg: PointCloud2, acc: list[int]) -> None:
        for x, y, _z in point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                                  skip_nans=True):
            acc[sector(float(x), float(y))] += 1

    def _on_ground(self, msg: PointCloud2) -> None:
        self.g_frames += 1
        self._bin_cloud(msg, self.g)

    def _on_non_ground(self, msg: PointCloud2) -> None:
        self.ng_frames += 1
        self._bin_cloud(msg, self.ng)

    def _on_lm(self, msg: TreeLandmarkArray) -> None:
        for t in msg.trees:
            self.det[sector(t.base.x, t.base.y)] += 1

    def _tick(self) -> None:
        t = time.monotonic() - self.t0
        if not self.args.no_drive:
            m = Twist()
            m.angular.z = 0.45  # roda no sítio
            self.cmd.publish(m)
        if t >= self.args.duration:
            self.cmd.publish(Twist())
            self._report()
            raise SystemExit(0)

    def _report(self) -> None:
        gf = max(self.g_frames, 1)
        nf = max(self.ng_frames, 1)
        print("=" * 70)
        print("  ASSIMETRIA FRENTE vs ATRÁS — deteção de troncos (LiDAR inclinado)")
        print("=" * 70)
        print(f"  frames chão={self.g_frames} não-chão={self.ng_frames}")
        print(f"  {'setor':<16}{'chão/frame':>12}{'nãochão/frame':>15}{'deteções':>11}")
        for s in range(NSECT):
            print(f"  {SECTOR_LABEL[s]:<16}{self.g[s]/gf:>12.0f}"
                  f"{self.ng[s]/nf:>15.0f}{self.det[s]:>11}")
        # Resumo frente (setores 3,4,5) vs atrás (7,0,1)
        front = [3, 4, 5]
        rear = [7, 0, 1]
        gf_front = sum(self.g[s] for s in front) / gf
        gf_rear = sum(self.g[s] for s in rear) / gf
        ng_front = sum(self.ng[s] for s in front) / nf
        ng_rear = sum(self.ng[s] for s in rear) / nf
        d_front = sum(self.det[s] for s in front)
        d_rear = sum(self.det[s] for s in rear)
        print("-" * 70)
        print(f"  FRENTE: chão/f={gf_front:.0f}  nãochão/f={ng_front:.0f}  deteções={d_front}")
        print(f"  ATRÁS : chão/f={gf_rear:.0f}  nãochão/f={ng_rear:.0f}  deteções={d_rear}")
        print("-" * 70)
        if ng_rear > 0.3 * ng_front and gf_rear < 0.2 * max(gf_front, 1) and d_rear < 0.2 * max(d_front, 1):
            print("  VEREDITO: CONFIRMADO — atrás HÁ alvo (não-chão) mas falta CHÃO e há ~0")
            print("            deteções. Causa = sem chão atrás (raios sobem) → region")
            print("            growing não semeia. Problema físico-geométrico.")
        elif gf_rear >= 0.5 * max(gf_front, 1) and d_rear < 0.3 * max(d_front, 1):
            print("  VEREDITO: há chão atrás mas poucas deteções → problema na region")
            print("            growing/HAG atrás, não no sensor (fixável em software).")
        else:
            print("  VEREDITO: assimetria fraca nesta corrida (poucos alvos atrás?).")
        print("=" * 70)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=50.0)
    p.add_argument("--no-drive", action="store_true")
    args = p.parse_args()
    rclpy.init()
    node = RearEval(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Compara TRÊS trajetórias do robô e gera gráficos: ground-truth do Gazebo,
pose só do EKF (IMU+odometria, dead-reckoning) e pose corrigida pelo Tree-SLAM.

Arquitetura (porque há três e como se ligam):
  - O EKF (`ekf_local`) funde SÓ roda + IMU e publica a TF `odom -> base_link`.
    NÃO consome o SLAM. É dead-reckoning: deriva sem limite.
  - O Tree-SLAM publica a TF `map -> odom` (a "correção"). NÃO entra no EKF;
    compõe-se com ele via TF. A pose completa corrigida = map->odom (SLAM) ∘
    odom->base (EKF) = TF `map -> base_link`.
  - A ground-truth é a pose real do modelo no mundo Gazebo (`world_tf`).

Como o robô nasce na origem do mundo e os frames `map`/`odom` inicializam aí,
as três trajetórias partilham a origem em t0 → comparam-se diretamente (a
divergência ao longo do tempo é a deriva). Se o SLAM funciona, a curva SLAM
fica colada à GT enquanto a EKF-only diverge.

Saídas: PNG (trajetórias XY + erro vs tempo) e CSV. Auto-conduz o robô.

Uso:
  forest up sim-tree-slam -d --world forest_gentle_trees_rocks
  python3 slam_trajectory_eval.py --duration 60 --out /tmp/slam_traj.png
"""
from __future__ import annotations

import argparse
import csv
import math
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import rclpy  # noqa: E402
import tf2_ros  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from forest_hybrid_msgs.msg import TrackedTreeLandmarkArray  # noqa: E402

CMD_VEL = "/forest_gen/cmd_vel"

# Rota em loop fechado para forest_rugged_trees_rocks (~50x50 m, robô nasce em 0,0).
# Passa pela zona sul densa (até ~20 landmarks/ponto) e VOLTA à origem → testa loop
# closure. Conduzida por GT (a navegação não é o que está sob teste; o estimado é).
# Loop validado por folga: cada segmento fica a ≥3.0 m de TODAS as 50 árvores e
# 20 rochas do rugged (gerado por procura de folga; o robô tem ~0.6 m de largura).
# Compacto (~6 m) porque a floresta é densa perto da origem — orbita os landmarks
# centrais e volta a (0,0) para testar loop closure, sem cruzar obstáculos.
ROUTES = {
    "rugged-loop": [(0.0, 0.0), (5.6, 2.1), (-2.1, 5.6), (-5.6, -2.1),
                    (2.1, -5.6), (0.0, 0.0)],
}


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def load_world_landmarks(sdf_path):
    """Lê (kind, name, x, y) de árvores e rochas do SDF do mundo."""
    import re
    from pathlib import Path
    txt = Path(sdf_path).read_text(errors="ignore")
    blocks = re.findall(
        r'<uri>model://(Tree\d|Rock\d)</uri>\s*<name>([^<]+)</name>\s*<pose>([^<]+)</pose>', txt)
    trees, rocks = [], []
    for kind, name, pose in blocks:
        pp = pose.split()
        x, y = float(pp[0]), float(pp[1])
        (trees if kind.startswith("Tree") else rocks).append((name, x, y))
    return trees, rocks


class TrajEval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("slam_trajectory_eval")
        self.args = args
        self.base = args.base_frame
        self.gt = None  # (x, y, yaw) mais recente da GT
        self.t0 = time.monotonic()
        # séries: lista de (t, x, y, yaw)
        self.s_gt: list[tuple[float, float, float, float]] = []
        self.s_ekf: list[tuple[float, float, float, float]] = []
        self.s_slam: list[tuple[float, float, float, float]] = []

        # Estado da rota por waypoints (modo --route).
        self.route = ROUTES.get(args.route)
        self.wp_idx = 0
        self.lap = 0
        self.lap_start_sample = [0]  # índice em s_gt onde cada volta começa

        # Landmarks do mundo (GT) e do SLAM, para o mapa.
        self.world_trees, self.world_rocks = ([], [])
        if args.world_sdf:
            try:
                self.world_trees, self.world_rocks = load_world_landmarks(args.world_sdf)
            except Exception as e:  # noqa: BLE001
                print(f"AVISO: não li o SDF do mundo: {e}")
        self.slam_landmarks: list[tuple[int, float, float]] = []  # (semantic_class, x, y)

        # Baseline de churn de uid (Passo 0): se a associação/loop closure
        # funcionasse, o nº de uids distintos no mapa ≈ nº real de landmarks;
        # sem loop closure, cresce sem parar (cada re-visita = uid novo).
        self.uids_ever: set[int] = set()      # todos os uids vistos em /slam/tree_map
        self.n_landmarks_series: list[tuple[float, int]] = []  # (t, nº simultâneo)
        self.max_uid_seen = 0

        # Verifica que a rota não cruza obstáculos (folga mínima de cada segmento).
        if self.route is not None and (self.world_trees or self.world_rocks):
            pts = [(x, y) for _, x, y in self.world_trees + self.world_rocks]
            mc = min(self._seg_clear(self.route[i], self.route[i + 1], pts)
                     for i in range(len(self.route) - 1))
            print(f"[rota] folga mínima do percurso a árvores/rochas: {mc:.2f} m"
                  + ("  ⚠️ <1 m — risco de embate!" if mc < 1.0 else "  OK"))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(TFMessage, "/forest_gen/gz/world_tf", self._on_gt, 10)
        self.create_subscription(TrackedTreeLandmarkArray, "/slam/tree_map", self._on_map, 10)
        self.cmd = self.create_publisher(Twist, CMD_VEL, 10)
        self.create_timer(0.1, self._tick)  # 10 Hz

    def _on_map(self, msg) -> None:
        self.slam_landmarks = [(t.semantic_class, t.position.x, t.position.y) for t in msg.trees]
        for t in msg.trees:
            self.uids_ever.add(int(t.uid))
            self.max_uid_seen = max(self.max_uid_seen, int(t.uid))
        self.n_landmarks_series.append((time.monotonic() - self.t0, len(msg.trees)))

    @staticmethod
    def _seg_clear(a, b, pts) -> float:
        ax, ay = a; bx, by = b; dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        m = 1e9
        for px, py in pts:
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if d < m:
                m = d
        return m

    def _on_gt(self, msg: TFMessage) -> None:
        # O robô (base_link) é a transform com z ~ altura da base (~0.55 m).
        best = None
        for tr in msg.transforms:
            t = tr.transform.translation
            if 0.35 <= t.z <= 0.80:
                best = tr
                break
        if best is None and msg.transforms:
            best = msg.transforms[0]
        if best is not None:
            t = best.transform.translation
            q = best.transform.rotation
            self.gt = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def _lookup(self, parent: str):
        try:
            tf = self.tf_buffer.lookup_transform(parent, self.base, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        except Exception:
            return None

    def _drive(self) -> None:
        if self.args.no_drive:
            return
        if self.route is not None:
            self._drive_route()
            return
        frac = (time.monotonic() - self.t0) / self.args.duration * 100.0
        m = Twist()
        if frac < 25:
            m.linear.x = 0.35           # reta
        elif frac < 35:
            m.angular.z = 0.5           # roda no sítio (muda de vista)
        elif frac < 70:
            m.linear.x = 0.3; m.angular.z = 0.25   # arco
        elif frac < 80:
            m.angular.z = -0.5
        else:
            m.linear.x = 0.35
        self.cmd.publish(m)

    def _drive_route(self) -> None:
        """Segue os waypoints da rota por controlo proporcional (usa GT p/ navegar)."""
        if self.gt is None:
            return
        gx, gy, gyaw = self.gt
        wx, wy = self.route[self.wp_idx]
        dx, dy = wx - gx, wy - gy
        dist = math.hypot(dx, dy)
        if dist < self.args.wp_tol:
            self.wp_idx += 1
            if self.wp_idx >= len(self.route):  # completou uma volta
                self.wp_idx = 1 if len(self.route) > 1 else 0  # salta o (0,0) inicial repetido
                self.lap += 1
                self.lap_start_sample.append(len(self.s_gt))
            return
        yaw_err = wrap(math.atan2(dy, dx) - gyaw)
        m = Twist()
        if abs(yaw_err) > 0.4:
            m.angular.z = 0.6 * (1 if yaw_err > 0 else -1)   # vira para o waypoint
        else:
            m.linear.x = min(0.5, 0.4 * dist)
            m.angular.z = max(-0.8, min(0.8, 1.2 * yaw_err))  # corrige rumo a andar
        self.cmd.publish(m)

    def _tick(self) -> None:
        self._drive()
        t = time.monotonic() - self.t0
        # Fim antecipado por nº de voltas (modo rota).
        if self.route is not None and self.lap >= self.args.laps:
            self._finish()
            raise SystemExit(0)
        slam = self._lookup(self.args.map_frame)
        ekf = self._lookup(self.args.odom_frame)
        if self.gt is not None and slam is not None and ekf is not None:
            self.s_gt.append((t, *self.gt))
            self.s_slam.append((t, *slam))
            self.s_ekf.append((t, *ekf))
        if t >= self.args.duration:
            self._finish()
            raise SystemExit(0)

    def _finish(self) -> None:
        self.cmd.publish(Twist())  # parar
        n = len(self.s_gt)
        if n < 5:
            print(f"AMOSTRAS INSUFICIENTES (n={n}). Verifica world_tf / TF map|odom->base.")
            return

        # Erros instantâneos vs GT (mesmos índices: capturados no mesmo tick).
        def err(series):
            return [math.hypot(s[1] - g[1], s[2] - g[2])
                    for s, g in zip(series, self.s_gt)]

        e_ekf = err(self.s_ekf)
        e_slam = err(self.s_slam)
        mean_ekf, mean_slam = sum(e_ekf) / n, sum(e_slam) / n
        fin_ekf, fin_slam = e_ekf[-1], e_slam[-1]
        max_ekf, max_slam = max(e_ekf), max(e_slam)

        # CSV
        csv_path = self.args.out.rsplit(".", 1)[0] + ".csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "gt_x", "gt_y", "ekf_x", "ekf_y", "slam_x", "slam_y",
                        "err_ekf", "err_slam"])
            for i in range(n):
                w.writerow([f"{self.s_gt[i][0]:.3f}",
                            f"{self.s_gt[i][1]:.3f}", f"{self.s_gt[i][2]:.3f}",
                            f"{self.s_ekf[i][1]:.3f}", f"{self.s_ekf[i][2]:.3f}",
                            f"{self.s_slam[i][1]:.3f}", f"{self.s_slam[i][2]:.3f}",
                            f"{e_ekf[i]:.3f}", f"{e_slam[i]:.3f}"])

        # Gráficos
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        gx = [s[1] for s in self.s_gt]; gy = [s[2] for s in self.s_gt]
        ex = [s[1] for s in self.s_ekf]; ey = [s[2] for s in self.s_ekf]
        sx = [s[1] for s in self.s_slam]; sy = [s[2] for s in self.s_slam]
        ax1.plot(gx, gy, "k-", lw=2.5, label="Ground-truth (Gazebo)")
        ax1.plot(ex, ey, "b--", lw=1.5, label="EKF only (IMU+odom, dead-reckoning)")
        ax1.plot(sx, sy, "r-", lw=1.5, label="Tree-SLAM (map→base)")
        ax1.plot(gx[0], gy[0], "go", ms=10, label="início")
        ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]")
        ax1.set_title("Trajetórias (mesma origem em t0)")
        ax1.axis("equal"); ax1.grid(True, alpha=0.3); ax1.legend()

        ts = [s[0] for s in self.s_gt]
        ax2.plot(ts, e_ekf, "b--", lw=1.5, label=f"EKF only (méd {mean_ekf:.2f} m)")
        ax2.plot(ts, e_slam, "r-", lw=1.5, label=f"Tree-SLAM (méd {mean_slam:.2f} m)")
        ax2.set_xlabel("t [s]"); ax2.set_ylabel("erro de posição vs GT [m]")
        ax2.set_title("Erro vs ground-truth ao longo do tempo")
        ax2.grid(True, alpha=0.3); ax2.legend()

        fig.suptitle("Avaliação de localização: GT vs EKF-only vs Tree-SLAM", fontsize=13)
        fig.tight_layout()
        fig.savefig(self.args.out, dpi=110)

        print("=" * 64)
        print("  AVALIAÇÃO DE TRAJETÓRIA — GT vs EKF-only vs Tree-SLAM")
        print("=" * 64)
        print(f"  amostras alinhadas       : {n}")
        print(f"  erro médio   EKF-only    : {mean_ekf:.3f} m")
        print(f"  erro médio   Tree-SLAM   : {mean_slam:.3f} m")
        print(f"  erro final   EKF-only    : {fin_ekf:.3f} m")
        print(f"  erro final   Tree-SLAM   : {fin_slam:.3f} m")
        print(f"  erro máximo  EKF-only    : {max_ekf:.3f} m")
        print(f"  erro máximo  Tree-SLAM   : {max_slam:.3f} m")
        if mean_ekf > 1e-6:
            print(f"  melhoria do SLAM (médio) : {100*(1-mean_slam/mean_ekf):+.0f}%")
        print("-" * 64)
        verdict = "SLAM AJUDA" if mean_slam < mean_ekf else "SLAM NÃO MELHORA (investigar)"
        print(f"  VEREDITO                 : {verdict}")
        # Erro por volta (modo rota): mostra se o loop closure reduz a deriva.
        if self.route is not None and self.lap >= 1:
            print("-" * 64)
            print(f"  voltas completas: {self.lap}")
            bounds = self.lap_start_sample + [n]
            for lap in range(min(self.lap, len(bounds) - 1)):
                a, b = bounds[lap], bounds[lap + 1]
                if b > a:
                    lap_slam = sum(e_slam[a:b]) / (b - a)
                    lap_ekf = sum(e_ekf[a:b]) / (b - a)
                    print(f"   volta {lap+1}: erro médio SLAM={lap_slam:.3f} m  EKF={lap_ekf:.3f} m")
        # ---- Baseline de CHURN DE UID (Passo 0) ----
        # Métrica-raiz do bug #1 (sem loop closure): nº de uids distintos no
        # mapa vs nº real de landmarks. Sem associação persistente, cada
        # re-visita gera uid novo → uids_distintos >> landmarks reais.
        n_true = len(self.world_trees) + len(self.world_rocks)
        n_distinct = len(self.uids_ever)
        n_simul_max = max((c for _, c in self.n_landmarks_series), default=0)
        n_simul_fin = self.n_landmarks_series[-1][1] if self.n_landmarks_series else 0
        print("-" * 64)
        print("  CHURN DE UID (baseline Passo 0 — comparar após correções)")
        print(f"  uids distintos no mapa (total): {n_distinct}")
        print(f"  uid máximo observado          : {self.max_uid_seen}")
        print(f"  landmarks simultâneos (máx)   : {n_simul_max}")
        print(f"  landmarks simultâneos (final) : {n_simul_fin}")
        # Indicador de churn independente da rota: quantos tracks foram criados
        # por cada "slot" de landmark visível em simultâneo. Numa rota que
        # revisita o mesmo sítio, com loop closure deve ficar perto de 1; sem
        # ele, dispara (cada re-visita = uid novo).
        if n_simul_max:
            print(f"  CHURN = uid_máx / simult_máx  : {self.max_uid_seen / n_simul_max:.1f}x  "
                  "(ideal ~1-2; alto = re-visitas geram uids novos)")
        if n_true:
            print(f"  landmarks reais no mundo (SDF): {n_true}  "
                  "(rota compacta não visita todos)")
        print("=" * 64)
        print(f"  PNG trajetória: {self.args.out}")
        print(f"  CSV: {csv_path}")

        # ---- Mapa do percurso com árvores/rochas identificadas ----
        map_path = self.args.out.rsplit(".", 1)[0] + "_map.png"
        figm, axm = plt.subplots(figsize=(10, 10))
        # GT dos landmarks do mundo
        for name, x, y in self.world_trees:
            axm.plot(x, y, "^", color="forestgreen", ms=9, mec="k", mew=0.4)
            idx = name.split("_")[-1]
            axm.annotate(idx, (x, y), fontsize=6, color="darkgreen",
                         xytext=(2, 2), textcoords="offset points")
        for name, x, y in self.world_rocks:
            axm.plot(x, y, "o", color="dimgray", ms=8, mec="k", mew=0.4)
            idx = name.split("_")[-1]
            axm.annotate("R" + idx, (x, y), fontsize=6, color="black",
                         xytext=(2, 2), textcoords="offset points")
        # Landmarks que o SLAM detetou (em map ≈ mundo)
        for sem, x, y in self.slam_landmarks:
            mk = "x" if sem != 6 else "+"
            col = "red" if sem != 6 else "purple"
            axm.plot(x, y, mk, color=col, ms=11, mew=2)
        # Rota planeada (waypoints)
        if self.route is not None:
            rx = [w[0] for w in self.route]; ry = [w[1] for w in self.route]
            axm.plot(rx, ry, "c--", lw=1.2, alpha=0.7, label="rota planeada")
            for i, (wx, wy) in enumerate(self.route):
                axm.plot(wx, wy, "cs", ms=7)
                axm.annotate(f"W{i}", (wx, wy), fontsize=8, color="teal",
                             xytext=(4, -8), textcoords="offset points")
        # Trajetórias
        axm.plot(gx, gy, "k-", lw=2.0, label="GT percorrido")
        axm.plot(sx, sy, "r-", lw=1.2, alpha=0.8, label="SLAM (map→base)")
        axm.plot(gx[0], gy[0], "go", ms=12, label="início")
        # Legenda de símbolos
        from matplotlib.lines import Line2D
        handles = [
            Line2D([], [], marker="^", color="w", mfc="forestgreen", mec="k", ms=9, label="árvore GT (id)"),
            Line2D([], [], marker="o", color="w", mfc="dimgray", mec="k", ms=8, label="rocha GT (Rid)"),
            Line2D([], [], marker="x", color="red", ms=10, lw=0, label="tronco detetado (SLAM)"),
            Line2D([], [], marker="+", color="purple", ms=10, lw=0, label="rocha detetada (SLAM)"),
            Line2D([], [], color="k", lw=2, label="GT percorrido"),
            Line2D([], [], color="r", lw=1.2, label="SLAM percorrido"),
            Line2D([], [], color="c", ls="--", label="rota planeada"),
        ]
        axm.legend(handles=handles, loc="upper right", fontsize=8)
        axm.set_xlabel("x [m]"); axm.set_ylabel("y [m]")
        axm.set_title(f"Mapa do percurso — {len(self.world_trees)} árvores, "
                      f"{len(self.world_rocks)} rochas | SLAM detetou "
                      f"{len(self.slam_landmarks)} landmarks")
        axm.axis("equal"); axm.grid(True, alpha=0.3)
        figm.tight_layout()
        figm.savefig(map_path, dpi=120)
        print(f"  PNG mapa: {map_path}")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--out", default="/tmp/slam_trajectory.png")
    p.add_argument("--map-frame", default="map")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--base-frame", default="marble_hd2/base_link")
    p.add_argument("--no-drive", action="store_true")
    p.add_argument("--route", default=None, choices=list(ROUTES.keys()),
                   help="segue uma rota em loop por waypoints (ex.: rugged-loop)")
    p.add_argument("--laps", type=int, default=2, help="nº de voltas no modo --route")
    p.add_argument("--wp-tol", type=float, default=1.8, help="raio de chegada ao waypoint [m]")
    p.add_argument("--world-sdf", default=None,
                   help="SDF do mundo p/ desenhar árvores/rochas identificadas no mapa")
    args = p.parse_args()
    rclpy.init()
    node = TrajEval(args)
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

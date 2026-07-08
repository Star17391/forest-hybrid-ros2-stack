#!/usr/bin/env python3
"""Corrida AUTÓNOMA do Tree-SLAM num mundo realista + avaliação MAPA × GT.

Porquê: nos mundos realistas (forest_realistic_v2_*) as árvores estão espalhadas
(espaçamento ~6 m, a maioria a >10 m do robô) sobre terreno irregular. O robô TEM
de andar, e conduzir às cegas bate nos troncos. Este harness:

  1) lê a GT (posições dos troncos) do .sdf do mundo;
  2) PLANEIA um laço fechado que volta à origem, evitando as árvores (A* numa
     grelha de ocupação com as árvores infladas) — passando por 4 pontos de tour
     nos quadrantes para cobrir a floresta e testar loop closure;
  3) CONDUZ o robô pelos waypoints (vira-e-anda, feedback de /state/pose_fused);
  4) AVALIA o /slam/tree_map final contra a GT: recall, precisão, RMSE de posição,
     DUPLICADOS, falsos positivos, histograma de CONFIANÇAS e de classe, n_obs.

O /slam/tree_map está em frame `map`; o robô nasce na origem do mundo e map
inicializa aí → map ≈ world, logo compara-se diretamente com a GT (mesma
convenção dos outros eval em tools/diagnostics).

Uso:
  forest up sim-tree-slam -d --world forest_realistic_v2_trees_rocks
  python3 slam_race.py --world forest_realistic_v2_trees_rocks            # corre
  python3 slam_race.py --world forest_realistic_v2_trees_rocks --plan-only # só planeia
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import re
import time

FORESTGEN = os.environ.get("FORESTGEN_PATH", "/home/star17391/Projetos/Gazebo/ForestGen")


# ----------------------------- GT do mundo -----------------------------------
def load_gt(world: str):
    """Devolve [(name, species, x, y, z)] das árvores e [(x,y)] das rochas."""
    path = world if os.path.isabs(world) else os.path.join(FORESTGEN, "worlds", world)
    if not path.endswith(".sdf"):
        path += ".sdf"
    txt = open(path).read()
    trees, rocks = [], []
    for b in re.findall(r"<include>(.*?)</include>", txt, re.S):
        uri = re.search(r"model://([A-Za-z0-9_]+)", b)
        if not uri:
            continue
        m = uri.group(1)
        nm = re.search(r"<name>(.*?)</name>", b)
        po = re.search(r"<pose>(.*?)</pose>", b)
        nm = nm.group(1) if nm else m
        p = [float(v) for v in po.group(1).split()] if po else [0, 0, 0, 0, 0, 0]
        if m.startswith("Tree"):
            trees.append((nm, m, p[0], p[1], p[2]))
        elif m.startswith("Rock"):
            rocks.append((p[0], p[1]))
    return trees, rocks


def load_other_obstacles(world: str):
    """Modelos INLINE do mundo que NÃO são árvore/rocha (bush, fallen_log, stump):
    [(kind, x, y)]. São objetos reais que a perceção VÊ mas não constam dos
    <include> — sem isto, tracks sobre eles eram contados como 'fantasmas'."""
    path = world if os.path.isabs(world) else os.path.join(FORESTGEN, "worlds", world)
    if not path.endswith(".sdf"):
        path += ".sdf"
    txt = open(path).read()
    others = []
    for name, body in re.findall(r'<model name="([a-z][a-z_]*_\d+)">(.*?)</model>', txt, re.S):
        kind = re.sub(r"_\d+$", "", name)  # bush_3 -> bush ; fallen_log_1 -> fallen_log
        po = re.search(r"<pose>(.*?)</pose>", body)
        if not po:
            continue
        p = [float(v) for v in po.group(1).split()]
        others.append((kind, p[0], p[1]))
    return others


# ----------------------------- planeamento A* --------------------------------
class Grid:
    def __init__(self, obstacles, cell=0.5, margin=1.3, pad=4.0):
        xs = [o[0] for o in obstacles] or [0.0]
        ys = [o[1] for o in obstacles] or [0.0]
        self.cell = cell
        self.x0 = min(xs) - pad
        self.y0 = min(ys) - pad
        self.nx = int((max(xs) + pad - self.x0) / cell) + 1
        self.ny = int((max(ys) + pad - self.y0) / cell) + 1
        self.occ = [[False] * self.ny for _ in range(self.nx)]
        r_cells = int(math.ceil(margin / cell))
        for ox, oy in obstacles:
            ci, cj = self.to_cell(ox, oy)
            for di in range(-r_cells, r_cells + 1):
                for dj in range(-r_cells, r_cells + 1):
                    i, j = ci + di, cj + dj
                    if 0 <= i < self.nx and 0 <= j < self.ny and math.hypot(di, dj) <= r_cells:
                        self.occ[i][j] = True

    def to_cell(self, x, y):
        return int(round((x - self.x0) / self.cell)), int(round((y - self.y0) / self.cell))

    def to_world(self, i, j):
        return self.x0 + i * self.cell, self.y0 + j * self.cell

    def free(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny and not self.occ[i][j]

    def nearest_free(self, x, y):
        ci, cj = self.to_cell(x, y)
        if self.free(ci, cj):
            return ci, cj
        for rad in range(1, max(self.nx, self.ny)):
            for di in range(-rad, rad + 1):
                for dj in range(-rad, rad + 1):
                    if max(abs(di), abs(dj)) != rad:
                        continue
                    if self.free(ci + di, cj + dj):
                        return ci + di, cj + dj
        return ci, cj

    def astar(self, start, goal):
        s, g = self.nearest_free(*start), self.nearest_free(*goal)
        openh = [(0.0, s)]
        came, gsc = {s: None}, {s: 0.0}
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while openh:
            _, cur = heapq.heappop(openh)
            if cur == g:
                break
            for di, dj in nbrs:
                ni, nj = cur[0] + di, cur[1] + dj
                if not self.free(ni, nj):
                    continue
                step = math.hypot(di, dj)
                ng = gsc[cur] + step
                if (ni, nj) not in gsc or ng < gsc[(ni, nj)]:
                    gsc[(ni, nj)] = ng
                    f = ng + math.hypot(ni - g[0], nj - g[1])
                    heapq.heappush(openh, (f, (ni, nj)))
                    came[(ni, nj)] = cur
        if g not in came:
            return []
        path, c = [], g
        while c is not None:
            path.append(self.to_world(*c))
            c = came[c]
        return path[::-1]


def simplify(path, tol=1.0):
    """Reduz waypoints colineares (mantém só pontos de viragem)."""
    if len(path) < 3:
        return path
    out = [path[0]]
    for i in range(1, len(path) - 1):
        ax, ay = out[-1]
        bx, by = path[i]
        cx, cy = path[i + 1]
        # área do triângulo ~ colinearidade
        area = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))
        if area > tol:
            out.append(path[i])
    out.append(path[-1])
    return out


def plan_loop(trees, rocks, tour_radius=12.0):
    obstacles = [(t[2], t[3]) for t in trees] + list(rocks)
    grid = Grid(obstacles)
    # 4 pontos de tour nos quadrantes (NE, NW, SW, SE) + volta à origem.
    tour = [(0.0, 0.0)]
    for ang in (45, 135, 225, 315):
        a = math.radians(ang)
        tour.append((tour_radius * math.cos(a), tour_radius * math.sin(a)))
    tour.append((0.0, 0.0))
    wps = []
    for a, b in zip(tour[:-1], tour[1:]):
        seg = grid.astar(a, b)
        if not seg:
            continue
        wps.extend(simplify(seg)[:-1] if b != (0.0, 0.0) else simplify(seg))
    # clearance: distância mínima do percurso a qualquer árvore
    clr = min(
        (math.hypot(wx - ox, wy - oy) for wx, wy in wps for ox, oy in obstacles),
        default=99.0,
    )
    return wps, clr


# ----------------------------- avaliação MAPA × GT ---------------------------
def evaluate(gt_xy, map_subset, match_gate=1.5):
    """Greedy nearest match (map_subset → GT) dentro do gate. gt_xy = [(x,y)]."""
    used_gt = {}
    matches, false_pos = [], []
    for mt in map_subset:
        best, bd = None, match_gate
        for gi, (gx, gy) in enumerate(gt_xy):
            d = math.hypot(mt["x"] - gx, mt["y"] - gy)
            if d < bd:
                bd, best = d, gi
        if best is None:
            false_pos.append(mt)
        else:
            matches.append((best, mt, bd))
            used_gt.setdefault(best, []).append((mt, bd))
    dups = {gi: v for gi, v in used_gt.items() if len(v) > 1}
    pos_err = [bd for _, _, bd in matches]
    return {
        "gt_total": len(gt_xy),
        "map_total": len(map_subset),
        "recalled": len(used_gt),
        "recall": len(used_gt) / len(gt_xy) if gt_xy else 0.0,
        "false_positives": len(false_pos),
        "duplicated_gt": len(dups),
        "extra_dup_tracks": sum(len(v) - 1 for v in dups.values()),
        "pos_rmse_m": math.sqrt(sum(e * e for e in pos_err) / len(pos_err)) if pos_err else None,
        "pos_mean_m": sum(pos_err) / len(pos_err) if pos_err else None,
    }


def histogram(vals, edges):
    h = [0] * (len(edges) - 1)
    for v in vals:
        for k in range(len(edges) - 1):
            if edges[k] <= v < edges[k + 1] or (k == len(edges) - 2 and v == edges[-1]):
                h[k] += 1
                break
    return h


def _conf_hist(subset, label):
    confs = [m["confidence"] for m in subset]
    if not confs:
        return
    edges = [0.0, 0.5, 0.8, 0.95, 0.999, 1.0001]
    h = histogram(confs, edges)
    labels = ["[0,0.5)", "[0.5,0.8)", "[0.8,0.95)", "[0.95,1)", "==1.0"]
    print(f"  CONFIANÇAS {label} (fixas a 1.0: {h[-1]}/{len(confs)}):")
    for lab, c in zip(labels, h):
        print(f"      {lab:12} {c:3d}  {'#'*c}")


def _line(tag, mt):
    fp = mt["false_positives"]
    rmse = f"{mt['pos_rmse_m']:.2f}m" if mt["pos_rmse_m"] is not None else "—"
    print(f"  {tag}: recall {mt['recalled']}/{mt['gt_total']} ({100*mt['recall']:.0f}%) | "
          f"falsos+ {fp} | duplicados {mt['duplicated_gt']} GT (+{mt['extra_dup_tracks']} tracks) | "
          f"RMSE {rmse}")


def print_report(m_tree, m_rock, trunks, rock_tracks):
    print("\n==================== RELATÓRIO  MAPA-SLAM × GT ====================")
    _line("TRONCOS (cls 3)", m_tree)
    _line("ROCHAS  (cls 6)", m_rock)
    falsos_uid = (m_tree["false_positives"] + m_tree["extra_dup_tracks"]
                  + m_rock["false_positives"] + m_rock["extra_dup_tracks"])
    total = m_tree["map_total"] + m_rock["map_total"]
    print(f"  UIDs FALSOS (falsos+ e duplicados): {falsos_uid}/{total} tracks")
    _conf_hist(trunks, "troncos")
    _conf_hist(rock_tracks, "rochas")
    nobs = sorted(t["n_observations"] for t in (trunks + rock_tracks))
    if nobs:
        print(f"  n_observations: min={nobs[0]} mediana={nobs[len(nobs)//2]} max={nobs[-1]}")
    print("==================================================================\n")


# ----------------------------- condução (via Nav2 / camada de missão) --------
def run_drive(wps, args, gt_trees):
    """Conduz o laço AUTONOMAMENTE pela stack de navegação: manda UM
    CMD_PATROL_WAYPOINTS à camada de missão (mission_manager publica a rota →
    a ponte segue-a com NavigateThroughPoses) e espera /planning/goal_reached.
    Não há condução por cmd_vel aqui — a gestão de waypoints é da camada de missão.
    Recolhe /slam/tree_map e regista diagnósticos (LOST, bloqueios, cov, distância)."""
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    from forest_hybrid_msgs.msg import (
        TrackedTreeLandmarkArray, MissionCommand, SlamStatus)

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    class Racer(Node):
        def __init__(self):
            super().__init__("slam_race")
            self.cmd_pub = self.create_publisher(MissionCommand, "/mission/command", 10)
            self.create_subscription(PoseStamped, "/state/pose_fused", self.cb_pose, 20)
            self.create_subscription(
                TrackedTreeLandmarkArray, "/slam/tree_map", self.cb_map, 10)
            self.create_subscription(Bool, "/planning/goal_reached", self.cb_reached, 10)
            self.create_subscription(Bool, "/planning/path_blocked", self.cb_blocked, 10)
            self.create_subscription(SlamStatus, "/slam/status", self.cb_slam, 10)
            self.x = self.y = self.yaw = None
            self.map = []
            self.reached = False
            self.blocked_events = 0
            self.lost_events = 0          # nº de transições para LOST
            self._last_mode = None
            self.max_cov_trace = 0.0
            self.path_len = 0.0
            self._last_xy = None

        def cb_pose(self, m):
            self.x, self.y = m.pose.position.x, m.pose.position.y
            self.yaw = yaw_of(m.pose.orientation)
            if self._last_xy is not None:
                self.path_len += math.hypot(
                    self.x - self._last_xy[0], self.y - self._last_xy[1])
            self._last_xy = (self.x, self.y)

        def cb_map(self, m):
            self.map = [dict(uid=t.uid, x=t.position.x, y=t.position.y, diameter=t.diameter,
                             confidence=t.confidence, semantic_class=t.semantic_class,
                             n_observations=t.n_observations) for t in m.trees]

        def cb_reached(self, m):
            if m.data:
                self.reached = True

        def cb_blocked(self, m):
            if m.data:
                self.blocked_events += 1

        def cb_slam(self, m):
            if m.mode == SlamStatus.LOST and self._last_mode != SlamStatus.LOST:
                self.lost_events += 1
            self._last_mode = m.mode
            if m.pose_covariance_trace > self.max_cov_trace:
                self.max_cov_trace = float(m.pose_covariance_trace)

        def send_patrol(self, wps):
            c = MissionCommand()
            c.command_type = MissionCommand.CMD_PATROL_WAYPOINTS
            c.frame_type = MissionCommand.FRAME_MAP
            c.command_id = "slam_race_loop"
            c.source = "slam_race"
            c.waypoint_x = [float(x) for x, _ in wps]
            c.waypoint_y = [float(y) for _, y in wps]
            c.waypoint_z = [0.0 for _ in wps]
            self.cmd_pub.publish(c)

    rclpy.init()
    r = Racer()
    t0 = time.time()
    while r.x is None and time.time() - t0 < 30:
        rclpy.spin_once(r, timeout_sec=0.1)
    if r.x is None:
        print("!! sem /state/pose_fused — a sim está de pé? (forest up sim-tree-slam-nav2)")
        rclpy.shutdown()
        return None

    # NavigateThroughPoses verifica o ÚLTIMO waypoint com o goal_checker. Se o laço
    # fechar na origem (plan_loop faz wp_final == wp_inicial == arranque do robô), o
    # alvo final fica EM CIMA do robô → "já cheguei" em 0.3s sem andar. Retirar os
    # waypoints finais que coincidam (<1.5 m) com a pose inicial para o alvo final
    # ficar longe e o robô percorrer o laço todo.
    drive_wps = list(wps)
    while len(drive_wps) > 2 and math.hypot(
            drive_wps[-1][0] - r.x, drive_wps[-1][1] - r.y) < 1.5:
        drive_wps.pop()
    print(f"pose inicial=({r.x:.1f},{r.y:.1f}); a enviar PATROL com {len(drive_wps)}/"
          f"{len(wps)} waypoints à camada de missão (Nav2 conduz; último="
          f"({drive_wps[-1][0]:.1f},{drive_wps[-1][1]:.1f}))...")
    # Esperar o emparelhamento do subscritor (mission_manager) antes de publicar —
    # senão a 1.ª mensagem perde-se (corrida de discovery) e o robô nunca arranca.
    tw = time.time()
    while r.cmd_pub.get_subscription_count() < 1 and time.time() - tw < 10:
        rclpy.spin_once(r, timeout_sec=0.2)
    if r.cmd_pub.get_subscription_count() < 1:
        print("!! ninguém subscreve /mission/command — mission_manager vivo?")
        rclpy.shutdown()
        return None
    for _ in range(3):  # publicar algumas vezes p/ robustez
        r.send_patrol(drive_wps)
        rclpy.spin_once(r, timeout_sec=0.3)

    budget = args.timeout
    t0 = time.time()
    last_log = -1e9
    while rclpy.ok() and not r.reached and time.time() - t0 < budget:
        rclpy.spin_once(r, timeout_sec=0.1)
        el = time.time() - t0
        if el - last_log > 15.0:
            last_log = el
            print(f"  t+{el:4.0f}s pose=({r.x:5.1f},{r.y:5.1f}) | mapa={len(r.map):2d} tracks "
                  f"| LOST={r.lost_events} | bloqueios={r.blocked_events} | dist={r.path_len:.0f}m")
    status = "GOAL_REACHED" if r.reached else "TIMEOUT"
    print(f"\nlaço terminou: {status} em {time.time()-t0:.0f}s")
    for _ in range(20):
        rclpy.spin_once(r, timeout_sec=0.1)
    final_map = list(r.map)
    diag = dict(status=status, lost_events=r.lost_events, blocked_events=r.blocked_events,
                max_cov_trace=round(r.max_cov_trace, 4), path_len_m=round(r.path_len, 1))
    rclpy.shutdown()
    print(f"diagnóstico da condução: {diag}")
    return final_map, diag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="forest_realistic_v2_trees_rocks")
    ap.add_argument("--v", type=float, default=0.6)
    ap.add_argument("--w", type=float, default=0.6)
    ap.add_argument("--radius", type=float, default=12.0)
    ap.add_argument("--timeout", type=float, default=300.0,
                    help="orçamento global do laço (s) à espera de goal_reached")
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--out", default="/tmp/slam_race.json")
    args = ap.parse_args()

    trees, rocks = load_gt(args.world)
    others = load_other_obstacles(args.world)
    wps, clr = plan_loop(trees, rocks, args.radius)
    from collections import Counter
    oc = Counter(k for k, _, _ in others)
    print(f"GT: {len(trees)} árvores, {len(rocks)} rochas, "
          f"{len(others)} outros {dict(oc)} | mundo={args.world}")
    print(f"laço planeado: {len(wps)} waypoints | clearance mínima às árvores ≈ {clr:.2f} m")
    if args.plan_only or not wps:
        for k, (x, y) in enumerate(wps):
            print(f"  wp {k+1:2d}: ({x:6.1f},{y:6.1f})")
        return

    result = run_drive(wps, args, trees)
    if result is None:
        return
    final_map, diag = result
    trunks = [t for t in final_map if t["semantic_class"] == 3]
    rock_tracks = [t for t in final_map if t["semantic_class"] == 6]
    m_tree = evaluate([(t[2], t[3]) for t in trees], trunks)
    m_rock = evaluate(rocks, rock_tracks)
    print_report(m_tree, m_rock, trunks, rock_tracks)
    json.dump({"trees": m_tree, "rocks": m_rock, "map": final_map,
               "n_waypoints": len(wps), "drive": diag, "others": others},
              open(args.out, "w"), indent=2, default=str)
    print(f"guardado: {args.out}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Diagnóstico da CADEIA de navegação autónoma — diz ONDE falha.

Cadeia testada (cada elo depende do anterior):
  S0 nós vivos        — mission_manager / mission_nav2_bridge / planner / controller / bt
  S1 relógio          — /clock a avançar (sim a correr)
  S2 TF map->base     — existe e o map->odom está pós-datado (sem 'extrapolation')
  S3 obstáculos       — nuvem de obstáculos do costmap local a publicar (stem_band)
  S4 comando aceite   — mission_manager recebeu o comando (publicou mission_goal/route)
  S5 plano global     — planeador produziu /plan não-vazio
  S6 controlador      — /cmd_vel_nav com comando não-nulo
  S7 robô a mexer     — a pose mudou > limiar
  S8 chegou           — /planning/goal_reached (ou FALHA: /planning/path_blocked)

Reporta o 1.º elo partido + uma pista da causa. Robusto à corrida de discovery
(espera o emparelhamento do publisher de /mission/command antes de publicar).

Uso: python3 drive_chain_check.py --x 5 --y 0 [--patrol] [--timeout 40]
"""
from __future__ import annotations
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from forest_hybrid_msgs.msg import MissionCommand

REQUIRED_NODES = [
    "mission_manager_node", "mission_nav2_bridge", "planner_server",
    "controller_server", "bt_navigator",
]
OBSTACLE_TOPIC = "/perception/lidar3d/experimental/non_ground"


def best_effort(depth=5):
    q = QoSProfile(depth=depth)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


def transient(depth=1):
    q = QoSProfile(depth=depth)
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    q.reliability = ReliabilityPolicy.RELIABLE
    return q


class DriveChainCheck(Node):
    def __init__(self):
        super().__init__("drive_chain_check")
        self.clock_t = None
        self.mo_stamp = None          # stamp do map->odom
        self.has_map_base = False
        self.obstacle_w = None
        self.mission_goal_seen = False
        self.route_seen = False
        self.plan_poses = 0
        self.cmd_nonzero = False
        self.reached = False
        self.blocked = False
        self.start_xy = None
        self.cur_xy = None
        self.max_move = 0.0

        self.create_subscription(Clock, "/clock", self._on_clock, best_effort())
        self.create_subscription(TFMessage, "/tf", self._tf, best_effort(50))
        self.create_subscription(PointCloud2, OBSTACLE_TOPIC, self._obs, best_effort())
        self.create_subscription(PoseStamped, "/state/pose_fused", self._pose, best_effort(20))
        self.create_subscription(PoseStamped, "/planning/mission_goal", self._mgoal, transient())
        self.create_subscription(Path, "/planning/mission_route", self._mroute, transient())
        self.create_subscription(Path, "/plan", self._plan, best_effort())
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd, 10)
        self.create_subscription(Bool, "/planning/goal_reached", self._reached, 10)
        self.create_subscription(Bool, "/planning/path_blocked", self._blocked, 10)
        self.cmd_pub = self.create_publisher(MissionCommand, "/mission/command", 10)

    def _on_clock(self, m):
        self.clock_t = m.clock.sec + m.clock.nanosec * 1e-9

    def _tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                self.mo_stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            if t.child_frame_id.endswith("base_link"):
                self.has_map_base = True

    def _obs(self, m):
        self.obstacle_w = m.width

    def _pose(self, m):
        self.cur_xy = (m.pose.position.x, m.pose.position.y)
        if self.start_xy is None:
            self.start_xy = self.cur_xy
        self.max_move = max(self.max_move, math.hypot(
            self.cur_xy[0] - self.start_xy[0], self.cur_xy[1] - self.start_xy[1]))

    def _mgoal(self, m):
        self.mission_goal_seen = True

    def _mroute(self, m):
        if len(m.poses) > 0:
            self.route_seen = True

    def _plan(self, m):
        self.plan_poses = max(self.plan_poses, len(m.poses))

    def _cmd(self, m):
        if abs(m.linear.x) > 0.02 or abs(m.angular.z) > 0.05:
            self.cmd_nonzero = True

    def _reached(self, m):
        if m.data:
            self.reached = True

    def _blocked(self, m):
        if m.data:
            self.blocked = True

    def spin(self, dur):
        t0 = time.time()
        while time.time() - t0 < dur and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_goto(self, x, y):
        c = MissionCommand()
        c.command_type = MissionCommand.CMD_GOTO_XYZ
        c.frame_type = MissionCommand.FRAME_MAP
        c.command_id = "drive_chain_check"
        c.source = "diag"
        c.target_x, c.target_y, c.target_z = float(x), float(y), 0.0
        self.cmd_pub.publish(c)

    def send_patrol(self, wps):
        c = MissionCommand()
        c.command_type = MissionCommand.CMD_PATROL_WAYPOINTS
        c.frame_type = MissionCommand.FRAME_MAP
        c.command_id = "drive_chain_check"
        c.source = "diag"
        c.waypoint_x = [float(a) for a, _ in wps]
        c.waypoint_y = [float(b) for _, b in wps]
        c.waypoint_z = [0.0 for _ in wps]
        self.cmd_pub.publish(c)


def line(ok, name, detail=""):
    mark = "\033[92mOK\033[0m" if ok else "\033[91mFALHA\033[0m"
    return f"  [{mark}] {name}" + (f"  — {detail}" if detail else "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=5.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--patrol", action="store_true", help="testar PATROL em vez de GOTO")
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--move-thresh", type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    n = DriveChainCheck()
    results = []   # (ok, name, detail)
    broke_at = None

    def record(ok, name, detail=""):
        nonlocal broke_at
        results.append((ok, name, detail))
        if not ok and broke_at is None:
            broke_at = name
        return ok

    # ---- S0 nós vivos (poll: discovery pode demorar) ----
    t0 = time.time()
    missing = list(REQUIRED_NODES)
    while missing and time.time() - t0 < 8:
        n.spin(0.5)
        names = n.get_node_names()
        missing = [x for x in REQUIRED_NODES if x not in names]
    record(not missing, "S0 nós vivos",
           "todos presentes" if not missing else f"em falta: {missing}")

    # ---- S1 relógio (poll até avançar; best-effort pode falhar amostras) ----
    t0 = time.time()
    while n.clock_t is None and time.time() - t0 < 3:
        n.spin(0.3)
    c0 = n.clock_t
    advancing = False
    t0 = time.time()
    while c0 is not None and not advancing and time.time() - t0 < 4:
        n.spin(0.3)
        advancing = n.clock_t is not None and n.clock_t > c0
    record(advancing, "S1 relógio /clock",
           f"t={n.clock_t:.1f}s a avançar" if advancing else "parado/ausente (sim a correr?)")

    # ---- S2 TF (poll: tree_slam pode demorar a 1ª publicação) ----
    t0 = time.time()
    while n.mo_stamp is None and time.time() - t0 < 6:
        n.spin(0.3)
    if n.mo_stamp is None or n.clock_t is None:
        record(False, "S2 TF map->odom", "sem map->odom (tree_slam publica?)")
    else:
        lead = n.mo_stamp - n.clock_t
        record(n.has_map_base and lead > -0.05, "S2 TF map->base",
               f"map->odom pós-datado {lead:+.2f}s (precisa >0 p/ RPP não extrapolar)")

    # ---- S3 obstáculos ----
    n.spin(1.5)
    record(n.obstacle_w is not None, "S3 nuvem obstáculos",
           f"{OBSTACLE_TOPIC} width={n.obstacle_w}" if n.obstacle_w is not None
           else f"{OBSTACLE_TOPIC} SEM dados (perceção a publicar?)")

    if n.start_xy is None:
        n.spin(2.0)
    if n.start_xy is None:
        record(False, "pose inicial", "sem /state/pose_fused")
        _print(results, broke_at); rclpy.shutdown(); return

    # ---- enviar comando (robusto à corrida de discovery) ----
    t0 = time.time()
    while n.cmd_pub.get_subscription_count() < 1 and time.time() - t0 < 8:
        n.spin(0.3)
    sub_ok = n.cmd_pub.get_subscription_count() >= 1
    if not sub_ok:
        record(False, "S4 comando aceite",
               "ninguém subscreve /mission/command (mission_manager vivo?)")
        _print(results, broke_at); rclpy.shutdown(); return

    # Enviar UMA vez (o subscritor já está emparelhado). Enviar repetido criava
    # cancelamentos/abortos transitórios → path_blocked falso.
    if args.patrol:
        wps = [(args.x, 0.0), (args.x, args.y), (0.0, args.y)]
        n.send_patrol(wps)
    else:
        n.send_goto(args.x, args.y)
    n.spin(0.5)

    # ---- monitorizar a cadeia até CHEGAR ou timeout (blocked é só transitório) ----
    t0 = time.time()
    while time.time() - t0 < args.timeout and not n.reached:
        n.spin(0.2)

    cmd_target = n.route_seen if args.patrol else n.mission_goal_seen
    record(cmd_target, "S4 comando aceite",
           "mission_manager publicou " + ("rota" if args.patrol else "mission_goal")
           if cmd_target else "mission_manager NÃO publicou (comando perdido/rejeitado)")
    record(n.plan_poses > 1, "S5 plano global /plan",
           f"{n.plan_poses} poses" if n.plan_poses > 1 else "vazio (planeador falhou)")
    # cmd_nonzero é avaliado por amostragem do /cmd_vel_nav durante o monitor:
    record(n.cmd_nonzero, "S6 controlador /cmd_vel_nav",
           "comandou velocidade" if n.cmd_nonzero
           else "linear/angular ~0 (RPP não conduz — TF? costmap? rotate_to_heading?)")
    record(n.max_move >= args.move_thresh, "S7 robô a mexer",
           f"deslocou {n.max_move:.2f} m" if n.max_move >= args.move_thresh
           else f"só {n.max_move:.2f} m (controlador/colisão a travar?)")
    if n.reached:
        record(True, "S8 resultado",
               "GOAL_REACHED" + (" (após recovery)" if n.blocked else ""))
    elif n.blocked:
        record(False, "S8 resultado", "PATH_BLOCKED (não chegou — obstáculo/inflação)")
    else:
        record(False, "S8 resultado", "TIMEOUT (não chegou)")

    _print(results, broke_at)
    rclpy.shutdown()


def _print(results, broke_at):
    print("\n========== DIAGNÓSTICO DA CADEIA DE CONDUÇÃO ==========")
    for ok, name, detail in results:
        print(line(ok, name, detail))
    first_fail = next((nm for ok, nm, _ in results if not ok), None)
    print("------------------------------------------------------")
    if first_fail is None:
        print("  \033[92mTUDO OK — o robô navega end-to-end.\033[0m")
    else:
        print(f"  \033[91m>>> FALHA NO ELO: {first_fail}\033[0m (corrige aqui primeiro)")
    print("======================================================\n")


if __name__ == "__main__":
    main()

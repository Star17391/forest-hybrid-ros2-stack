#!/usr/bin/env python3
"""Conduz o robô num LAÇO FECHADO (polígono) pela floresta e volta ao início.

Objetivo: validar o Tree-SLAM end-to-end num mundo multi-árvore —
  - tracker:      nascem vários landmarks, n_observations cresce, posições guardadas;
  - backend:      o grafo otimiza (pose_covariance_trace) à medida que anda;
  - relocalizador: ao FECHAR o laço (revisitar as árvores do início) NÃO devem
                   nascer uids duplicados — os uids antigos são re-associados.

Skid-steer (linear.x + angular.z), pose do EKF (/state/pose_fused, frame map, com IMU).
Pura-perseguição "virar-e-andar": em cada waypoint roda para o próximo, depois anda.

Uso: slam_loop_drive.py [--side 8] [--v 0.5] [--laps 1]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LoopDriver(Node):
    def __init__(self, waypoints, v, w_max=0.6):
        super().__init__("slam_loop_drive")
        self.pub = self.create_publisher(Twist, "/forest_gen/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/state/pose_fused", self.cb, 20)
        self.x = self.y = self.yaw = None
        self.wps = waypoints
        self.v = v
        self.w_max = w_max

    def cb(self, m):
        self.x = m.pose.position.x
        self.y = m.pose.position.y
        self.yaw = yaw_of(m.pose.orientation)

    def cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def spin_until_pose(self, timeout=30.0):
        t0 = time.time()
        while self.x is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.x is not None

    def goto(self, gx, gy, pos_tol=0.5, face_tol=0.15):
        # 1) virar para o waypoint (parado)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.x is None:
                continue
            bearing = math.atan2(gy - self.y, gx - self.x)
            err = wrap(bearing - self.yaw)
            if abs(err) < face_tol:
                break
            self.cmd(0.0, max(-self.w_max, min(self.w_max, 1.5 * err)))
        self.cmd(0.0, 0.0)
        # 2) andar até ao waypoint, corrigindo o rumo levemente
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.x is None:
                continue
            d = math.hypot(gx - self.x, gy - self.y)
            if d < pos_tol:
                break
            bearing = math.atan2(gy - self.y, gx - self.x)
            err = wrap(bearing - self.yaw)
            self.cmd(self.v, max(-self.w_max, min(self.w_max, 1.2 * err)))
        self.cmd(0.0, 0.0)

    def run(self):
        if not self.spin_until_pose():
            self.get_logger().error("sem pose /state/pose_fused")
            return
        self.get_logger().info("pose ok; a iniciar laço")
        for i, (gx, gy) in enumerate(self.wps):
            self.get_logger().info(f"waypoint {i+1}/{len(self.wps)} -> ({gx:.1f},{gy:.1f})")
            self.goto(gx, gy)
        self.cmd(0.0, 0.0)
        self.get_logger().info("laço completo (de volta ao início)")


def main():
    side = 8.0
    v = 0.5
    laps = 1
    a = sys.argv[1:]
    for i, tok in enumerate(a):
        if tok == "--side":
            side = float(a[i + 1])
        elif tok == "--v":
            v = float(a[i + 1])
        elif tok == "--laps":
            laps = int(a[i + 1])
    # Laço quadrado a partir da origem, voltando à origem (revisita = loop closure).
    square = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    wps = square * laps
    rclpy.init()
    node = LoopDriver(wps, v)
    try:
        node.run()
    finally:
        node.cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Órbita PARAR-E-OLHAR à volta de uma árvore, para construir a referência multi-vista.

PORQUÊ (medido em dados reais): o LiDAR está inclinado e o tronco SÓ é detetado quando
está À FRENTE do robô (0–60° → 100% deteção); ao LADO (90°) ou atrás a deteção é ~0%.
Logo uma órbita tangencial contínua (árvore sempre ao lado) NÃO deteta nada. A solução
é discreta: ir a um ponto da órbita, VIRAR-SE PARA A ÁRVORE, ficar uns segundos parado
a acumular (vista nova = arco novo do tronco), e avançar para o ponto seguinte.

Skid-steer (sem andar de lado) + pose do EKF (/state/pose_fused, frame map, com IMU).
Máquina de estados: GOTO (ir ao waypoint) → FACE (virar para a árvore) → DWELL (acumular).

Uso:  orbit_tree.py --tree-x 4 --radius 4 --sweep-deg 180 [--step-deg 30] [--dwell 4]
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Orbit(Node):
    def __init__(self, a):
        super().__init__("orbit_tree")
        self.a = a
        self.tx, self.ty, self.R = a.tree_x, a.tree_y, a.radius
        self.pub = self.create_publisher(Twist, "/forest_gen/cmd_vel", 10)
        # Pose do EKF (fundida com IMU), frame map — melhor e menos deriva que odom puro.
        self.sub = self.create_subscription(PoseStamped, "/state/pose_fused", self.cb, 20)
        self.pose = None
        self.waypoints = None        # ângulos de órbita (tronco->robô) a visitar
        self.wi = 0
        self.state = "GOTO"
        self.dwell_t0 = None
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)

    def cb(self, m):
        p = m.pose.position
        self.pose = (p.x, p.y, yaw_of(m.pose.orientation))

    def _drive(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.pub.publish(t)

    def tick(self):
        if self.pose is None:
            return
        rx, ry, yaw = self.pose
        ex, ey = rx - self.tx, ry - self.ty          # tronco -> robô
        ang = math.atan2(ey, ex)
        face = wrap(math.atan2(self.ty - ry, self.tx - rx) - yaw)   # erro p/ encarar a árvore

        if self.waypoints is None:                   # gera waypoints a partir da pose inicial
            n = int(self.a.sweep_deg / self.a.step_deg)
            self.waypoints = [wrap(ang + math.radians(self.a.step_deg * (k + 1))) for k in range(n)]
            self.get_logger().info(f"{n} paragens de {self.a.step_deg:.0f}° (dwell {self.a.dwell:.0f}s)")

        if self.done:
            self._drive(0, 0); return
        if self.wi >= len(self.waypoints):
            self._drive(0, 0); self.done = True
            self.get_logger().info("órbita parar-e-olhar completa")
            rclpy.shutdown(); return

        target = self.waypoints[self.wi]
        if self.state == "GOTO":
            # ir ao ponto do círculo no ângulo `target` (go-to-goal)
            gx = self.tx + self.R * math.cos(target)
            gy = self.ty + self.R * math.sin(target)
            herr = wrap(math.atan2(gy - ry, gx - rx) - yaw)
            if math.hypot(gx - rx, gy - ry) < 0.35:
                self.state = "FACE"
            else:
                w = max(-0.8, min(0.8, 1.6 * herr))
                v = self.a.v * max(0.0, math.cos(herr)) if abs(herr) < 1.0 else 0.0
                self._drive(v, w)
        elif self.state == "FACE":
            # virar-se para a árvore (trazê-la a 0° → setor de deteção)
            if abs(face) < math.radians(5):
                self.state = "DWELL"; self.dwell_t0 = self.get_clock().now()
                self._drive(0, 0)
            else:
                self._drive(0, max(-0.7, min(0.7, 1.5 * face)))
        elif self.state == "DWELL":
            self._drive(0, 0)                          # parado, a olhar -> perceção acumula
            if (self.get_clock().now() - self.dwell_t0).nanoseconds * 1e-9 >= self.a.dwell:
                self.wi += 1; self.state = "GOTO"

    def stop(self):
        for _ in range(5):
            self._drive(0, 0)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tree-x", type=float, default=4.0)
    ap.add_argument("--tree-y", type=float, default=0.0)
    ap.add_argument("--radius", type=float, default=4.0)
    ap.add_argument("--sweep-deg", type=float, default=180.0)
    ap.add_argument("--step-deg", type=float, default=30.0)   # ângulo entre paragens
    ap.add_argument("--dwell", type=float, default=4.0)       # segundos parado a olhar
    ap.add_argument("--v", type=float, default=0.45)
    a = ap.parse_args()
    rclpy.init()
    node = Orbit(a)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()

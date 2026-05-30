#!/usr/bin/env python3
"""Compara pose estimada (EKF/state_contract) vs ground truth Gazebo (world_tf).

Requer: use_pose_bridge:=true OU tópico /forest_gen/gz/world_tf a publicar.
Com sim_mvp (EKF only): compara /state/pose_fused com integração — use após activar world_tf.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseCompare(Node):
    def __init__(self, model: str, duration: float) -> None:
        super().__init__("pose_compare")
        self._model = model
        self._duration = duration
        self._t0 = time.monotonic()
        self._fused: PoseStamped | None = None
        self._gz: tuple[float, float, float] | None = None
        self._errors: list[tuple[float, float, float]] = []

        self.create_subscription(PoseStamped, "/state/pose_fused", self._on_fused, 10)
        self.create_subscription(TFMessage, "/forest_gen/gz/world_tf_full", self._on_gz, 10)

    def _on_fused(self, msg: PoseStamped) -> None:
        self._fused = msg
        self._tick()

    def _on_gz(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            child = tr.child_frame_id or ""
            if self._model in child or child.endswith("/base_link"):
                p = tr.transform.translation
                q = tr.transform.rotation
                self._gz = (p.x, p.y, yaw(q))
                break
        self._tick()

    def _tick(self) -> None:
        if self._fused is None or self._gz is None:
            return
        fx = self._fused.pose.position.x
        fy = self._fused.pose.position.y
        fq = self._fused.pose.orientation
        fyaw = yaw(fq)
        gx, gy, gyaw = self._gz
        dpos = math.hypot(fx - gx, fy - gy)
        dyaw = abs(fyaw - gyaw)
        if dyaw > math.pi:
            dyaw = 2 * math.pi - dyaw
        self._errors.append((dpos, dyaw, time.monotonic() - self._t0))
        if time.monotonic() - self._t0 >= self._duration:
            self._report()
            raise SystemExit(0)

    def _report(self) -> None:
        if not self._errors:
            print("Sem amostras alinhadas. world_tf pode estar lazy — use use_pose_bridge:=true")
            return
        dpos = sorted(e[0] for e in self._errors)
        dyaw = sorted(math.degrees(e[1]) for e in self._errors)
        print("\n=== pose_fused vs Gazebo (world_tf_full) ===")
        print(f"Samples: {len(self._errors)}")
        print(f"|Δpos| max={dpos[-1]:.3f} m  p95={dpos[int(0.95*(len(dpos)-1))]:.3f} m  mean={sum(dpos)/len(dpos):.3f} m")
        print(f"|Δyaw| max={dyaw[-1]:.1f}°  p95={dyaw[int(0.95*(len(dyaw)-1))]:.1f}°")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--model", default="marble_hd2")
    args = p.parse_args()
    rclpy.init()
    node = PoseCompare(args.model, args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

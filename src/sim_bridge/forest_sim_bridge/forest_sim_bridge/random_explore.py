#!/usr/bin/env python3
"""Exploração aleatória com cmd_vel contínuo (segmentos longos + rampa suave).

Publica geometry_msgs/Twist em /forest_gen/cmd_vel. Cada segmento mantém um
comportamento (rotação, avanço, recuo) durante vários segundos; velocidades
aproximam-se do alvo com rampa — não há mudança de intenção a cada tick.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _ramp(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + max_delta if delta > 0.0 else current - max_delta


@dataclass(frozen=True)
class Segment:
    """Alvo de velocidade para um segmento de movimento."""

    linear_x: float
    angular_z: float
    duration_s: float


class RandomExploreNode(Node):
    def __init__(self) -> None:
        super().__init__("forest_random_explore")
        self.declare_parameter("cmd_vel_topic", "/forest_gen/cmd_vel")
        self.declare_parameter("linear_speed", 0.45)
        self.declare_parameter("angular_speed", 0.55)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("segment_min_s", 3.5)
        self.declare_parameter("segment_max_s", 9.0)
        self.declare_parameter("ramp_linear_per_s", 0.35)
        self.declare_parameter("ramp_angular_per_s", 0.9)
        # Pesos relativos: rotação pura, avanço, recuo, avanço+rotação, paragem
        self.declare_parameter("weight_rotate", 0.42)
        self.declare_parameter("weight_forward", 0.33)
        self.declare_parameter("weight_backward", 0.12)
        self.declare_parameter("weight_forward_turn", 0.10)
        self.declare_parameter("weight_stop", 0.03)

        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._lin_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self._ang_speed = self.get_parameter("angular_speed").get_parameter_value().double_value
        hz = max(5.0, self.get_parameter("rate_hz").get_parameter_value().double_value)
        self._dt = 1.0 / hz
        self._seg_min = self.get_parameter("segment_min_s").get_parameter_value().double_value
        self._seg_max = max(
            self._seg_min,
            self.get_parameter("segment_max_s").get_parameter_value().double_value,
        )
        self._ramp_lin = self.get_parameter("ramp_linear_per_s").get_parameter_value().double_value
        self._ramp_ang = self.get_parameter("ramp_angular_per_s").get_parameter_value().double_value
        self._weights = (
            self.get_parameter("weight_rotate").get_parameter_value().double_value,
            self.get_parameter("weight_forward").get_parameter_value().double_value,
            self.get_parameter("weight_backward").get_parameter_value().double_value,
            self.get_parameter("weight_forward_turn").get_parameter_value().double_value,
            self.get_parameter("weight_stop").get_parameter_value().double_value,
        )

        self._pub = self.create_publisher(Twist, topic, 10)
        self._cur_vx = 0.0
        self._cur_wz = 0.0
        self._segment: Segment | None = None
        self._segment_elapsed = 0.0

        self._timer = self.create_timer(self._dt, self._on_timer)
        self._pick_segment()
        self.get_logger().info(
            f"Random explore → {topic} "
            f"(segments {self._seg_min:.1f}–{self._seg_max:.1f}s, {hz:.0f} Hz)"
        )

    def _segment_duration(self) -> float:
        return random.uniform(self._seg_min, self._seg_max)

    def _pick_segment(self) -> None:
        w_rot, w_fwd, w_back, w_ft, w_stop = self._weights
        total = w_rot + w_fwd + w_back + w_ft + w_stop
        if total <= 0.0:
            w_rot, w_fwd, w_back, w_ft, w_stop = 0.42, 0.33, 0.12, 0.10, 0.03
            total = 1.0
        r = random.random() * total
        sign = 1.0 if random.random() < 0.5 else -1.0
        dur = self._segment_duration()

        if r < w_rot:
            self._segment = Segment(0.0, sign * self._ang_speed, dur)
            label = "rotate"
        elif r < w_rot + w_fwd:
            self._segment = Segment(self._lin_speed, 0.0, dur)
            label = "forward"
        elif r < w_rot + w_fwd + w_back:
            self._segment = Segment(-self._lin_speed * 0.65, 0.0, dur)
            label = "backward"
        elif r < w_rot + w_fwd + w_back + w_ft:
            turn = sign * self._ang_speed * 0.45
            self._segment = Segment(self._lin_speed * 0.7, turn, dur)
            label = "forward+turn"
        else:
            self._segment = Segment(0.0, 0.0, dur * 0.35)
            label = "pause"

        self._segment_elapsed = 0.0
        assert self._segment is not None
        self.get_logger().info(
            f"Segment {label}: vx={self._segment.linear_x:.2f} "
            f"wz={self._segment.angular_z:.2f} for {self._segment.duration_s:.1f}s"
        )

    def _on_timer(self) -> None:
        if self._segment is None:
            self._pick_segment()
            return

        self._segment_elapsed += self._dt
        if self._segment_elapsed >= self._segment.duration_s:
            self._pick_segment()
            assert self._segment is not None

        max_dv = self._ramp_lin * self._dt
        max_dw = self._ramp_ang * self._dt
        self._cur_vx = _ramp(self._cur_vx, self._segment.linear_x, max_dv)
        self._cur_wz = _ramp(self._cur_wz, self._segment.angular_z, max_dw)

        msg = Twist()
        msg.linear.x = self._cur_vx
        msg.angular.z = self._cur_wz
        self._pub.publish(msg)

    def stop(self) -> None:
        self._segment = Segment(0.0, 0.0, 0.5)
        self._cur_vx = 0.0
        self._cur_wz = 0.0
        msg = Twist()
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RandomExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

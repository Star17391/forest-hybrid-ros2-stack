#!/usr/bin/env python3
"""Aerial motor mixing after track→drone transform (replaces stock Lee timing).

Recalculates the Lee allocation matrix when lagartas reach ±90°, then publishes
``Actuators`` on ``/{ns}/gazebo/command/motor_speed`` via ``gz.transport`` (not ``gz topic`` CLI).
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
import rclpy
from forest_hybrid_msgs.msg import HybridTransitionStatus
from geometry_msgs.msg import Twist
from gz.msgs10.actuators_pb2 import Actuators
from gz.transport13 import Node as GzTransportNode
from rclpy.node import Node
from sensor_msgs.msg import JointState

G = 9.81
ROTOR_DIRS = (1, -1, -1, 1)
PROP_X = 0.35
SHOULDER_Y = 0.35
YAW_REBUILD_EPS = 0.02
YAW_THRUST_READY_EPS = 0.12


def _rot_x(angle: float, v: tuple[float, float, float]) -> tuple[float, float, float]:
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = v
    return (x, c * y - s * z, s * y + c * z)


def _prop_xy_in_base(
    left_roll: float, right_roll: float
) -> list[tuple[float, float]]:
    left_shoulder = (0.0, SHOULDER_Y, 0.0)
    right_shoulder = (0.0, -SHOULDER_Y, 0.0)
    out: list[tuple[float, float]] = []
    for roll, shoulder, x_prop in (
        (left_roll, left_shoulder, PROP_X),
        (right_roll, right_shoulder, PROP_X),
        (left_roll, left_shoulder, -PROP_X),
        (right_roll, right_shoulder, -PROP_X),
    ):
        p = _rot_x(roll, (x_prop, 0.0, 0.0))
        out.append((shoulder[0] + p[0], shoulder[1] + p[1]))
    return out


def build_allocation_matrix(
    positions_xy: Sequence[tuple[float, float]],
    directions: Sequence[int],
    force_k: float,
    moment_k: float,
) -> np.ndarray:
    a = np.zeros((4, 4))
    for i, ((x, y), d) in enumerate(zip(positions_xy, directions)):
        arm = math.hypot(x, y)
        ang = math.atan2(y, x)
        a[0, i] = math.sin(ang) * arm * force_k
        a[1, i] = -math.cos(ang) * arm * force_k
        a[2, i] = -d * force_k * moment_k
        a[3, i] = force_k
    return a


class HybridAerialMotorController(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_aerial_motor_controller")

        self.declare_parameter("robot_namespace", "marble_hd2")
        self.declare_parameter("mass_kg", 5.84)
        self.declare_parameter("motor_constant", 1.25e-4)
        self.declare_parameter("moment_constant", 0.010)
        self.declare_parameter("spin_up_sec", 2.5)
        self.declare_parameter("takeoff_equal_thrust", True)
        self.declare_parameter("max_omega", 800.0)
        self.declare_parameter("velocity_gain", 1.0)
        self.declare_parameter("left_track_yaw_aerial_rad", math.pi / 2.0)
        self.declare_parameter("right_track_yaw_aerial_rad", -math.pi / 2.0)
        self.declare_parameter("use_joint_states_for_geometry", True)
        self.declare_parameter("control_rate_hz", 50.0)

        self._ns = self.get_parameter("robot_namespace").value
        self._mass = self.get_parameter("mass_kg").value
        self._k = self.get_parameter("motor_constant").value
        self._m = self.get_parameter("moment_constant").value
        self._max_omega = self.get_parameter("max_omega").value
        self._vg = self.get_parameter("velocity_gain").value
        self._left_aerial = self.get_parameter("left_track_yaw_aerial_rad").value
        self._right_aerial = self.get_parameter("right_track_yaw_aerial_rad").value
        self._use_js = self.get_parameter("use_joint_states_for_geometry").value

        self._left_roll = 0.0
        self._right_roll = 0.0
        self._aerial_active = False
        self._cmd = Twist()
        self._alloc: np.ndarray | None = None
        self._alloc_signature: tuple[float, float] | None = None
        self._hover_omega = math.sqrt((self._mass * G) / (4.0 * self._k))
        self._publish_fail_count = 0
        self._thrust_gated_logged = False
        self._spin_up_sec = max(0.0, float(self.get_parameter("spin_up_sec").value))
        self._equal_thrust = bool(self.get_parameter("takeoff_equal_thrust").value)
        self._aerial_since: float | None = None

        self._gz = GzTransportNode()
        self._motor_topic = f"/{self._ns}/gazebo/command/motor_speed"
        self._gz_pub = self._gz.advertise(self._motor_topic, Actuators)

        rate = max(10.0, float(self.get_parameter("control_rate_hz").value))
        self.create_subscription(
            HybridTransitionStatus,
            "/forest_gen/hybrid/transition_status",
            self._on_status,
            10,
        )
        self.create_subscription(
            Twist, "/forest_gen/hybrid/aerial_cmd_vel", self._on_cmd, 10
        )
        self.create_subscription(JointState, "/forest_gen/hybrid/joint_states", self._on_js, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"Motor cmd via gz.transport on {self._motor_topic} "
            f"(hover ω≈{self._hover_omega:.0f} rad/s)"
        )

    def _current_rolls(self) -> tuple[float, float]:
        if self._use_js:
            return self._left_roll, self._right_roll
        return self._left_aerial, self._right_aerial

    def _tracks_ready_for_thrust(self) -> bool:
        """Empuxo vertical só com lagartas na pose drone (±90°); senão eixo Y → órbita no ar."""
        l_roll, r_roll = self._current_rolls()
        return (
            abs(l_roll - self._left_aerial) <= YAW_THRUST_READY_EPS
            and abs(r_roll - self._right_aerial) <= YAW_THRUST_READY_EPS
        )

    def _maybe_rebuild_allocation(self, force: bool = False) -> None:
        l_roll, r_roll = self._current_rolls()
        sig = (round(l_roll, 3), round(r_roll, 3))
        if not force and self._alloc is not None and self._alloc_signature == sig:
            return

        xy = _prop_xy_in_base(l_roll, r_roll)
        a = build_allocation_matrix(xy, ROTOR_DIRS, self._k, self._m)
        rank = int(np.linalg.matrix_rank(a))
        if rank < 4:
            self.get_logger().error(
                f"Allocation rank {rank} < 4 "
                f"(L={math.degrees(l_roll):.0f}° R={math.degrees(r_roll):.0f}°)"
            )
            self._alloc = None
            self._alloc_signature = None
            return

        self._alloc = a
        self._alloc_signature = sig
        self.get_logger().info(
            f"Allocation OK (rank 4) L={math.degrees(l_roll):.1f}° "
            f"R={math.degrees(r_roll):.1f}°"
        )

    def _on_status(self, msg: HybridTransitionStatus) -> None:
        was_active = self._aerial_active
        self._aerial_active = msg.state_name in (
            "AERIAL_READY",
            "AERIAL_FLY",
            "AERIAL_HOVER",
        )
        if self._aerial_active and not was_active:
            self._aerial_since = self.get_clock().now().nanoseconds / 1e9
            self._maybe_rebuild_allocation(force=True)
        if was_active and not self._aerial_active:
            self._aerial_since = None
            self._alloc = None
            self._alloc_signature = None
            self._publish_gz_actuators([0.0, 0.0, 0.0, 0.0])

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = msg

    def _on_js(self, msg: JointState) -> None:
        if not self._use_js:
            return
        changed = False
        for name, pos in zip(msg.name, msg.position):
            if "left_track_yaw_joint" in name:
                v = float(pos)
                if abs(v - self._left_roll) > YAW_REBUILD_EPS:
                    self._left_roll = v
                    changed = True
            elif "right_track_yaw_joint" in name:
                v = float(pos)
                if abs(v - self._right_roll) > YAW_REBUILD_EPS:
                    self._right_roll = v
                    changed = True
        if changed and self._aerial_active:
            self._maybe_rebuild_allocation()

    def _spin_scale(self) -> float:
        if self._aerial_since is None or self._spin_up_sec <= 0.0:
            return 1.0
        t = self.get_clock().now().nanoseconds / 1e9 - self._aerial_since
        return max(0.0, min(1.0, t / self._spin_up_sec))

    def _motor_omegas(self) -> list[float]:
        if self._alloc is None:
            return [0.0, 0.0, 0.0, 0.0]
        scale = self._spin_scale()
        if self._equal_thrust and scale < 1.0:
            w = self._hover_omega * scale
            return [w, w, w, w]
        thrust = self._mass * G + self._mass * self._vg * float(self._cmd.linear.z)
        fx = self._mass * self._vg * float(self._cmd.linear.x)
        fy = self._mass * self._vg * float(self._cmd.linear.y)
        mz = self._mass * 0.15 * float(self._cmd.angular.z)
        wrench = np.array([fx, fy, mz, thrust])
        try:
            omega_sq = np.linalg.lstsq(self._alloc, wrench, rcond=None)[0]
        except np.linalg.LinAlgError:
            omega_sq = np.full(4, (self._mass * G) / (4.0 * self._k))
        omegas: list[float] = []
        for w2 in omega_sq:
            w = math.sqrt(max(0.0, float(w2)))
            omegas.append(max(0.0, min(self._max_omega, w * scale)))
        return omegas

    def _publish_gz_actuators(self, omegas: Sequence[float]) -> None:
        msg = Actuators()
        msg.velocity.extend(float(w) for w in omegas)
        ok = self._gz_pub.publish(msg)
        if not ok:
            self._publish_fail_count += 1
            if self._publish_fail_count % 50 == 1:
                self.get_logger().warn(
                    f"gz.transport publish failed on {self._motor_topic}",
                    throttle_duration_sec=10.0,
                )
        else:
            self._publish_fail_count = 0

    def _tick(self) -> None:
        if not self._aerial_active:
            return
        if not self._tracks_ready_for_thrust():
            if not self._thrust_gated_logged:
                l_roll, r_roll = self._current_rolls()
                self.get_logger().warn(
                    f"Motors idle until track yaw at drone pose "
                    f"(L={math.degrees(l_roll):.0f}°/{math.degrees(self._left_aerial):.0f}° "
                    f"R={math.degrees(r_roll):.0f}°/{math.degrees(self._right_aerial):.0f}°)"
                )
                self._thrust_gated_logged = True
            self._publish_gz_actuators([0.0, 0.0, 0.0, 0.0])
            return
        self._thrust_gated_logged = False
        if self._alloc is None:
            self._maybe_rebuild_allocation(force=True)
        self._publish_gz_actuators(self._motor_omegas())


def main() -> None:
    rclpy.init()
    node = HybridAerialMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

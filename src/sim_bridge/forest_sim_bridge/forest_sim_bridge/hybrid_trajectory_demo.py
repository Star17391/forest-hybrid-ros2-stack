#!/usr/bin/env python3
"""Demo mission: ground → transition point → aerial leg → land → ground to final point.

Requires ``sim-hybrid-test`` stack (hybrid_transition_manager + marble_pose_from_gz).

Waypoints (map frame, metres):
  1. transition_xy — stop here, publish ``to_aerial``
  2. aerial_xyz — cruise in multicopter mode (AERIAL_HOVER + aerial_nav_cmd)
  3. final_xy — after ``to_ground``, drive here on tracks

Trigger manually or via ``forest test hybrid-mission``.
"""

from __future__ import annotations

import math
import time
from enum import IntEnum

import rclpy
from forest_hybrid_msgs.msg import HybridTransitionStatus
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String


class Phase(IntEnum):
    GROUND_TO_TRANSITION = 0
    WAIT_AERIAL = 1
    AERIAL_CRUISE = 2
    WAIT_GROUND = 3
    GROUND_TO_FINAL = 4
    DONE = 5
    FAILED = 6


PHASE_NAMES = {p: p.name for p in Phase}


class HybridTrajectoryDemo(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_trajectory_demo")

        self.declare_parameter("transition_x", 3.5)
        self.declare_parameter("transition_y", 0.0)
        self.declare_parameter("aerial_x", 3.5)
        self.declare_parameter("aerial_y", 3.5)
        self.declare_parameter("aerial_z", 1.0)
        self.declare_parameter("final_x", 7.0)
        self.declare_parameter("final_y", 3.5)
        self.declare_parameter("ground_xy_tol_m", 0.45)
        self.declare_parameter("aerial_xy_tol_m", 0.55)
        self.declare_parameter("aerial_z_tol_m", 0.2)
        self.declare_parameter("ground_linear_speed", 0.45)
        self.declare_parameter("ground_angular_gain", 1.2)
        self.declare_parameter("aerial_linear_speed", 0.22)
        self.declare_parameter("aerial_z_gain", 0.8)
        self.declare_parameter("phase_timeout_sec", 180.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("start_delay_sec", 2.0)

        self._tx = self.get_parameter("transition_x").value
        self._ty = self.get_parameter("transition_y").value
        self._ax = self.get_parameter("aerial_x").value
        self._ay = self.get_parameter("aerial_y").value
        self._az = self.get_parameter("aerial_z").value
        self._fx = self.get_parameter("final_x").value
        self._fy = self.get_parameter("final_y").value
        self._g_tol = self.get_parameter("ground_xy_tol_m").value
        self._a_tol = self.get_parameter("aerial_xy_tol_m").value
        self._z_tol = self.get_parameter("aerial_z_tol_m").value
        self._g_v = self.get_parameter("ground_linear_speed").value
        self._g_k = self.get_parameter("ground_angular_gain").value
        self._a_v = self.get_parameter("aerial_linear_speed").value
        self._a_z_k = self.get_parameter("aerial_z_gain").value
        self._timeout = self.get_parameter("phase_timeout_sec").value

        self._x = 0.0
        self._y = 0.0
        self._z = 0.35
        self._yaw = 0.0
        self._have_pose = False
        self._fsm_state = "GROUND_DRIVE"
        self._airborne = False
        self._phase = Phase.GROUND_TO_TRANSITION
        self._phase_entered = time.monotonic()
        self._started = False

        self._pub_cmd = self.create_publisher(Twist, "/forest_gen/cmd_vel", 10)
        self._pub_aerial_nav = self.create_publisher(
            Twist, "/forest_gen/hybrid/aerial_nav_cmd", 10
        )
        self._pub_transition = self.create_publisher(
            String, "/forest_gen/hybrid/transition_request", 10
        )
        self._pub_phase = self.create_publisher(
            String, "/forest_gen/hybrid/trajectory_phase", 10
        )

        self.create_subscription(PoseStamped, "/state/pose_fused", self._on_pose, 10)
        self.create_subscription(
            HybridTransitionStatus, "/forest_gen/hybrid/transition_status", self._on_status, 10
        )
        self.create_timer(0.05, self._tick)

        if self.get_parameter("auto_start").value:
            delay = max(0.0, self.get_parameter("start_delay_sec").value)
            self.create_timer(delay, self._mark_started, callback_group=None)

        self.get_logger().info(
            f"Hybrid trajectory demo ready. Path: ground→({self._tx:.1f},{self._ty:.1f}) "
            f"aerial→({self._ax:.1f},{self._ay:.1f},{self._az:.1f}) "
            f"ground→({self._fx:.1f},{self._fy:.1f})"
        )

    def _mark_started(self) -> None:
        self._started = True

    def _on_pose(self, msg: PoseStamped) -> None:
        self._x = float(msg.pose.position.x)
        self._y = float(msg.pose.position.y)
        self._z = float(msg.pose.position.z)
        q = msg.pose.orientation
        self._yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._have_pose = True

    def _on_status(self, msg: HybridTransitionStatus) -> None:
        self._fsm_state = msg.state_name
        self._airborne = bool(msg.airborne)

    def _enter_phase(self, phase: Phase, detail: str) -> None:
        self._phase = phase
        self._phase_entered = time.monotonic()
        name = PHASE_NAMES[phase]
        self.get_logger().info(f"phase → {name} ({detail})")
        msg = String()
        msg.data = name
        self._pub_phase.publish(msg)

    def _elapsed(self) -> float:
        return time.monotonic() - self._phase_entered

    def _publish_ground_stop(self) -> None:
        self._pub_cmd.publish(Twist())

    def _publish_aerial_stop(self) -> None:
        self._pub_aerial_nav.publish(Twist())

    def _request_transition(self, cmd: str) -> None:
        m = String()
        m.data = cmd
        self._pub_transition.publish(m)
        self.get_logger().info(f"transition_request: {cmd}")

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _ground_goto(self, tx: float, ty: float) -> None:
        dx = tx - self._x
        dy = ty - self._y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self._publish_ground_stop()
            return
        desired_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize_angle(desired_yaw - self._yaw)
        t = Twist()
        if abs(yaw_err) > 0.35:
            t.angular.z = max(-1.0, min(1.0, self._g_k * yaw_err))
        else:
            t.linear.x = min(self._g_v, 0.85 * dist)
            t.angular.z = max(-0.6, min(0.6, 0.5 * yaw_err))
        self._pub_cmd.publish(t)

    def _aerial_goto(self, tx: float, ty: float, tz: float) -> None:
        dx = tx - self._x
        dy = ty - self._y
        dz = tz - self._z
        dist_xy = math.hypot(dx, dy)
        t = Twist()
        if dist_xy > 1e-3:
            scale = min(1.0, self._a_v / dist_xy)
            t.linear.x = dx * scale
            t.linear.y = dy * scale
        if abs(dz) > self._z_tol * 0.5:
            t.linear.z = max(-0.4, min(0.4, self._a_z_k * dz))
        self._pub_aerial_nav.publish(t)

    def _at_xy(self, tx: float, ty: float, tol: float) -> bool:
        return math.hypot(tx - self._x, ty - self._y) <= tol

    def _at_aerial(self, tx: float, ty: float, tz: float) -> bool:
        return (
            self._at_xy(tx, ty, self._a_tol)
            and abs(tz - self._z) <= self._z_tol
        )

    def _tick(self) -> None:
        if not self._started or not self._have_pose:
            return

        if self._phase != Phase.DONE and self._phase != Phase.FAILED:
            if self._elapsed() > self._timeout:
                self._publish_ground_stop()
                self._publish_aerial_stop()
                self._enter_phase(Phase.FAILED, f"timeout in {PHASE_NAMES[self._phase]}")
                return

        if self._phase == Phase.GROUND_TO_TRANSITION:
            if self._at_xy(self._tx, self._ty, self._g_tol):
                self._publish_ground_stop()
                self._request_transition("to_aerial")
                self._enter_phase(Phase.WAIT_AERIAL, "at transition point")
            else:
                self._ground_goto(self._tx, self._ty)

        elif self._phase == Phase.WAIT_AERIAL:
            self._publish_ground_stop()
            if self._fsm_state in ("AERIAL_HOVER", "AERIAL_FLY") and self._airborne:
                self._enter_phase(Phase.AERIAL_CRUISE, "airborne, cruising")

        elif self._phase == Phase.AERIAL_CRUISE:
            if self._at_aerial(self._ax, self._ay, self._az):
                self._publish_aerial_stop()
                self._request_transition("to_ground")
                self._enter_phase(Phase.WAIT_GROUND, "at aerial waypoint")
            else:
                self._aerial_goto(self._ax, self._ay, self._az)

        elif self._phase == Phase.WAIT_GROUND:
            self._publish_aerial_stop()
            if self._fsm_state == "GROUND_DRIVE":
                self._enter_phase(Phase.GROUND_TO_FINAL, "landed, tracks at 0°")

        elif self._phase == Phase.GROUND_TO_FINAL:
            if self._at_xy(self._fx, self._fy, self._g_tol):
                self._publish_ground_stop()
                self._enter_phase(Phase.DONE, "mission complete")
            else:
                self._ground_goto(self._fx, self._fy)

        elif self._phase == Phase.DONE:
            self._publish_ground_stop()
            self._publish_aerial_stop()

        elif self._phase == Phase.FAILED:
            self._publish_ground_stop()
            self._publish_aerial_stop()


def main() -> None:
    rclpy.init()
    node = HybridTrajectoryDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

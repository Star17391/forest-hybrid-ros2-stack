#!/usr/bin/env python3
"""Ground → aerial reconfiguration FSM for forest_hybrid_robot in Gazebo.

Sequence (to_aerial):
  GROUND_DRIVE → TRANSITION_LOCK → LEGS_EXTENDING → TRACKS_ROTATING (±90°) → AERIAL_*

Sequence (to_ground):
  AERIAL_* → (lagartas já a 0°) → LEGS_RETRACTING → GROUND_DRIVE

Trigger: ``/forest_gen/hybrid/transition_request`` (std_msgs/String):
  ``to_aerial`` | ``to_ground`` | ``abort``
"""

from __future__ import annotations

import math
import time
from enum import IntEnum

import rclpy
from forest_hybrid_msgs.msg import HybridTransitionStatus, OperationMode
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage


class State(IntEnum):
    GROUND_DRIVE = 0
    TRANSITION_LOCK = 1
    TRACKS_ROTATING = 2
    AERIAL_READY = 3
    AERIAL_FLY = 4
    AERIAL_HOVER = 5
    LEGS_EXTENDING = 6
    LEGS_RETRACTING = 7
    FAILED = 255


STATE_NAMES = {
    State.GROUND_DRIVE: "GROUND_DRIVE",
    State.TRANSITION_LOCK: "TRANSITION_LOCK",
    State.TRACKS_ROTATING: "TRACKS_ROTATING",
    State.AERIAL_READY: "AERIAL_READY",
    State.AERIAL_FLY: "AERIAL_FLY",
    State.AERIAL_HOVER: "AERIAL_HOVER",
    State.LEGS_EXTENDING: "LEGS_EXTENDING",
    State.LEGS_RETRACTING: "LEGS_RETRACTING",
    State.FAILED: "FAILED",
}


class HybridTransitionManager(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_transition_manager")
        # Transformação drone: lagartas ±90° (eixo X); hélices nas lagartas.
        self.declare_parameter("left_track_yaw_aerial_rad", math.pi / 2.0)
        self.declare_parameter("right_track_yaw_aerial_rad", -math.pi / 2.0)
        self.declare_parameter("rotate_tracks_for_aerial", True)
        self.declare_parameter("track_yaw_tolerance_rad", 0.06)
        self.declare_parameter("lock_duration_sec", 0.8)
        self.declare_parameter("rotation_timeout_sec", 25.0)
        self.declare_parameter("legs_timeout_sec", 20.0)
        self.declare_parameter("leg_extension_retracted_m", 0.0)
        self.declare_parameter("leg_extension_deployed_m", 0.17)
        self.declare_parameter("leg_extension_tolerance_m", 0.008)
        self.declare_parameter("min_leg_extend_sec", 0.0)
        self.declare_parameter("min_tracks_rotate_sec", 0.0)
        self.declare_parameter("min_aerial_ready_sec", 0.0)
        self.declare_parameter("disable_leg_commands", False)
        self.declare_parameter("airborne_z_threshold_m", 0.55)
        self.declare_parameter("spawn_z_m", 0.35)
        self.declare_parameter("model_frame", "marble_hd2/base_link")
        self.declare_parameter("left_track_frame", "marble_hd2/left_track")
        self.declare_parameter("right_track_frame", "marble_hd2/right_track")
        self.declare_parameter("use_pose_fallback", False)

        self._rotate_tracks_aerial = self.get_parameter("rotate_tracks_for_aerial").value
        self._left_yaw_aerial = self.get_parameter("left_track_yaw_aerial_rad").value
        self._right_yaw_aerial = self.get_parameter("right_track_yaw_aerial_rad").value
        self._yaw_tol = self.get_parameter("track_yaw_tolerance_rad").value
        self._lock_dur = self.get_parameter("lock_duration_sec").value
        self._rot_timeout = self.get_parameter("rotation_timeout_sec").value
        self._legs_timeout = self.get_parameter("legs_timeout_sec").value
        self._leg_retracted = self.get_parameter("leg_extension_retracted_m").value
        self._leg_deployed = self.get_parameter("leg_extension_deployed_m").value
        self._leg_tol = self.get_parameter("leg_extension_tolerance_m").value
        self._min_leg_extend_sec = max(
            0.0, float(self.get_parameter("min_leg_extend_sec").value)
        )
        self._min_tracks_rotate_sec = max(
            0.0, float(self.get_parameter("min_tracks_rotate_sec").value)
        )
        self._min_aerial_ready_sec = max(
            0.0, float(self.get_parameter("min_aerial_ready_sec").value)
        )
        self._disable_leg_commands = bool(
            self.get_parameter("disable_leg_commands").value
        )
        self._airborne_z = self.get_parameter("airborne_z_threshold_m").value
        self._spawn_z = self.get_parameter("spawn_z_m").value
        self._model_frame = self.get_parameter("model_frame").value
        self._left_track_frame = self.get_parameter("left_track_frame").value
        self._right_track_frame = self.get_parameter("right_track_frame").value
        self._use_pose_fallback = self.get_parameter("use_pose_fallback").value

        self._state = State.GROUND_DRIVE
        self._detail = "ready"
        self._left_yaw = 0.0
        self._right_yaw = 0.0
        self._leg_ext = 0.0
        self._leg_positions: list[float] = []
        self._base_z = self._spawn_z  # só telemetria (base_z_m), via pose_fused
        # Altitude AGL do ArduPilot (relativa ao home/takeoff), fonte de verdade do "airborne".
        # Independente do terreno e do EKF — o AP é a autoridade da pose no ar (design §6).
        self._ap_z: float = 0.0
        self._ap_valid: bool = False
        self._state_entered = time.monotonic()
        self._returning_to_ground = False
        self._js_seen = False
        self._pose_fallback_warned = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_status = self.create_publisher(
            HybridTransitionStatus, "/forest_gen/hybrid/transition_status", qos
        )
        self._pub_mode = self.create_publisher(
            OperationMode, "/system/locomotion_mode",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self._pub_cmd_vel = self.create_publisher(Twist, "/forest_gen/cmd_vel", 10)
        self._pub_left_yaw = self.create_publisher(
            Float64, "/forest_gen/hybrid/left_track_yaw_cmd", 10
        )
        self._pub_right_yaw = self.create_publisher(
            Float64, "/forest_gen/hybrid/right_track_yaw_cmd", 10
        )
        self._pub_left_track_drive = self.create_publisher(
            Float64, "/model/marble_hd2/link/left_track/track_cmd_vel", 10
        )
        self._pub_right_track_drive = self.create_publisher(
            Float64, "/model/marble_hd2/link/right_track/track_cmd_vel", 10
        )
        self._pub_legs = [
            self.create_publisher(Float64, f"/forest_gen/hybrid/support_leg_{n}_cmd", 10)
            for n in ("fl", "fr", "rl", "rr")
        ]

        self.create_subscription(
            String, "/forest_gen/hybrid/transition_request", self._on_request, 10
        )
        self.create_subscription(JointState, "/forest_gen/hybrid/joint_states", self._on_js, 10)
        self.create_subscription(
            TFMessage, "/forest_gen/gz/world_tf", self._on_world_tf, 10
        )
        # Pose_V via bridge has empty child_frame_id — use pose_fused (marble_pose_from_gz).
        self.create_subscription(PoseStamped, "/state/pose_fused", self._on_pose_fused, 10)
        # Altitude AGL do ArduPilot (z em ENU, relativo ao home) — gatilho de "airborne".
        self.create_subscription(
            Odometry, "/ardupilot/local_position_odom", self._on_ap_odom, 10
        )
        if self._use_pose_fallback:
            self.create_subscription(
                TFMessage, "/forest_gen/gz/world_tf_full", self._on_world_tf_full, 10
            )

        self.create_timer(0.1, self._tick)
        self._publish_track_yaw(0.0, 0.0)
        self._publish_legs(self._leg_retracted)
        self._publish_mode_ground()
        self.get_logger().info(
            "Hybrid transition manager ready. Publish 'to_aerial' on "
            "/forest_gen/hybrid/transition_request to start. "
            f"legs deployed={self._leg_deployed:.3f}m "
            f"min_dwell L/T/R={self._min_leg_extend_sec}/"
            f"{self._min_tracks_rotate_sec}/{self._min_aerial_ready_sec}s "
            f"leg_cmds={'off' if self._disable_leg_commands else 'on'}"
        )

    def _on_request(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == "to_aerial":
            if self._state in (State.GROUND_DRIVE, State.FAILED):
                self._returning_to_ground = False
                self._enter(State.TRANSITION_LOCK, "operator to_aerial")
            else:
                self.get_logger().warn(f"Ignore to_aerial in state {STATE_NAMES[self._state]}")
        elif cmd == "to_ground":
            self._begin_ground_return()
        elif cmd == "abort":
            self._begin_ground_return(detail="aborted")
        else:
            self.get_logger().warn(f"Unknown transition_request: {msg.data!r}")

    def _begin_ground_return(self, detail: str = "return to ground") -> None:
        if self._state == State.GROUND_DRIVE:
            return
        self._returning_to_ground = True
        if self._tracks_at_target(0.0, 0.0):
            self._enter(State.LEGS_RETRACTING, detail)
        else:
            self._enter(State.TRACKS_ROTATING, detail)

    def _on_js(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        self._js_seen = True
        legs: list[float] = []
        for name, pos in zip(msg.name, msg.position):
            if "left_track_yaw_joint" in name:
                self._left_yaw = float(pos)
            elif "right_track_yaw_joint" in name:
                self._right_yaw = float(pos)
            elif "support_leg_" in name and name.endswith("_joint"):
                legs.append(float(pos))
        if legs:
            self._leg_positions = legs
            self._leg_ext = sum(legs) / len(legs)

    @staticmethod
    def _roll_from_quat(x: float, y: float, z: float, w: float) -> float:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        return math.atan2(sinr_cosp, cosr_cosp)

    @staticmethod
    def _relative_roll(base_q, child_q) -> float:
        roll_b = HybridTransitionManager._roll_from_quat(*base_q)
        roll_c = HybridTransitionManager._roll_from_quat(*child_q)
        delta = roll_c - roll_b
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        return delta

    def _frame_matches(self, child_frame_id: str, target: str) -> bool:
        if not child_frame_id:
            return False
        return (
            child_frame_id == target
            or child_frame_id.endswith("/" + target.split("/")[-1])
            or child_frame_id.endswith("::" + target.split("/")[-1])
        )

    def _on_world_tf_full(self, msg: TFMessage) -> None:
        if self._js_seen:
            return
        base = None
        left = None
        right = None
        for t in msg.transforms:
            cid = t.child_frame_id
            if self._frame_matches(cid, self._model_frame):
                base = t
            elif self._frame_matches(cid, self._left_track_frame):
                left = t
            elif self._frame_matches(cid, self._right_track_frame):
                right = t
        if base is None or left is None or right is None:
            return
        bq = base.transform.rotation
        lq = left.transform.rotation
        rq = right.transform.rotation
        self._left_yaw = self._relative_roll(
            (bq.x, bq.y, bq.z, bq.w),
            (lq.x, lq.y, lq.z, lq.w),
        )
        self._right_yaw = self._relative_roll(
            (bq.x, bq.y, bq.z, bq.w),
            (rq.x, rq.y, rq.z, rq.w),
        )

    def _on_world_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.child_frame_id == self._model_frame:
                self._base_z = float(t.transform.translation.z)
                return

    def _on_pose_fused(self, msg: PoseStamped) -> None:
        self._base_z = float(msg.pose.position.z)

    def _on_ap_odom(self, msg: Odometry) -> None:
        # z em ENU = AGL acima do home/takeoff do ArduPilot (terreno-independente).
        self._ap_z = float(msg.pose.pose.position.z)
        self._ap_valid = True

    def _is_airborne(self) -> bool:
        return self._ap_valid and self._ap_z >= self._airborne_z

    def _enter(self, state: State, detail: str) -> None:
        self._state = state
        self._detail = detail
        self._state_entered = time.monotonic()
        self.get_logger().info(f"→ {STATE_NAMES[state]} ({detail})")

    def _elapsed(self) -> float:
        return time.monotonic() - self._state_entered

    def _publish_mode_ground(self) -> None:
        m = OperationMode()
        m.mode = OperationMode.MODE_GROUND
        m.mode_name = "ground"
        self._pub_mode.publish(m)

    def _publish_mode_aerial(self) -> None:
        m = OperationMode()
        m.mode = OperationMode.MODE_AERIAL
        m.mode_name = "aerial"
        self._pub_mode.publish(m)

    def _zero_ground_cmd(self) -> None:
        self._pub_cmd_vel.publish(Twist())

    def _zero_track_drive(self) -> None:
        z = Float64()
        z.data = 0.0
        self._pub_left_track_drive.publish(z)
        self._pub_right_track_drive.publish(z)

    def _publish_track_yaw(self, left: float, right: float) -> None:
        l = Float64()
        l.data = left
        r = Float64()
        r.data = right
        self._pub_left_yaw.publish(l)
        self._pub_right_yaw.publish(r)

    def _publish_legs(self, extension_m: float) -> None:
        m = Float64()
        m.data = float(extension_m)
        for pub in self._pub_legs:
            pub.publish(m)

    def _legs_deployed_states(self) -> bool:
        if self._state == State.LEGS_RETRACTING:
            return False
        return self._state in (
            State.LEGS_EXTENDING,
            State.TRACKS_ROTATING,
            State.AERIAL_READY,
            State.AERIAL_FLY,
            State.AERIAL_HOVER,
        )

    def _leg_extension_cmd(self) -> float:
        # As pernas ficam estendidas durante TODO o voo (servem de trem de aterragem):
        # estendem antes de rodar as lagartas, mantêm-se no hover e na descida, e só
        # retraem em LEGS_RETRACTING — depois de pousar e com as lagartas já a 0°.
        # Retrair em voo faria o robô pousar sobre o chassis/lagartas (sem suporte).
        if self._legs_deployed_states():
            return self._leg_deployed
        return self._leg_retracted

    def _track_cmd_targets(self) -> tuple[float, float]:
        if self._returning_to_ground:
            return 0.0, 0.0
        if self._state == State.TRACKS_ROTATING and self._returning_to_ground:
            return 0.0, 0.0
        if (
            self._rotate_tracks_aerial
            and self._state
            in (
                State.TRACKS_ROTATING,
                State.AERIAL_READY,
                State.AERIAL_FLY,
                State.AERIAL_HOVER,
            )
        ):
            return self._left_yaw_aerial, self._right_yaw_aerial
        return 0.0, 0.0

    def _tracks_at_target(self, left_tgt: float, right_tgt: float) -> bool:
        return (
            abs(self._left_yaw - left_tgt) <= self._yaw_tol
            and abs(self._right_yaw - right_tgt) <= self._yaw_tol
        )

    def _legs_at_target(self, target_m: float) -> bool:
        if len(self._leg_positions) < 4:
            return False
        return all(abs(p - target_m) <= self._leg_tol for p in self._leg_positions)

    def _tick(self) -> None:
        left_tgt, right_tgt = self._track_cmd_targets()
        leg_tgt = self._leg_extension_cmd()
        self._publish_track_yaw(left_tgt, right_tgt)
        if not self._disable_leg_commands:
            self._publish_legs(leg_tgt)
        if self._state != State.GROUND_DRIVE:
            self._zero_ground_cmd()
            self._zero_track_drive()

        if self._state == State.TRANSITION_LOCK:
            self._zero_ground_cmd()
            if self._elapsed() >= self._lock_dur:
                self._enter(State.LEGS_EXTENDING, "deploy support legs")

        elif self._state == State.LEGS_EXTENDING:
            self._zero_ground_cmd()
            legs_ready = self._disable_leg_commands or self._legs_at_target(
                self._leg_deployed
            )
            if legs_ready and self._elapsed() >= self._min_leg_extend_sec:
                if self._rotate_tracks_aerial:
                    self._enter(
                        State.TRACKS_ROTATING,
                        f"rotate tracks L={math.degrees(self._left_yaw_aerial):.0f}° "
                        f"R={math.degrees(self._right_yaw_aerial):.0f}°",
                    )
                else:
                    self._enter(State.AERIAL_READY, "legs deployed (rotate_tracks_for_aerial=false)")
            elif self._elapsed() > self._legs_timeout:
                self._enter(
                    State.FAILED,
                    f"leg deploy timeout (ext={self._leg_ext:.3f} tgt={self._leg_deployed})",
                )

        elif self._state == State.TRACKS_ROTATING:
            self._zero_ground_cmd()
            if (
                not self._js_seen
                and self._use_pose_fallback
                and self._elapsed() > 2.0
                and not self._pose_fallback_warned
            ):
                self._pose_fallback_warned = True
                self.get_logger().warn(
                    "No joint_states yet; using world_tf_full pose fallback for track yaw"
                )
            tracks_ready = self._js_seen and self._tracks_at_target(left_tgt, right_tgt)
            if tracks_ready and self._elapsed() >= self._min_tracks_rotate_sec:
                if self._returning_to_ground:
                    self._enter(State.LEGS_RETRACTING, "retract support legs")
                else:
                    self._enter(State.AERIAL_READY, "tracks rotated to aerial (±90°)")
            elif self._elapsed() > self._rot_timeout:
                self._enter(
                    State.FAILED,
                    "track rotation timeout "
                    f"(L={math.degrees(self._left_yaw):.1f}°/"
                    f"{math.degrees(left_tgt):.1f}° "
                    f"R={math.degrees(self._right_yaw):.1f}°/"
                    f"{math.degrees(right_tgt):.1f}° "
                    f"legs={self._leg_ext:.3f})",
                )

        elif self._state == State.LEGS_RETRACTING:
            self._zero_ground_cmd()
            if self._legs_at_target(self._leg_retracted):
                self._returning_to_ground = False
                self._publish_mode_ground()
                self._enter(State.GROUND_DRIVE, "legs retracted, tracks at 0°")
            elif self._elapsed() > self._legs_timeout:
                self._enter(State.FAILED, f"leg retract timeout (ext={self._leg_ext:.3f})")

        elif self._state == State.AERIAL_READY:
            self._zero_ground_cmd()
            if self._elapsed() < self._min_aerial_ready_sec:
                return
            # Pronto a voar: o ArduPilot (via hybrid_hop_executor) arma e descola.
            self._publish_mode_aerial()
            self._enter(State.AERIAL_FLY, "aerial mode (ArduPilot controla o voo)")

        elif self._state == State.AERIAL_FLY:
            # O FSM apenas observa a subida (AGL do AP); a propulsão é do ArduPilot.
            self._zero_ground_cmd()
            if self._is_airborne():
                self._enter(State.AERIAL_HOVER, "airborne")

        elif self._state == State.AERIAL_HOVER:
            self._zero_ground_cmd()

        elif self._state == State.FAILED:
            self._zero_ground_cmd()

        self._publish_status(left_tgt, right_tgt, leg_tgt)

    def _publish_status(self, left_tgt: float, right_tgt: float, leg_tgt: float) -> None:
        s = HybridTransitionStatus()
        s.state = int(self._state)
        s.state_name = STATE_NAMES.get(self._state, "UNKNOWN")
        s.detail = self._detail
        s.left_track_yaw_rad = self._left_yaw
        s.right_track_yaw_rad = self._right_yaw
        s.leg_extension_m = self._leg_ext
        s.base_z_m = self._base_z
        # Sem controlador multicopter ROS: a propulsão aérea é toda do ArduPilot.
        s.multicopter_enabled = False
        # airborne via AGL do ArduPilot (relativo ao home) — terreno-independente, sem EKF.
        s.airborne = self._is_airborne()
        self._pub_status.publish(s)


def main() -> None:
    rclpy.init()
    node = HybridTransitionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

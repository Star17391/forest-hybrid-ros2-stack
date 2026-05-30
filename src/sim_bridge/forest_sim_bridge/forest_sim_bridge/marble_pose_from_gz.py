#!/usr/bin/env python3
"""Publish ``/state/pose_fused`` and ``map -> marble_hd2/base_link`` from Gazebo.

Uses ``dynamic_pose`` (only moving entities) as primary source and
``pose/info`` (all entities) as fallback.  Picks the robot by matching
``model_name`` in the ``child_frame_id`` when populated, otherwise uses
the **best moving transform** (highest cumulative motion score) and latches
the index for the rest of the session.

When a real localizer (FAST-LIO2, etc.) is available, it publishes
``/state/pose_fused`` directly and this bridge is disabled.
"""

from __future__ import annotations

import copy
import math

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Quaternion, Transform, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def _yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _stamp_is_zero(stamp: TimeMsg) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


class MarblePoseFromGz(Node):
    def __init__(self) -> None:
        super().__init__("marble_pose_from_gz")

        self.declare_parameter("source_topic", "/forest_gen/gz/world_tf")
        self.declare_parameter("fallback_source_topic", "/forest_gen/gz/world_tf_full")
        self.declare_parameter("fallback_timeout_sec", 1.0)
        self.declare_parameter("model_name", "marble_hd2")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "marble_hd2/base_link")
        self.declare_parameter("republish_hz", 20.0)
        self.declare_parameter("stale_tf_max_age_sec", 0.5)
        self.declare_parameter("seed_x", 0.0)
        self.declare_parameter("seed_y", 0.0)
        self.declare_parameter("seed_z", 0.35)

        src = self.get_parameter("source_topic").get_parameter_value().string_value
        fb = self.get_parameter("fallback_source_topic").get_parameter_value().string_value
        self._fallback_topic = fb if fb and fb != src else ""
        self._fallback_timeout = max(
            0.2,
            self.get_parameter("fallback_timeout_sec").get_parameter_value().double_value,
        )
        self._model = self.get_parameter("model_name").get_parameter_value().string_value
        self._parent = self.get_parameter("parent_frame").get_parameter_value().string_value
        self._child = self.get_parameter("child_frame").get_parameter_value().string_value
        hz = max(1.0, self.get_parameter("republish_hz").get_parameter_value().double_value)
        self._stale_tf_max_age = max(
            0.05, self.get_parameter("stale_tf_max_age_sec").get_parameter_value().double_value
        )
        self._seed_x = self.get_parameter("seed_x").get_parameter_value().double_value
        self._seed_y = self.get_parameter("seed_y").get_parameter_value().double_value
        self._seed_z = self.get_parameter("seed_z").get_parameter_value().double_value

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_pose = self.create_publisher(PoseStamped, "/state/pose_fused", 10)
        self._broadcaster = TransformBroadcaster(self)

        self.create_subscription(TFMessage, src, self._on_tf_primary, qos)
        self._last_primary_rx = self.get_clock().now()
        if self._fallback_topic:
            self.create_subscription(TFMessage, self._fallback_topic, self._on_tf_fallback, qos)

        self._got_any_pose = False
        self._last_geom: Transform | None = None
        self._last_source_stamp: TimeMsg | None = None

        self._latched_tf_index: int | None = None
        self._tf_index_origin: dict[int, tuple[float, float, float]] = {}
        self._tf_index_motion: dict[int, float] = {}

        self._timer = self.create_timer(1.0 / hz, self._tick_republish)

        self.get_logger().info(
            f"-> /state/pose_fused + /tf "
            f"(primary={src}, fallback={self._fallback_topic or 'none'}) "
            f"republish {hz:.0f} Hz"
        )

    # ── Named transform matching ──

    def _matches_model(self, child_frame_id: str) -> bool:
        if not child_frame_id:
            return False
        m = self._model
        return (
            child_frame_id == m
            or child_frame_id == self._child
            or child_frame_id.startswith(m + "/")
            or child_frame_id.startswith(m + "::")
        )

    def _pick_named(self, msg: TFMessage) -> TransformStamped | None:
        candidates = [tf for tf in msg.transforms if self._matches_model(tf.child_frame_id)]
        if not candidates:
            return None

        def rank(child: str) -> tuple[int, int]:
            if child in (self._child, f"{self._model}/base_link", f"{self._model}::base_link"):
                return (0, len(child))
            if child == self._model:
                return (1, len(child))
            return (2, len(child))

        candidates.sort(key=lambda tf: rank(tf.child_frame_id))
        return candidates[0]

    # ── Unnamed transform matching (Pose_V with empty frame_ids) ──

    def _motion_score(self, idx: int, geom: Transform) -> float:
        x, y = geom.translation.x, geom.translation.y
        yaw = _yaw_from_quat(geom.rotation)
        if idx not in self._tf_index_origin:
            self._tf_index_origin[idx] = (x, y, yaw)
        ox, oy, oyaw = self._tf_index_origin[idx]
        score = (x - ox) ** 2 + (y - oy) ** 2 + (abs(_normalize_angle(yaw - oyaw)) * 0.35) ** 2
        self._tf_index_motion[idx] = max(self._tf_index_motion.get(idx, 0.0), score)
        return score

    def _pick_unnamed(self, msg: TFMessage) -> TransformStamped | None:
        if not msg.transforms:
            return None

        if self._latched_tf_index is not None:
            idx = self._latched_tf_index
            if idx < len(msg.transforms):
                return msg.transforms[idx]
            return None

        best_idx: int | None = None
        best_score = -1.0
        for idx, tf in enumerate(msg.transforms):
            self._motion_score(idx, tf.transform)
            hist = self._tf_index_motion.get(idx, 0.0)
            if hist > best_score:
                best_score = hist
                best_idx = idx

        if best_idx is not None and best_score > 0.01:
            self._latched_tf_index = best_idx
            self.get_logger().info(
                f"Latched Gazebo transform index {best_idx} "
                f"(motion={math.sqrt(best_score):.3f} m)"
            )
            return msg.transforms[best_idx]

        if best_idx is not None:
            return msg.transforms[best_idx]
        return None

    # ── Timestamp handling ──

    def _stamp_age_sec(self, stamp: TimeMsg) -> float:
        if _stamp_is_zero(stamp):
            return 999.0
        now = self.get_clock().now()
        try:
            msg_time = Time.from_msg(stamp)
        except Exception:
            return 999.0
        if not now.nanoseconds or not msg_time.nanoseconds:
            return 999.0
        return max(0.0, (now - msg_time).nanoseconds * 1e-9)

    def _best_stamp(self, source_stamp: TimeMsg) -> TimeMsg:
        """Use source stamp when valid, otherwise node clock."""
        if _stamp_is_zero(source_stamp):
            return self.get_clock().now().to_msg()
        return source_stamp

    # ── Publishing ──

    def _publish(self, geom: Transform, stamp: TimeMsg) -> None:
        self._last_geom = copy.deepcopy(geom)
        self._last_source_stamp = stamp

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self._parent
        ps.pose.position.x = geom.translation.x
        ps.pose.position.y = geom.translation.y
        ps.pose.position.z = geom.translation.z
        ps.pose.orientation = geom.rotation
        self._pub_pose.publish(ps)

        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = self._parent
        out.child_frame_id = self._child
        out.transform = geom
        self._broadcaster.sendTransform(out)

    # ── Callbacks ──

    def _consume_tf(self, msg: TFMessage, source: str) -> None:
        chosen = self._pick_named(msg)
        if chosen is None:
            chosen = self._pick_unnamed(msg)
        if chosen is None:
            return

        stamp = self._best_stamp(chosen.header.stamp)
        age = self._stamp_age_sec(stamp)
        if age > self._stale_tf_max_age:
            return

        if not self._got_any_pose:
            label = chosen.child_frame_id or f"index[{self._latched_tf_index}]"
            self.get_logger().info(f"First pose from {source} '{label}' -- publishing.")
            self._got_any_pose = True

        self._publish(chosen.transform, stamp)

    def _on_tf_primary(self, msg: TFMessage) -> None:
        self._last_primary_rx = self.get_clock().now()
        self._consume_tf(msg, "primary")

    def _on_tf_fallback(self, msg: TFMessage) -> None:
        age_primary = (self.get_clock().now() - self._last_primary_rx).nanoseconds * 1e-9
        if age_primary < self._fallback_timeout:
            return
        self._consume_tf(msg, "fallback")

    def _tick_republish(self) -> None:
        """Re-publica a última pose com o stamp Gazebo (não avançar /clock).

        Avançar o stamp a cada 20 Hz sem nova pose do Gazebo faz o RViz
        interpolar/extrapolar no tempo de simulação mais depressa que o modelo 3D.
        """
        if self._last_geom is None:
            return
        stamp = self._last_source_stamp
        if stamp is None or _stamp_is_zero(stamp):
            stamp = self.get_clock().now().to_msg()
        self._publish(self._last_geom, stamp)


def main() -> None:
    rclpy.init()
    node = MarblePoseFromGz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

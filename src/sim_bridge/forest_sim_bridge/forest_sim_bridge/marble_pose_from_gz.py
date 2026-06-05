#!/usr/bin/env python3
"""Publish ``/state/pose_fused`` and ``map -> marble_hd2/base_link`` from Gazebo.

Primary source: Gazebo ``pose/info`` read **directly via gz.transport**, where
each pose carries its entity ``name``.  We select the entry whose name equals
``model_name`` (the model's canonical link = base_link in world frame).  This is
robust: the ros_gz_bridge strips entity names when converting ``Pose_V`` to
``TFMessage`` (all ``child_frame_id`` come out empty), so the bridged topics
cannot be matched by name on the ROS side.

The bridged ``TFMessage`` topics remain as a degraded fallback (index heuristic)
only if the gz.transport source is unavailable.

When a real localizer (FAST-LIO2, etc.) is available, it publishes
``/state/pose_fused`` directly and this bridge is disabled.
"""

from __future__ import annotations

import copy
import math
import threading

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Quaternion, Transform, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

try:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzTransportNode

    _GZ_TRANSPORT_OK = True
except ImportError:  # pragma: no cover - gz python bindings absent
    _GZ_TRANSPORT_OK = False


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

        # pose/info tem frame_id (base_link); dynamic_pose não — latch em hélice = pião no RViz
        self.declare_parameter("source_topic", "/forest_gen/gz/world_tf_full")
        self.declare_parameter("fallback_source_topic", "/forest_gen/gz/world_tf")
        self.declare_parameter("fallback_timeout_sec", 1.0)
        self.declare_parameter("model_name", "marble_hd2")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "marble_hd2/base_link")
        self.declare_parameter("republish_hz", 20.0)
        self.declare_parameter("stale_tf_max_age_sec", 0.5)
        self.declare_parameter("seed_x", 0.0)
        self.declare_parameter("seed_y", 0.0)
        self.declare_parameter("seed_z", 0.35)
        # Authoritative source: gz pose/info read directly (carries entity names).
        self.declare_parameter("gz_pose_topic", "/world/unified_world/pose/info")
        self.declare_parameter("use_gz_direct", True)
        # If gz-direct delivered within this window, ignore the bridged TF fallback.
        self.declare_parameter("gz_direct_timeout_sec", 0.5)

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
        self._gz_pose_topic = (
            self.get_parameter("gz_pose_topic").get_parameter_value().string_value
        )
        self._use_gz_direct = (
            self.get_parameter("use_gz_direct").get_parameter_value().bool_value
            and _GZ_TRANSPORT_OK
        )
        self._gz_direct_timeout = max(
            0.1, self.get_parameter("gz_direct_timeout_sec").get_parameter_value().double_value
        )

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
        self._tf_index_history: dict[int, list[tuple[float, float, float, float]]] = {}
        self._unnamed_samples = 0

        # gz.transport direct source (authoritative, name-matched).
        self._gz_lock = threading.Lock()
        self._gz_last_rx = self.get_clock().now()
        self._gz_node: GzTransportNode | None = None
        if self._use_gz_direct:
            self._gz_node = GzTransportNode()
            if self._gz_node.subscribe(Pose_V, self._gz_pose_topic, self._on_gz_pose):
                self.get_logger().info(
                    f"gz.transport pose source: {self._gz_pose_topic} "
                    f"(matching entity name '{self._model}')"
                )
            else:
                self.get_logger().error(
                    f"gz.transport subscribe failed on {self._gz_pose_topic}; "
                    f"falling back to bridged TF heuristic"
                )
                self._use_gz_direct = False

        self._timer = self.create_timer(1.0 / hz, self._tick_republish)

        self.get_logger().info(
            f"-> /state/pose_fused + /tf "
            f"(gz_direct={self._use_gz_direct}, primary={src}, "
            f"fallback={self._fallback_topic or 'none'}) republish {hz:.0f} Hz"
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

    def _stability_cost(self, idx: int, geom: Transform) -> float:
        """Lower is better. Reject spinning props (high yaw rate) and world origin."""
        z = float(geom.translation.z)
        if z < 0.12 or z > 6.0:
            return float("inf")
        x, y = geom.translation.x, geom.translation.y
        if abs(x) < 1e-4 and abs(y) < 1e-4:
            return float("inf")
        q = geom.rotation
        yaw = _yaw_from_quat(q)
        hist = self._tf_index_history.setdefault(idx, [])
        hist.append((x, y, yaw, z))
        if len(hist) > 12:
            del hist[:-12]
        if len(hist) < 4:
            return float("inf")
        max_dyaw = 0.0
        max_dxy = 0.0
        for i in range(1, len(hist)):
            max_dyaw = max(
                max_dyaw, abs(_normalize_angle(hist[i][2] - hist[i - 1][2]))
            )
            max_dxy = max(
                max_dxy,
                math.hypot(hist[i][0] - hist[i - 1][0], hist[i][1] - hist[i - 1][1]),
            )
        return abs(z - self._seed_z) + max_dyaw * 8.0 + max_dxy * 2.0

    def _pick_unnamed(self, msg: TFMessage) -> TransformStamped | None:
        if not msg.transforms:
            return None

        if self._latched_tf_index is not None:
            idx = self._latched_tf_index
            if idx < len(msg.transforms):
                return msg.transforms[idx]
            return None

        self._unnamed_samples += 1
        best_idx: int | None = None
        best_cost = float("inf")
        for idx, tf in enumerate(msg.transforms):
            cost = self._stability_cost(idx, tf.transform)
            if cost < best_cost:
                best_cost = cost
                best_idx = idx

        if best_idx is not None and best_cost < float("inf") and self._unnamed_samples >= 8:
            self._latched_tf_index = best_idx
            z = msg.transforms[best_idx].transform.translation.z
            self.get_logger().info(
                f"Latched stable Pose_V index {best_idx} (cost={best_cost:.2f}, z={z:.3f}) "
                f"— evita hélice em rotação"
            )
            return msg.transforms[best_idx]

        if best_idx is not None and self._unnamed_samples < 8:
            return msg.transforms[best_idx]
        return None

    # ── gz.transport direct source (authoritative, name-matched) ──

    def _gz_match(self, name: str) -> int:
        """Selection priority for a gz entity name (lower = better, -1 = reject)."""
        if name == self._child or name == f"{self._model}/base_link":
            return 0
        if name == self._model:
            return 1
        if name == "base_link":
            return 2
        return -1

    def _on_gz_pose(self, msg: Pose_V) -> None:
        """Runs on a gz.transport thread; stash the named base pose for the ROS timer."""
        best_rank = 99
        best = None
        for p in msg.pose:
            r = self._gz_match(p.name)
            if r >= 0 and r < best_rank:
                best_rank = r
                best = p
        if best is None:
            return

        geom = Transform()
        geom.translation.x = float(best.position.x)
        geom.translation.y = float(best.position.y)
        geom.translation.z = float(best.position.z)
        geom.rotation.x = float(best.orientation.x)
        geom.rotation.y = float(best.orientation.y)
        geom.rotation.z = float(best.orientation.z)
        geom.rotation.w = float(best.orientation.w)

        stamp = TimeMsg()
        stamp.sec = int(msg.header.stamp.sec)
        stamp.nanosec = int(msg.header.stamp.nsec)

        with self._gz_lock:
            self._last_geom = geom
            self._last_source_stamp = stamp
            self._gz_last_rx = self.get_clock().now()
            if not self._got_any_pose:
                self.get_logger().info(
                    f"First pose from gz.transport '{best.name}' "
                    f"(z={geom.translation.z:.3f}) -- publishing."
                )
                self._got_any_pose = True

    def _gz_direct_fresh(self) -> bool:
        if not self._use_gz_direct:
            return False
        age = (self.get_clock().now() - self._gz_last_rx).nanoseconds * 1e-9
        return age <= self._gz_direct_timeout

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
        # Authoritative gz.transport source wins; ignore the bridged (name-stripped) TF.
        if self._gz_direct_fresh():
            return
        chosen = self._pick_named(msg)
        if chosen is None:
            chosen = self._pick_unnamed(msg)
        if chosen is None:
            return

        g = chosen.transform
        if self._latched_tf_index is not None and abs(g.translation.x) < 1e-6 and abs(
            g.translation.y
        ) < 1e-6:
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
        with self._gz_lock:
            geom = self._last_geom
            stamp = self._last_source_stamp
        if geom is None:
            return
        if stamp is None or _stamp_is_zero(stamp):
            stamp = self.get_clock().now().to_msg()
        self._publish(geom, stamp)


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

#!/usr/bin/env python3
"""Bootstrap odom -> marble_hd2/base_link from Gazebo world TF (modo LiDAR 3D).

Enquanto o EKF não publica (wheel odom ainda sem dados ou GPU LiDAR a arrancar),
evita RViz com laser/base_link órfãos. Desactiva-se automaticamente quando
odom -> marble_hd2/base_link aparece em /tf.
"""

from __future__ import annotations

import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class GzOdomBaseBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("gz_odom_base_bootstrap")
        self.declare_parameter("source_topic", "/forest_gen/gz/world_tf")
        self.declare_parameter("model_name", "marble_hd2")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "marble_hd2/base_link")
        self.declare_parameter("republish_hz", 20.0)

        src = self.get_parameter("source_topic").get_parameter_value().string_value
        self._model = self.get_parameter("model_name").get_parameter_value().string_value
        self._parent = self.get_parameter("parent_frame").get_parameter_value().string_value
        self._child = self.get_parameter("child_frame").get_parameter_value().string_value
        hz = max(1.0, self.get_parameter("republish_hz").get_parameter_value().double_value)

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._br = TransformBroadcaster(self)
        self._last: TransformStamped | None = None
        self._ekf_active = False

        self.create_subscription(TFMessage, src, self._on_gz, qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 20)
        self._bootstrap_timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"Bootstrap {self._parent} -> {self._child} from {src} until EKF publishes"
        )

    def _matches(self, child: str) -> bool:
        if not child:
            return False
        m = self._model
        return (
            child == self._child
            or child == f"{m}/base_link"
            or child.startswith(m + "/")
            or child.startswith(m + "::")
        )

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.header.frame_id == self._parent and t.child_frame_id == self._child:
                if not self._ekf_active:
                    self._ekf_active = True
                    self._bootstrap_timer.cancel()
                    self._last = None
                    self.get_logger().info(
                        "EKF publica odom->base_link; bootstrap TF desactivado"
                    )
                return

    def _on_gz(self, msg: TFMessage) -> None:
        if self._ekf_active:
            return
        for t in msg.transforms:
            if not self._matches(t.child_frame_id):
                continue
            out = TransformStamped()
            out.header.stamp = t.header.stamp
            out.header.frame_id = self._parent
            out.child_frame_id = self._child
            out.transform = copy.deepcopy(t.transform)
            self._last = out
            self._br.sendTransform(out)
            return

    def _tick(self) -> None:
        """Re-publica com sim-clock actual (garante que TF buffer está 'fresco')."""
        if self._ekf_active or self._last is None:
            return
        msg = copy.deepcopy(self._last)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._br.sendTransform(msg)


def main() -> None:
    rclpy.init()
    node = GzOdomBaseBootstrap()
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

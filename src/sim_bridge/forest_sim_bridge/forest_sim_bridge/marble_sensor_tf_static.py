#!/usr/bin/env python3
"""Static TF: marble_hd2/base_link -> camera frames (sim only).

LiDAR extrinsics are published by ``forest_sensors_cpp/static_sensor_tf_node``
from ``forest_lidar_extrinsics.yaml`` (frame ``laser``, 25 deg tilt-down).

Offsets match camera <pose> elements in ForestGen ``model.sdf``.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

# (child_frame, x, y, z, roll, pitch, yaw) in metres / radians
# Extracted from model.sdf sensor <pose> elements.
_SENSOR_LINKS: tuple[tuple[str, float, float, float, float, float, float], ...] = (
    ("marble_hd2/camera_front_optical", 0.40, 0.0, 0.24, 0.0, 0.0, 0.0),
    ("marble_hd2/camera_front_depth", 0.40, 0.0, 0.24, 0.0, 0.0, 0.0),
    ("marble_hd2/camera_down_optical", 0.35, 0.0, 0.14, 0.0, 0.55, 0.0),
)


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class MarbleSensorTfStatic(Node):
    def __init__(self) -> None:
        super().__init__("marble_sensor_tf_static")
        self.declare_parameter("parent_frame", "marble_hd2/base_link")
        self.declare_parameter("republish_period_sec", 5.0)
        parent = self.get_parameter("parent_frame").get_parameter_value().string_value
        period = max(0.5, self.get_parameter("republish_period_sec").get_parameter_value().double_value)

        self._br = StaticTransformBroadcaster(self)
        self._parent = parent
        self._links = list(_SENSOR_LINKS)
        self._timer = self.create_timer(period, self._tick)
        self._tick()
        self.get_logger().info(
            f"{len(self._links)} static TFs {parent} -> sensors (republish {period:.1f} s)"
        )

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        out: list[TransformStamped] = []
        for child, x, y, z, rr, rp, ry in self._links:
            qx, qy, qz, qw = _quat_from_rpy(rr, rp, ry)
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._parent
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            out.append(t)
        if out:
            self._br.sendTransform(out)


def main() -> None:
    rclpy.init()
    node = MarbleSensorTfStatic()
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

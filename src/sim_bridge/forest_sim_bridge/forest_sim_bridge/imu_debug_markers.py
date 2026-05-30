#!/usr/bin/env python3
"""RViz markers for IMU: gravity vector + gyro magnitude (Jazzy has no Imu display)."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray


class ImuDebugMarkers(Node):
    def __init__(self) -> None:
        super().__init__("imu_debug_markers")
        self.declare_parameter("imu_topic", "/sensors/imu/data_raw")
        self.declare_parameter("scale", 0.08)
        topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self._scale = self.get_parameter("scale").get_parameter_value().double_value
        self._pub = self.create_publisher(MarkerArray, "/diagnostics/imu/markers", 10)
        self.create_subscription(Imu, topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f"IMU debug markers <- {topic} -> /diagnostics/imu/markers")

    def _cb(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        g = math.sqrt(ax * ax + ay * ay + az * az)
        if g < 1e-3:
            return

        sx = self._scale * ax / g
        sy = self._scale * ay / g
        sz = self._scale * az / g

        arr = MarkerArray()
        grav = Marker()
        grav.header = msg.header
        grav.ns = "imu_gravity"
        grav.id = 0
        grav.type = Marker.ARROW
        grav.action = Marker.ADD
        grav.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=sx, y=sy, z=sz)]
        grav.scale.x = 0.02
        grav.scale.y = 0.04
        grav.scale.z = 0.04
        grav.color.r = 0.1
        grav.color.g = 0.9
        grav.color.b = 0.2
        grav.color.a = 1.0
        arr.markers.append(grav)

        text = Marker()
        text.header = msg.header
        text.ns = "imu_stats"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.z = 0.5
        text.scale.z = 0.08
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        gz = msg.angular_velocity.z
        text.text = f"|g|={g:.2f} wz={gz:.4f}"
        arr.markers.append(text)

        self._pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = ImuDebugMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

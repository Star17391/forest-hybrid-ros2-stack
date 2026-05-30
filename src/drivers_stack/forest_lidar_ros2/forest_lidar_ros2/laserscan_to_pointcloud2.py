#!/usr/bin/env python3
"""Convert sensor_msgs/LaserScan to sensor_msgs/PointCloud2 on the forest contract topic."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class LaserScanToPointCloud2(Node):
  def __init__(self) -> None:
    super().__init__("laserscan_to_pointcloud2")
    self.declare_parameter("scan_topic", "/scan")
    self.declare_parameter("cloud_topic", "/sensors/lidar/points")
    self.declare_parameter("min_range_m", 0.12)
    self.declare_parameter("max_range_m", 10.0)

    scan_topic = self.get_parameter("scan_topic").value
    cloud_topic = self.get_parameter("cloud_topic").value
    self._min_r = float(self.get_parameter("min_range_m").value)
    self._max_r = float(self.get_parameter("max_range_m").value)

    # ydlidar_ros2_driver publishes /scan with SensorDataQoS (best effort).
    self._pub = self.create_publisher(
        PointCloud2, cloud_topic, qos_profile_sensor_data
    )
    self._sub = self.create_subscription(
        LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
    )
    self.get_logger().info(f"{scan_topic} -> {cloud_topic}")

  def _on_scan(self, scan: LaserScan) -> None:
    points: list[list[float]] = []
    angle = scan.angle_min
    for r in scan.ranges:
      if (
        math.isfinite(r)
        and scan.range_min <= r <= scan.range_max
        and self._min_r <= r <= self._max_r
      ):
        points.append([r * math.cos(angle), r * math.sin(angle), 0.0])
      angle += scan.angle_increment

    cloud = point_cloud2.create_cloud_xyz32(scan.header, points)
    self._pub.publish(cloud)


def main() -> None:
  rclpy.init()
  node = LaserScanToPointCloud2()
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

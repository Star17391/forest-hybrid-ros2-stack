#!/usr/bin/env python3
"""Late fusion: project lidar points into semantic mask and attach class labels."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class SemanticPointFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_point_fusion_node")
        self.declare_parameter("points_topic", "/sensors/lidar/points")
        self.declare_parameter("mask_topic", "/perception/semantic_mask")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("output_topic", "/perception/semantic_points")
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 60.0)
        self.declare_parameter(
            "extrinsics_lidar_to_camera_row_major",
            [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )

        self._mask: np.ndarray | None = None
        self._cam_info: CameraInfo | None = None
        self._k: tuple[float, float, float, float] | None = None

        points_topic = self.get_parameter("points_topic").value
        mask_topic = self.get_parameter("mask_topic").value
        cam_topic = self.get_parameter("camera_info_topic").value
        out_topic = self.get_parameter("output_topic").value

        extr = self.get_parameter("extrinsics_lidar_to_camera_row_major").value
        self._t_lc = np.asarray(extr, dtype=np.float64).reshape(4, 4)
        self._min_depth = float(self.get_parameter("min_depth_m").value)
        self._max_depth = float(self.get_parameter("max_depth_m").value)

        qos = rclpy.qos.qos_profile_sensor_data
        self._mask_sub = self.create_subscription(Image, mask_topic, self._on_mask, qos)
        self._cam_sub = self.create_subscription(CameraInfo, cam_topic, self._on_cam_info, qos)
        self._pc_sub = self.create_subscription(PointCloud2, points_topic, self._on_points, qos)
        self._pub = self.create_publisher(PointCloud2, out_topic, qos)

        self.get_logger().info(
            f"semantic fusion ready: points={points_topic} mask={mask_topic} cam_info={cam_topic} -> {out_topic}"
        )

    def _on_cam_info(self, msg: CameraInfo) -> None:
        self._cam_info = msg
        self._k = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def _on_mask(self, msg: Image) -> None:
        if msg.encoding != "mono8":
            self.get_logger().warn(
                f"semantic mask encoding {msg.encoding} unsupported, expected mono8",
                throttle_duration_sec=2.0,
            )
            return
        self._mask = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))

    def _on_points(self, msg: PointCloud2) -> None:
        if self._mask is None or self._cam_info is None or self._k is None:
            return

        fx, fy, cx, cy = self._k
        h, w = self._mask.shape

        labeled_points: list[tuple[float, float, float, int]] = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x_l, y_l, z_l = float(p[0]), float(p[1]), float(p[2])
            p_l = np.array([x_l, y_l, z_l, 1.0], dtype=np.float64)
            p_c = self._t_lc @ p_l
            x_c, y_c, z_c = float(p_c[0]), float(p_c[1]), float(p_c[2])
            if z_c < self._min_depth or z_c > self._max_depth:
                continue

            u = int(round((fx * x_c / z_c) + cx))
            v = int(round((fy * y_c / z_c) + cy))
            if 0 <= u < w and 0 <= v < h:
                label = int(self._mask[v, u])
                labeled_points.append((x_l, y_l, z_l, label))

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="label", offset=12, datatype=PointField.UINT8, count=1),
        ]
        out = point_cloud2.create_cloud(msg.header, fields, labeled_points)
        self._pub.publish(out)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticPointFusionNode()
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


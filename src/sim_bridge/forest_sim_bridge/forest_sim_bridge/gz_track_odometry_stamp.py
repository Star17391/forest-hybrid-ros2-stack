#!/usr/bin/env python3
"""Preenche header.frame_id e child_frame_id em Odometry vinda do Gazebo (esteiras).

O ros_gz_bridge injecta só ``header.frame_id`` a partir do YAML; ``child_frame_id``
fica vazio nos gz.msgs.Odometry do MARBLE → o RViz MessageFilter descarta com
'frame id of the message is empty'.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class GzTrackOdometryStamp(Node):
    def __init__(self) -> None:
        super().__init__("gz_track_odometry_stamp")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("left_child_frame", "marble_hd2/left_track")
        self.declare_parameter("right_child_frame", "marble_hd2/right_track")
        self.declare_parameter("left_in", "/forest_gen/gz/left_track_odometry_raw")
        self.declare_parameter("right_in", "/forest_gen/gz/right_track_odometry_raw")
        self.declare_parameter("left_out", "/forest_gen/gz/left_track_odometry")
        self.declare_parameter("right_out", "/forest_gen/gz/right_track_odometry")

        self._parent = self.get_parameter("parent_frame").get_parameter_value().string_value
        lc = self.get_parameter("left_child_frame").get_parameter_value().string_value
        rc = self.get_parameter("right_child_frame").get_parameter_value().string_value
        li = self.get_parameter("left_in").get_parameter_value().string_value
        ri = self.get_parameter("right_in").get_parameter_value().string_value
        lo = self.get_parameter("left_out").get_parameter_value().string_value
        ro = self.get_parameter("right_out").get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
        )
        self._pub_l = self.create_publisher(Odometry, lo, qos)
        self._pub_r = self.create_publisher(Odometry, ro, qos)
        self.create_subscription(Odometry, li, lambda m: self._cb(m, lc, self._pub_l), qos)
        self.create_subscription(Odometry, ri, lambda m: self._cb(m, rc, self._pub_r), qos)
        self.get_logger().info(f"{li}→{lo}, {ri}→{ro} (parent={self._parent})")

    @staticmethod
    def _sanitize_orientation(q: Quaternion) -> None:
        """Gazebo por vezes envia quaternion inválido (ex. w=0); o RViz rejeita."""
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
        n = math.sqrt(w * w + x * x + y * y + z * z)
        if n < 1e-6:
            q.w, q.x, q.y, q.z = 1.0, 0.0, 0.0, 0.0
            return
        q.w, q.x, q.y, q.z = w / n, x / n, y / n, z / n

    def _cb(self, msg: Odometry, child: str, pub) -> None:
        out = Odometry()
        # Preserve Gazebo/sim stamp — do not replace with wall clock (breaks EKF + use_sim_time).
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        else:
            out.header.stamp = msg.header.stamp
        fid = (msg.header.frame_id or "").strip()
        cid = (msg.child_frame_id or "").strip()
        out.header.frame_id = self._parent if not fid or fid == "map" else fid
        out.child_frame_id = cid if cid else child
        out.pose = msg.pose
        out.twist = msg.twist
        self._sanitize_orientation(out.pose.pose.orientation)
        pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GzTrackOdometryStamp()
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

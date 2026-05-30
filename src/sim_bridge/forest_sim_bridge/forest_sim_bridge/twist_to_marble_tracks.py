#!/usr/bin/env python3
"""geometry_msgs/Twist → MARBLE_HD2 track_cmd_vel (legado).

Preferir ``TrackedVehicle`` no SDF + bridge ``/forest_gen/cmd_vel`` →
``/model/marble_hd2/cmd_vel`` (publica também ``track_cmd_center_of_rotation``).
Este nó só envia velocidades às esteiras sem centro de rotação — o robô não gira
no Gazebo TrackController.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class TwistToMarbleTracks(Node):
    def __init__(self) -> None:
        super().__init__("twist_to_marble_tracks")
        self.declare_parameter("cmd_vel_topic", "/forest_gen/cmd_vel")
        self.declare_parameter("track_width_m", 0.44)
        self.declare_parameter("track_gain", 2.5)
        self.declare_parameter("max_track_cmd", 1.2)
        self.declare_parameter("left_topic", "/model/marble_hd2/link/left_track/track_cmd_vel")
        self.declare_parameter("right_topic", "/model/marble_hd2/link/right_track/track_cmd_vel")

        ct = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._width = max(0.1, self.get_parameter("track_width_m").get_parameter_value().double_value)
        self._gain = self.get_parameter("track_gain").get_parameter_value().double_value
        self._max_cmd = abs(self.get_parameter("max_track_cmd").get_parameter_value().double_value)
        lt = self.get_parameter("left_topic").get_parameter_value().string_value
        rt = self.get_parameter("right_topic").get_parameter_value().string_value

        self._pub_l = self.create_publisher(Float64, lt, 10)
        self._pub_r = self.create_publisher(Float64, rt, 10)
        self.create_subscription(Twist, ct, self._cb, 10)
        self.get_logger().info(
            f"Sub {ct} → esteiras (skid-steer width={self._width:.2f} m, "
            f"gain={self._gain}, max={self._max_cmd})"
        )

    def _cb(self, msg: Twist) -> None:
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        half = self._width * 0.5
        left = (v - w * half) * self._gain
        right = (v + w * half) * self._gain
        peak = max(abs(left), abs(right), 1e-9)
        if peak > self._max_cmd:
            scale = self._max_cmd / peak
            left *= scale
            right *= scale
        fl = Float64()
        fr = Float64()
        fl.data = left
        fr.data = right
        self._pub_l.publish(fl)
        self._pub_r.publish(fr)


def main() -> None:
    rclpy.init()
    node = TwistToMarbleTracks()
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

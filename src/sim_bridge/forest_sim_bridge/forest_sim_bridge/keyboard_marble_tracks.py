#!/usr/bin/env python3
"""WASD teleop → geometry_msgs/Twist em /forest_gen/cmd_vel.

O Gazebo ``TrackedVehicle`` traduz Twist (m/s, rad/s) para as esteiras e publica
``track_cmd_center_of_rotation`` — necessário para skid-steer no TrackController.
"""

from __future__ import annotations

import fcntl
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardMarbleTracks(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_marble_tracks")
        self.declare_parameter("linear_speed", 0.6)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("hold_sec", 0.22)
        self.declare_parameter("cmd_vel_topic", "/forest_gen/cmd_vel")
        self.declare_parameter("rate_hz", 30.0)

        self._lin_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self._ang_speed = self.get_parameter("angular_speed").get_parameter_value().double_value
        self._hold = max(0.05, self.get_parameter("hold_sec").get_parameter_value().double_value)
        ct = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        hz = max(5.0, self.get_parameter("rate_hz").get_parameter_value().double_value)

        self._pub = self.create_publisher(Twist, ct, 10)
        self._timer = self.create_timer(1.0 / hz, self._tick)

        self._exp_linear = 0.0
        self._linear_dir = 0.0
        self._exp_yaw = 0.0
        self._yaw_dir = 0.0

        self._fd = sys.stdin.fileno()
        self._old_term: list | None = None
        self._old_flags: int | None = None
        if not os.isatty(self._fd):
            raise RuntimeError(
                "stdin is not a terminal (common when launched via ros2 launch). "
                "Options: (1) separate terminal: ros2 run forest_sim_bridge keyboard_marble_tracks ; "
                "(2) use teleop_twist_keyboard: "
                "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args "
                "-r /cmd_vel:=/forest_gen/cmd_vel"
            )
        self._enable_raw()

        self.get_logger().info(
            f"W/S frente-trás | A/D girar | Q ou Ctrl+C sair → {ct} "
            f"(v={self._lin_speed:.2f} m/s, ω={self._ang_speed:.2f} rad/s)"
        )

    def _enable_raw(self) -> None:
        self._old_term = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._old_flags = fcntl.fcntl(self._fd, fcntl.F_GETFL)
        fcntl.fcntl(self._fd, fcntl.F_SETFL, self._old_flags | os.O_NONBLOCK)

    def _restore_tty(self) -> None:
        if self._old_term is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
        if self._old_flags is not None:
            fcntl.fcntl(self._fd, fcntl.F_SETFL, self._old_flags)

    def _poll_keys(self) -> None:
        now = time.monotonic()
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                break
            try:
                ch = sys.stdin.read(1)
            except OSError:
                break
            if not ch:
                break
            c = ch.lower()
            if c in ("\x03", "q"):
                raise KeyboardInterrupt
            if c == "w":
                self._linear_dir, self._exp_linear = 1.0, now + self._hold
            elif c == "s":
                self._linear_dir, self._exp_linear = -1.0, now + self._hold
            elif c == "a":
                self._yaw_dir, self._exp_yaw = -1.0, now + self._hold
            elif c == "d":
                self._yaw_dir, self._exp_yaw = 1.0, now + self._hold
            elif ch in ("\x1b",):
                self._linear_dir = self._yaw_dir = 0.0
                self._exp_linear = self._exp_yaw = 0.0

    def _tick(self) -> None:
        self._poll_keys()
        now = time.monotonic()
        lin = self._linear_dir if now < self._exp_linear else 0.0
        yaw = self._yaw_dir if now < self._exp_yaw else 0.0

        msg = Twist()
        msg.linear.x = float(self._lin_speed * lin)
        msg.angular.z = float(self._ang_speed * yaw)
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        self._restore_tty()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: KeyboardMarbleTracks | None = None
    try:
        node = KeyboardMarbleTracks()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"keyboard_marble_tracks: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node._restore_tty()
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

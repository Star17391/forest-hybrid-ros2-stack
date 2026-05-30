#!/usr/bin/env python3
"""Painel Tk: teleop por botões → geometry_msgs/Twist em /forest_gen/cmd_vel."""

from __future__ import annotations

import math
from typing import Dict, Literal, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

Mode = Literal["hold", "toggle"]

_MOVE_DIRS: Dict[str, Tuple[float, float, float]] = {
    "up": (1.0, 0.0, 0.0),
    "down": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
}


def _norm_xy(lx: float, ly: float) -> Tuple[float, float, float]:
    s = math.hypot(lx, ly)
    return (lx / s, ly / s, 0.0) if s > 1e-9 else (0.0, 0.0, 0.0)


_MOVE_DIRS["up_left"] = _norm_xy(1.0, 1.0)
_MOVE_DIRS["up_right"] = _norm_xy(1.0, -1.0)
_MOVE_DIRS["down_left"] = _norm_xy(-1.0, 1.0)
_MOVE_DIRS["down_right"] = _norm_xy(-1.0, -1.0)

_ROT_DIRS: Dict[str, Tuple[float, float, float]] = {
    "rot_ccw": (0.0, 0.0, 1.0),
    "rot_cw": (0.0, 0.0, -1.0),
}

# Clique contrário desliga o sentido oposto neste eixo
_OPPOSITE: Dict[str, str] = {
    "up": "down",
    "down": "up",
    "left": "right",
    "right": "left",
    "up_left": "down_right",
    "down_right": "up_left",
    "up_right": "down_left",
    "down_left": "up_right",
    "rot_ccw": "rot_cw",
    "rot_cw": "rot_ccw",
}


class TeleopPanelNode(Node):
    def __init__(self) -> None:
        super().__init__("forest_teleop_panel")
        self.declare_parameter("cmd_vel_topic", "/forest_gen/cmd_vel")
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("lateral_speed", 0.35)
        self.declare_parameter("angular_speed", 0.7)
        self.declare_parameter("rate_hz", 25.0)

        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.linear_speed = (
            self.get_parameter("linear_speed").get_parameter_value().double_value
        )
        self.lateral_speed = (
            self.get_parameter("lateral_speed").get_parameter_value().double_value
        )
        self.angular_speed = (
            self.get_parameter("angular_speed").get_parameter_value().double_value
        )
        hz = max(5.0, self.get_parameter("rate_hz").get_parameter_value().double_value)

        self.mode: Mode = "hold"
        self._active: set[str] = set()
        self._pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._timer = self.create_timer(1.0 / hz, self._publish_cmd)
        self.get_logger().info(f"Teleop → {self.cmd_vel_topic}")

    def set_mode(self, mode: Mode) -> None:
        self.mode = mode
        self.stop_all()

    def set_speeds(self, linear: float, lateral: float, angular: float) -> None:
        self.linear_speed = max(0.0, float(linear))
        self.lateral_speed = max(0.0, float(lateral))
        self.angular_speed = max(0.0, float(angular))

    def stop_all(self) -> None:
        self._active.clear()
        self._publish_zero()

    def _activate(self, key: str) -> None:
        opp = _OPPOSITE.get(key)
        if opp and opp in self._active:
            self._active.discard(opp)
        self._active.add(key)

    def _deactivate(self, key: str) -> None:
        self._active.discard(key)

    def on_press(self, key: str) -> None:
        if self.mode == "hold":
            self._activate(key)

    def on_release(self, key: str) -> None:
        if self.mode == "hold":
            self._deactivate(key)

    def on_leave(self, key: str) -> None:
        if self.mode == "hold":
            self._deactivate(key)

    def on_click(self, key: str) -> None:
        if self.mode != "toggle":
            return
        if key in self._active:
            self._deactivate(key)
        else:
            self._activate(key)

    def _publish_zero(self) -> None:
        self._pub.publish(Twist())

    def _publish_cmd(self) -> None:
        if not self._active:
            self._publish_zero()
            return

        lx = ly = az = 0.0
        for key in self._active:
            if key in _MOVE_DIRS:
                dx, dy, dw = _MOVE_DIRS[key]
                lx += dx
                ly += dy
                az += dw
            elif key in _ROT_DIRS:
                dx, dy, dw = _ROT_DIRS[key]
                lx += dx
                ly += dy
                az += dw

        trans = math.hypot(lx, ly)
        if trans > 1.0:
            lx /= trans
            ly /= trans
        rot = max(-1.0, min(1.0, az))

        msg = Twist()
        msg.linear.x = float(self.linear_speed * lx)
        msg.linear.y = float(self.lateral_speed * ly)
        msg.angular.z = float(self.angular_speed * rot)
        self._pub.publish(msg)

    @property
    def status_line(self) -> str:
        active = ", ".join(sorted(self._active)) if self._active else "(parado)"
        return (
            f"modo={self.mode} | v={self.linear_speed:.2f} lat={self.lateral_speed:.2f} "
            f"ω={self.angular_speed:.2f} | {active}"
        )


def main() -> None:
    rclpy.init()
    node = TeleopPanelNode()

    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("Forest — teleop (cmd_vel)")

    frm = ttk.Frame(root, padding=10)
    frm.grid(row=0, column=0)

    mode_var = tk.StringVar(value="hold")
    speed_lin = tk.DoubleVar(value=node.linear_speed)
    speed_lat = tk.DoubleVar(value=node.lateral_speed)
    speed_ang = tk.DoubleVar(value=node.angular_speed)
    status_var = tk.StringVar(value=node.status_line)

    def refresh_status() -> None:
        status_var.set(node.status_line)

    def on_mode_change() -> None:
        node.set_mode("toggle" if mode_var.get() == "toggle" else "hold")
        refresh_status()

    def on_speed_change(*_args) -> None:
        node.set_speeds(speed_lin.get(), speed_lat.get(), speed_ang.get())
        refresh_status()

    mode_frm = ttk.LabelFrame(frm, text="Modo de controlo", padding=6)
    mode_frm.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))
    ttk.Radiobutton(
        mode_frm,
        text="1 — Manter premido para andar",
        variable=mode_var,
        value="hold",
        command=on_mode_change,
    ).grid(row=0, column=0, sticky="w")
    ttk.Radiobutton(
        mode_frm,
        text="2 — Clique: anda até clique no sentido contrário",
        variable=mode_var,
        value="toggle",
        command=on_mode_change,
    ).grid(row=1, column=0, sticky="w")

    spd_frm = ttk.LabelFrame(frm, text="Velocidades (m/s, rad/s)", padding=6)
    spd_frm.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 8))
    for col, (label, var) in enumerate(
        (
            ("Frente/trás (linear.x)", speed_lin),
            ("Lateral (linear.y)", speed_lat),
            ("Rotação (angular.z)", speed_ang),
        )
    ):
        ttk.Label(spd_frm, text=label).grid(row=0, column=col, padx=4, sticky="w")
        sp = ttk.Spinbox(
            spd_frm,
            from_=0.0,
            to=3.0,
            increment=0.05,
            textvariable=var,
            width=8,
            command=on_speed_change,
        )
        sp.grid(row=1, column=col, padx=4, pady=2)
        sp.bind("<Return>", on_speed_change)
        sp.bind("<FocusOut>", on_speed_change)

    pad = ttk.Frame(frm)
    pad.grid(row=2, column=0, columnspan=3)

    def bind_direction(btn: tk.Button, key: str) -> None:
        def press(_e=None) -> None:
            node.on_press(key)
            refresh_status()

        def release(_e=None) -> None:
            node.on_release(key)
            refresh_status()

        def leave(_e=None) -> None:
            node.on_leave(key)
            refresh_status()

        def click(_e=None) -> None:
            node.on_click(key)
            refresh_status()

        btn.bind("<ButtonPress-1>", press)
        btn.bind("<ButtonRelease-1>", release)
        btn.bind("<Leave>", leave)
        btn.bind("<Button-1>", click, add="+")

    def mk_btn(parent, text: str, key: str, row: int, col: int, *, width=6) -> tk.Button:
        b = tk.Button(parent, text=text, width=width, height=2)
        b.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
        bind_direction(b, key)
        return b

    mk_btn(pad, "↖", "up_left", 0, 0)
    mk_btn(pad, "↑", "up", 0, 1)
    mk_btn(pad, "↗", "up_right", 0, 2)
    mk_btn(pad, "←", "left", 1, 0)
    stop_btn = tk.Button(pad, text="■", width=6, height=2, command=lambda: (node.stop_all(), refresh_status()))
    stop_btn.grid(row=1, column=1, padx=3, pady=3)
    mk_btn(pad, "→", "right", 1, 2)
    mk_btn(pad, "↙", "down_left", 2, 0)
    mk_btn(pad, "↓", "down", 2, 1)
    mk_btn(pad, "↘", "down_right", 2, 2)

    rot_frm = ttk.Frame(frm)
    rot_frm.grid(row=3, column=0, columnspan=3, pady=(12, 4))
    ttk.Label(rot_frm, text="Girar no lugar:").grid(row=0, column=0, columnspan=2, pady=(0, 6))
    mk_btn(rot_frm, "↺  CCW", "rot_ccw", 1, 0, width=12)
    mk_btn(rot_frm, "CW  ↻", "rot_cw", 1, 1, width=12)

    ttk.Label(frm, textvariable=status_var, wraplength=420).grid(
        row=4, column=0, columnspan=3, pady=6
    )
    ttk.Label(
        frm,
        text="RViz: usa seta verde (/state/pose_fused). Se girar mais rápido que o Gazebo, "
        "confirma RViz → Painel Time → Sync = /clock.",
        font=("", 8),
        wraplength=420,
    ).grid(row=5, column=0, columnspan=3)

    def ros_spin_tick() -> None:
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
        refresh_status()
        root.after(40, ros_spin_tick)

    ros_spin_tick()

    def on_close() -> None:
        node.stop_all()
        root.quit()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        try:
            root.destroy()
        except Exception:
            pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

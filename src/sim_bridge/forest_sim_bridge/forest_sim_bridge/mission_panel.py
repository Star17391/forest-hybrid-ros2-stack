#!/usr/bin/env python3
"""Painel Tk: GOTO, PATROL multi-waypoint, HOLD, E-STOP → /mission/command.

Waypoints are planar (map x, y only). Z is always 0 — height comes from Gazebo physics.
"""

from __future__ import annotations

import uuid
from typing import List, Tuple

import rclpy
from rclpy.node import Node

# Planar missions: altitude is not commanded (terrain following via sim physics).
_PLANAR_Z = 0.0


def _load_mission_msg():
    try:
        from forest_hybrid_msgs.msg import MissionCommand, MissionStatus  # type: ignore

        return MissionCommand, MissionStatus
    except ImportError as exc:
        raise SystemExit(
            "forest_hybrid_msgs não encontrado. Faz source do workspace forest-hybrid-ros2-stack.\n"
            f"Detalhe: {exc}"
        ) from exc


class MissionPanelNode(Node):
    def __init__(self, MissionCommand, MissionStatus) -> None:
        super().__init__("forest_gen_mission_panel")
        self._C = MissionCommand
        self._S = MissionStatus
        self._pub = self.create_publisher(MissionCommand, "/mission/command", 10)
        self._status = "—"
        self.create_subscription(MissionStatus, "/mission/status", self._on_status, 10)
        self.get_logger().info("Painel → /mission/command (planar XY) | status ← /mission/status")

    def _on_status(self, msg) -> None:
        self._status = f"state={msg.state} progress={msg.progress:.0f}% — {msg.detail}"

    @property
    def status_text(self) -> str:
        return self._status

    def send_goto(self, x: float, y: float, use_yaw: bool, yaw_rad: float) -> None:
        msg = self._C()
        msg.command_type = self._C.CMD_GOTO_XYZ
        msg.frame_type = self._C.FRAME_MAP
        msg.command_id = str(uuid.uuid4())
        msg.source = "forest_gen_mission_panel"
        msg.target_x = float(x)
        msg.target_y = float(y)
        msg.target_z = _PLANAR_Z
        msg.use_target_yaw = bool(use_yaw)
        msg.target_yaw_rad = float(yaw_rad)
        self._pub.publish(msg)
        self.get_logger().info(f"GOTO ({x:.2f}, {y:.2f}) [planar]")

    def send_patrol(self, waypoints: List[Tuple[float, float]]) -> None:
        if len(waypoints) < 2:
            self.get_logger().warn("PATROL requer pelo menos 2 waypoints")
            return
        msg = self._C()
        msg.command_type = self._C.CMD_PATROL_WAYPOINTS
        msg.frame_type = self._C.FRAME_MAP
        msg.command_id = str(uuid.uuid4())
        msg.source = "forest_gen_mission_panel"
        msg.waypoint_x = [w[0] for w in waypoints]
        msg.waypoint_y = [w[1] for w in waypoints]
        msg.waypoint_z = [_PLANAR_Z] * len(waypoints)
        self._pub.publish(msg)
        self.get_logger().info(f"PATROL com {len(waypoints)} waypoints (planar)")

    def send_hold(self) -> None:
        msg = self._C()
        msg.command_type = self._C.CMD_HOLD
        msg.frame_type = self._C.FRAME_MAP
        msg.command_id = str(uuid.uuid4())
        msg.source = "forest_gen_mission_panel"
        self._pub.publish(msg)

    def send_estop(self) -> None:
        msg = self._C()
        msg.command_type = self._C.CMD_EMERGENCY_STOP
        msg.frame_type = self._C.FRAME_MAP
        msg.command_id = str(uuid.uuid4())
        msg.source = "forest_gen_mission_panel"
        self._pub.publish(msg)


def main() -> None:
    MissionCommand, MissionStatus = _load_mission_msg()
    rclpy.init()
    node = MissionPanelNode(MissionCommand, MissionStatus)

    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("Forest — missão / waypoints (map XY)")

    fx = tk.DoubleVar(value=3.0)
    fy = tk.DoubleVar(value=0.0)
    f_yaw = tk.BooleanVar(value=False)
    f_yaw_v = tk.DoubleVar(value=0.0)
    waypoints: List[Tuple[float, float]] = []

    frm = ttk.Frame(root, padding=8)
    frm.grid(row=0, column=0, sticky="nsew")

    ttk.Label(frm, text="Waypoint (map — só X/Y; altura pelo terreno)").grid(
        row=0, column=0, columnspan=3, pady=4
    )
    ttk.Label(frm, text="x").grid(row=1, column=0, sticky="e")
    ttk.Entry(frm, textvariable=fx, width=10).grid(row=1, column=1, sticky="w")
    ttk.Label(frm, text="y").grid(row=2, column=0, sticky="e")
    ttk.Entry(frm, textvariable=fy, width=10).grid(row=2, column=1, sticky="w")

    lb = tk.Listbox(frm, height=8, width=42)
    lb.grid(row=3, column=0, columnspan=3, pady=6)

    def refresh_list() -> None:
        lb.delete(0, tk.END)
        for i, (x, y) in enumerate(waypoints):
            lb.insert(tk.END, f"{i + 1}: ({x:.2f}, {y:.2f})")

    def add_wp() -> None:
        waypoints.append((fx.get(), fy.get()))
        refresh_list()

    def remove_wp() -> None:
        sel = lb.curselection()
        if sel:
            waypoints.pop(int(sel[0]))
            refresh_list()

    def clear_wp() -> None:
        waypoints.clear()
        refresh_list()

    ttk.Button(frm, text="Adicionar WP", command=add_wp).grid(row=4, column=0, pady=2, sticky="ew")
    ttk.Button(frm, text="Remover", command=remove_wp).grid(row=4, column=1, pady=2, sticky="ew")
    ttk.Button(frm, text="Limpar", command=clear_wp).grid(row=4, column=2, pady=2, sticky="ew")

    ttk.Button(frm, text="Enviar PATROL (lista)", command=lambda: node.send_patrol(waypoints)).grid(
        row=5, column=0, columnspan=3, pady=6, sticky="ew"
    )

    ttk.Separator(frm, orient="horizontal").grid(row=6, column=0, columnspan=3, sticky="ew", pady=4)
    ttk.Checkbutton(frm, text="use_target_yaw (GOTO)", variable=f_yaw).grid(row=7, column=0, sticky="e")
    ttk.Entry(frm, textvariable=f_yaw_v, width=10).grid(row=7, column=1, sticky="w")

    def on_goto() -> None:
        node.send_goto(fx.get(), fy.get(), f_yaw.get(), f_yaw_v.get())

    ttk.Button(frm, text="GOTO (1 ponto)", command=on_goto).grid(row=8, column=0, pady=4, sticky="ew")
    ttk.Button(frm, text="HOLD", command=node.send_hold).grid(row=8, column=1, pady=4, sticky="ew")
    ttk.Button(frm, text="E-STOP", command=node.send_estop).grid(row=8, column=2, pady=4, sticky="ew")

    status_lbl = ttk.Label(frm, text="—", wraplength=380)
    status_lbl.grid(row=9, column=0, columnspan=3, pady=8)

    def ros_spin_tick() -> None:
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
        status_lbl.config(text=node.status_text)
        root.after(50, ros_spin_tick)

    ros_spin_tick()

    def on_close() -> None:
        root.quit()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
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

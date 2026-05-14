#!/usr/bin/env python3
"""
Drive mission_manager smoke checks via `ros2 topic` subprocess calls (no rclpy).

Run after: source /opt/ros/jazzy/setup.bash && source install/setup.bash

Arrival uses /state/pose_fused (map frame) + metric distance and heading tolerance,
not /planning/progress.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def run(args: list[str], *, timeout: float) -> str:
    p = subprocess.run(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=False,
    )
    out = p.stdout or ""
    if p.returncode != 0:
        raise RuntimeError(f"cmd failed rc={p.returncode}: {' '.join(args)}\n{out}")
    return out


def pub_pose_map(x: float, y: float, z: float) -> None:
    yaml_pose = (
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, "
        "pose: {position: {x: "
        + str(x)
        + ", y: "
        + str(y)
        + ", z: "
        + str(z)
        + "}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    )
    run(
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/state/pose_fused",
            "geometry_msgs/msg/PoseStamped",
            yaml_pose,
        ],
        timeout=25.0,
    )


def main() -> int:
    ws = Path(__file__).resolve().parents[1]
    mgr = ["ros2", "run", "forest_planner_ros2", "mission_manager_node"]

    env = os.environ.copy()
    subprocess.run(["pkill", "-f", "mission_manager_node"], cwd=ws, capture_output=True)
    time.sleep(0.4)

    proc = subprocess.Popen(
        mgr,
        cwd=ws,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL,
    )
    try:
        time.sleep(2.8)
        if proc.poll() is not None:
            print("mission_manager exited early (missing msgs / bad install?)", file=sys.stderr)
            return 4

        run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)

        run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/mission/command",
                "forest_hybrid_msgs/msg/MissionCommand",
                (
                    "{command_type: 1, frame_type: 0, command_id: 'goto_cli', "
                    "source: 'py_smoke', use_target_yaw: false, target_yaw_rad: 0.0, "
                    "target_x: 1.0, target_y: -2.0, target_z: 0.25, "
                    "waypoint_x: [], waypoint_y: [], waypoint_z: [], waypoint_yaw: []}"
                ),
            ],
            timeout=25.0,
        )
        time.sleep(0.4)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 1" not in st:
            print("Expected EXECUTING after GOTO;\n", st, file=sys.stderr)
            return 5

        run(["ros2", "topic", "echo", "/planning/mission_goal", "--once"], timeout=15.0)

        pub_pose_map(1.0, -2.0, 0.25)
        time.sleep(0.5)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 5" not in st:
            print("Expected COMPLETED after pose at goal;\n", st, file=sys.stderr)
            return 6

        run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/mission/command",
                "forest_hybrid_msgs/msg/MissionCommand",
                (
                    "{command_type: 4, frame_type: 0, command_id: 'home_cli', "
                    "source: 'py_smoke', use_target_yaw: false, target_yaw_rad: 0.0, "
                    "target_x: 10.0, target_y: 11.0, target_z: 12.0, "
                    "waypoint_x: [], waypoint_y: [], waypoint_z: [], waypoint_yaw: []}"
                ),
            ],
            timeout=25.0,
        )
        time.sleep(0.4)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 2" not in st:
            print("Expected WAITING_ACK after RETURN_HOME;\n", st, file=sys.stderr)
            return 7

        run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/mission/ack",
                "forest_hybrid_msgs/msg/MissionAck",
                "{command_id: 'home_cli', approved: true, reason: ''}",
            ],
            timeout=25.0,
        )
        time.sleep(0.4)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 1" not in st:
            print("Expected EXECUTING after ACK;\n", st, file=sys.stderr)
            return 8

        pub_pose_map(10.0, 11.0, 12.0)
        time.sleep(0.5)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 5" not in st:
            print("Expected COMPLETED after RETURN_HOME pose;\n", st, file=sys.stderr)
            return 9

        run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/mission/command",
                "forest_hybrid_msgs/msg/MissionCommand",
                (
                    "{command_type: 5, frame_type: 0, command_id: 'stop_cli', "
                    "source: 'py_smoke', use_target_yaw: false, target_yaw_rad: 0.0, "
                    "target_x: 0.0, target_y: 0.0, target_z: 0.0, "
                    "waypoint_x: [], waypoint_y: [], waypoint_z: [], waypoint_yaw: []}"
                ),
            ],
            timeout=25.0,
        )
        time.sleep(0.4)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 4" not in st:
            print("Expected EMERGENCY after ESTOP;\n", st, file=sys.stderr)
            return 10

        run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/mission/command",
                "forest_hybrid_msgs/msg/MissionCommand",
                (
                    "{command_type: 1, frame_type: 0, command_id: 'goto_blocked', "
                    "source: 'py_smoke', use_target_yaw: false, target_yaw_rad: 0.0, "
                    "target_x: 99.0, target_y: 99.0, target_z: 0.0, "
                    "waypoint_x: [], waypoint_y: [], waypoint_z: [], waypoint_yaw: []}"
                ),
            ],
            timeout=25.0,
        )
        time.sleep(0.35)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 6" not in st:
            print("Expected FAILED while emergency latched;\n", st, file=sys.stderr)
            return 13

        run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/mission/command",
                "forest_hybrid_msgs/msg/MissionCommand",
                (
                    "{command_type: 6, frame_type: 0, command_id: 'clear_em', "
                    "source: 'py_smoke', use_target_yaw: false, target_yaw_rad: 0.0, "
                    "target_x: 0.0, target_y: 0.0, target_z: 0.0, "
                    "waypoint_x: [], waypoint_y: [], waypoint_z: [], waypoint_yaw: []}"
                ),
            ],
            timeout=25.0,
        )
        time.sleep(0.35)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 0" not in st:
            print("Expected IDLE after CLEAR_EMERGENCY_LATCH;\n", st, file=sys.stderr)
            return 14

        run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/mission/command",
                "forest_hybrid_msgs/msg/MissionCommand",
                (
                    "{command_type: 1, frame_type: 0, command_id: 'goto_after_clear', "
                    "source: 'py_smoke', use_target_yaw: false, target_yaw_rad: 0.0, "
                    "target_x: 0.5, target_y: 0.5, target_z: 0.0, "
                    "waypoint_x: [], waypoint_y: [], waypoint_z: [], waypoint_yaw: []}"
                ),
            ],
            timeout=25.0,
        )
        time.sleep(0.4)
        pub_pose_map(0.5, 0.5, 0.0)
        time.sleep(0.5)
        st = run(["ros2", "topic", "echo", "/mission/status", "--once"], timeout=20.0)
        if "state: 5" not in st:
            print("Expected COMPLETED after latch cleared + GOTO pose;\n", st, file=sys.stderr)
            return 15

        print("MISSION_SMOKE (subprocess): OK")
        return 0
    except subprocess.TimeoutExpired as exc:
        print(f"Timeout waiting for ROS graph / messages: {exc}", file=sys.stderr)
        return 11
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 12
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=6.0)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())

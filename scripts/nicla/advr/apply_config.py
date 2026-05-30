#!/usr/bin/env python3
"""Generate ADVR config.h and forest nicla_advr_receiver.yaml from config/forest_nicla_advr_config.h."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "config" / "forest_nicla_advr_config.h"
ADVR_CONFIG_H = ROOT / "third_party" / "nicla_vision_drivers" / "arduino" / "main" / "config.h"
ROS_YAML = (
    ROOT
    / "src"
    / "drivers_stack"
    / "forest_nicla_vision_ros2"
    / "config"
    / "nicla_advr_receiver.yaml"
)


def parse_defines(path: Path) -> dict[str, str]:
    text = path.read_text(encoding="utf-8")
    out: dict[str, str] = {}
    for line in text.splitlines():
        m = re.match(r'^\s*#\s*define\s+(\w+)\s+(.+?)\s*$', line)
        if not m:
            continue
        name, value = m.group(1), m.group(2).strip()
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
        out[name] = value
    return out


def as_bool(val: str) -> bool:
    return val.strip().lower() in ("1", "true", "yes", "on")


def ip_from_defines(cfg: dict[str, str]) -> str:
    raw = cfg.get("FOREST_NICLA_PC_IP", "192, 168, 1, 10")
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != 4:
        raise ValueError(f"FOREST_NICLA_PC_IP must have 4 octets: {raw!r}")
    return ".".join(parts)


def local_ipv4_addresses() -> set[str]:
    import socket

    addrs: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addrs.add(info[4][0])
    except OSError:
        pass
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            addrs.add(s.getsockname()[0])
    except OSError:
        pass
    return {a for a in addrs if not a.startswith("127.")}


def warn_pc_ip(receiver_ip: str) -> None:
    if receiver_ip in local_ipv4_addresses():
        return
    local = ", ".join(sorted(local_ipv4_addresses())) or "(none detected)"
    print(
        f"WARNING: FOREST_NICLA_PC_IP is {receiver_ip} but this host has: {local}\n"
        "  Nicla firmware will not reach ROS until the IP matches your PC on Wi-Fi.\n"
        "  Edit config/forest_nicla_advr_config.h, then re-run apply + upload firmware.",
        file=sys.stderr,
    )


def write_advr_config_h(cfg: dict[str, str]) -> None:
    net = cfg.get("FOREST_NICLA_NETWORK_TYPE", "tcp").strip().lower()
    network_type = "_TCP_" if net == "tcp" else "_UDP_"
    ip_octets = cfg.get("FOREST_NICLA_PC_IP", "192, 168, 1, 10")
    cam_fps = cfg.get("FOREST_NICLA_CAM_FPS", "60")

    content = f"""#ifndef CONFIG_H
#define CONFIG_H
#define _UDP_ 0
#define _TCP_ 1

// Generated from config/forest_nicla_advr_config.h — do not edit by hand.
#define _IP_ {ip_octets}
#define NETWORK_SSID "{cfg.get("FOREST_NICLA_WIFI_SSID", "")}"
#define NETWORK_KEY "{cfg.get("FOREST_NICLA_WIFI_PASSWORD", "")}"
#define NETWORK_TYPE {network_type}
#define ENABLE_ARDUINO_IDE_SERIAL_MONITOR false
#define ENABLE_VERBOSE false
#define ENABLE_VERBOSE_TIME false

#define USE_CAM {1 if as_bool(cfg.get("FOREST_NICLA_USE_CAMERA", "1")) else 0}
#define USE_MIC {1 if as_bool(cfg.get("FOREST_NICLA_USE_MIC", "0")) else 0}
#define USE_IMU {1 if as_bool(cfg.get("FOREST_NICLA_USE_IMU", "1")) else 0}
#define USE_TOF {1 if as_bool(cfg.get("FOREST_NICLA_USE_TOF", "0")) else 0}

#define CAM_FPS {cam_fps}

#endif // CONFIG_H
"""
    ADVR_CONFIG_H.parent.mkdir(parents=True, exist_ok=True)
    ADVR_CONFIG_H.write_text(content, encoding="utf-8")
    print(f"Wrote {ADVR_CONFIG_H}")


def write_ros_yaml(cfg: dict[str, str]) -> None:
    receiver_ip = ip_from_defines(cfg)
    port = cfg.get("FOREST_NICLA_RECEIVER_PORT", "8002")
    conn = cfg.get("FOREST_NICLA_CONNECTION_TYPE", "tcp").strip().lower()
    nicla_name = cfg.get("FOREST_NICLA_ROS_NAME", "nicla")
    rate = cfg.get("FOREST_NICLA_PUBLISH_RATE_HZ", "500")
    recv_compressed = as_bool(cfg.get("FOREST_NICLA_CAMERA_RECEIVE_COMPRESSED", "1"))

    yaml = f"""# Auto-generated from config/forest_nicla_advr_config.h — do not edit by hand.
# nicla_receiver publish loop rate in upstream: {rate} Hz
nicla_receiver:
  ros__parameters:
    nicla_name: "{nicla_name}"
    receiver_ip: "{receiver_ip}"
    receiver_port: "{port}"
    connection_type: "{conn}"
    enable_range: {str(as_bool(cfg.get("FOREST_NICLA_ENABLE_TOF", "0"))).lower()}
    enable_camera_raw: {str(as_bool(cfg.get("FOREST_NICLA_ENABLE_CAMERA_RAW", "1"))).lower()}
    enable_camera_compressed: {str(as_bool(cfg.get("FOREST_NICLA_ENABLE_CAMERA_COMPRESSED", "0"))).lower()}
    camera_receive_compressed: {str(recv_compressed).lower()}
    camera_pixel_format: "rgb565"
    camera_width: 320
    camera_height: 240
    camera_img_rotate_code: 0
    enable_audio: {str(as_bool(cfg.get("FOREST_NICLA_ENABLE_AUDIO", "0"))).lower()}
    enable_audio_stamped: {str(as_bool(cfg.get("FOREST_NICLA_ENABLE_AUDIO_STAMPED", "0"))).lower()}
    enable_audio_recognition_vosk: false
    enable_imu: {str(as_bool(cfg.get("FOREST_NICLA_ENABLE_IMU", "1"))).lower()}
"""
    ROS_YAML.parent.mkdir(parents=True, exist_ok=True)
    ROS_YAML.write_text(yaml, encoding="utf-8")
    print(f"Wrote {ROS_YAML}")
    print(f"  receiver_ip={receiver_ip} port={port} connection={conn} jpeg={recv_compressed}")
    warn_pc_ip(receiver_ip)


def main() -> int:
    if not HEADER.is_file():
        print(f"Missing {HEADER}", file=sys.stderr)
        return 1
    drivers = ROOT / "third_party" / "nicla_vision_drivers"
    if not (drivers / "arduino" / "main" / "main.ino").is_file():
        print(
            f"Missing {drivers}/arduino/main/main.ino\n"
            "Run: bash scripts/nicla/advr/init_submodules.sh",
            file=sys.stderr,
        )
        return 1
    cfg = parse_defines(HEADER)
    write_advr_config_h(cfg)
    write_ros_yaml(cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

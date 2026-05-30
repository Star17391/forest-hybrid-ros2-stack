#!/usr/bin/env bash
# Connect Nicla to Wi-Fi after USB works (WIFI_CONNECT over serial).
set -eo pipefail

PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"

python3 <<PY
import sys
import time

import serial

port = "${PORT}"
ser = serial.Serial(port, 921600, timeout=0.5)
time.sleep(1.5)

def talk(cmd: str, wait: float = 0.3) -> list[str]:
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\n").encode())
    ser.flush()
    time.sleep(wait)
    lines = []
    deadline = time.time() + 35.0
    while time.time() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            lines.append(line)
            print(f"  << {line}")
        if lines and lines[-1].startswith(("OK", "ERR", "INFO")):
            if cmd.startswith("WIFI") and "wifi_up" in line:
                break
            if cmd == "WIFI_STATUS" or cmd == "PING":
                break
            if "wifi_connect_scheduled" in line:
                break
            if "wifi_timeout" in line or "wifi=connected" in line or "wifi=disconnected" in line:
                break
    return lines

print(f"Port {port}")
talk("PING")
talk("WIFI_CONNECT")
print("Waiting for association (up to 30 s)...")
for _ in range(40):
    lines = talk("WIFI_STATUS", wait=0.5)
    if any("wifi=connected" in l for l in lines):
        print("Wi-Fi OK")
        sys.exit(0)
    if any("wifi_timeout" in l for l in lines):
        print("Wi-Fi timeout — check hotspot 2.4 GHz, antenna, password", file=sys.stderr)
        sys.exit(2)
    time.sleep(1.0)

print("No association yet — check phone hotspot settings", file=sys.stderr)
sys.exit(3)
PY

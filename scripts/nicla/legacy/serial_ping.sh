#!/usr/bin/env bash
# Ping Nicla over USB without reopening the port (avoids reset on each '>' redirect).
set -eo pipefail

PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"

python3 <<PY
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Install pyserial: sudo apt install python3-serial")

port = "${PORT}"
ser = serial.Serial(port, 921600, timeout=0.5)
time.sleep(2.0)
print(f"Opened {port}")

for _ in range(20):
    line = ser.readline().decode("ascii", errors="replace").strip()
    if line:
        print(f"  << {line}")
    if "READY" in line:
        break
else:
    print("(no READY seen yet — continuing)")

ser.write(b"PING\n")
ser.flush()
deadline = time.time() + 3.0
while time.time() < deadline:
    line = ser.readline().decode("ascii", errors="replace").strip()
    if line:
        print(f"  << {line}")
        if line == "PONG":
            print("OK")
            ser.close()
            raise SystemExit(0)

print("FAIL: no PONG", file=sys.stderr)
ser.close()
sys.exit(1)
PY

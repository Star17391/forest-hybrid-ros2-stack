#!/usr/bin/env bash
# One-time: flash Cypress/WICED Wi-Fi firmware to Nicla Vision QSPI (required before WiFi.begin).
set -eo pipefail

PORT="${NICLA_SERIAL_PORT:-/dev/ttyACM0}"
FQBN=arduino:mbed_nicla:nicla_vision
SKETCH="${HOME}/.arduino15/packages/arduino/hardware/mbed_nicla/4.5.0/libraries/STM32H747_System/examples/WiFiFirmwareUpdater"

if [[ ! -d "$SKETCH" ]]; then
  echo "WiFiFirmwareUpdater not found. Run: arduino-cli core install arduino:mbed_nicla" >&2
  exit 1
fi

echo "== Nicla Vision: install Wi-Fi coprocessor firmware (one-time) =="
echo "Port: $PORT"
echo ""
echo "This uploads Arduino's WiFiFirmwareUpdater sketch (115200 baud on serial)."
echo "It formats QSPI and writes 4343WA1.BIN — required before hotspot/router works."
echo ""

arduino-cli compile -b "$FQBN" "$SKETCH"
arduino-cli upload -b "$FQBN" -p "$PORT" "$SKETCH"

echo ""
echo "== Open serial monitor at 115200 baud and wait for: =="
echo "   Firmware and certificates updated!"
echo ""
echo "Example:"
echo "  python3 -c \"import serial,time; s=serial.Serial('$PORT',115200);"
echo "  time.sleep(1);"
echo "  [print(s.readline().decode(errors='replace'),end='') for _ in range(200) if s.in_waiting or time.sleep(0.2) is None]\""
echo ""
echo "If it asks to reinstall anyway, send: y"
echo ""
echo "Then re-flash your sensor firmware:"
echo "  SKETCH=~/Projetos/Tese/forest-hybrid-ros2-stack/src/drivers_stack/forest_nicla_vision_ros2/firmware/nicla_sensor_node"
echo "  arduino-cli upload -b $FQBN -p $PORT \"\$SKETCH\""
echo "  bash scripts/nicla/legacy/wifi_connect.sh"

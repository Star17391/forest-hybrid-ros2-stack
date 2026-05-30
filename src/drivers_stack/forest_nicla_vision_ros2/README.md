# `forest_nicla_vision_ros2`

Driver for the **Arduino Nicla Vision** (no Linux `/dev/video0`).

## Primary path: ADVR streaming stack (recommended)

Uses [ADVRHumanoids](https://github.com/ADVRHumanoids) firmware + ROS 2 (submodules) with forest topic remaps.

**Configure once:** [`../../../config/forest_nicla_advr_config.h`](../../../config/forest_nicla_advr_config.h)  
**Full guide:** [`../../../docs/NICLA_ADVR_SETUP.md`](../../../docs/NICLA_ADVR_SETUP.md)

```bash
bash scripts/nicla_advr_init_submodules.sh
# edit config/forest_nicla_advr_config.h
bash scripts/nicla_advr_apply_config.sh
bash scripts/nicla_advr_upload_firmware.sh
bash scripts/nicla_advr_build.sh
source install/setup.bash
ros2 launch forest_nicla_vision_ros2 nicla_vision_advr.launch.py
```

## Legacy path: NICLAv1 USB / Wi‑Fi (`nicla_sensor_node.ino`)

Custom firmware under `firmware/nicla_sensor_node/` — SNAP protocol, simpler but lower FPS.

## Phase 1 pipeline (done)

QQVGA 160×120, PING/SNAP, RGB565 big-endian decode.

## Transport strategy (USB vs Wi‑Fi)

**Recommended primary path:** **USB serial + JPEG on the Nicla** (Phase 4), decode on the Pi.

| Approach | Reliability (robotics) | Throughput | Notes |
|----------|------------------------|----------|--------|
| **USB CDC (current)** | Excellent — physical link, no association drops | ~0.75 FPS RGB565 QVGA; **3–8+ FPS** with JPEG | Best default when RPi and Nicla are **cm apart** |
| **Wi‑Fi** | Good but **not guaranteed** — interference, sleep, DHCP | High (Mbps) | Use as **optional** telemetry or second client, not sole safety path |
| **USB 3.0 port on Pi** | Does not help if Nicla is USB **2.0** device | Same as USB2 CDC | Cable quality matters; not a substitute for smaller payloads |

Wi‑Fi **can** fail (2.4 GHz congestion, router, power save). For a thesis robot, keep **USB for sensing**; add Wi‑Fi later only if you need untethered debugging or a second viewer.

**On-camera preprocessing** (semantic mask, detections) remains the best long-term bandwidth win for forest use.

## Phase 2 pipeline

| Feature | Detail |
|---------|--------|
| Camera | QVGA **320×240** RGB565 (~0.75 Hz max at 921600 baud) |
| IMU | **LSM6DSOX** via `IMU` command → `/sensors/imu/data` |
| Commands | `PING`, `STATUS`, `SNAP`, `IMU` |

Extra library for firmware:

```bash
arduino-cli lib install "STM32duino LSM6DSOX"
```

## Firmware

Sketch: [`firmware/nicla_sensor_node/nicla_sensor_node.ino`](firmware/nicla_sensor_node/nicla_sensor_node.ino)

1. Arduino IDE 2.x or `arduino-cli`
2. Board: **Arduino Nicla Vision**
3. Upload the sketch, then reset the board

### `arduino-cli` (optional)

```bash
arduino-cli core update-index
arduino-cli core install arduino:mbed_nicla
arduino-cli lib install Arduino_OAuth Arduino_DebugUtils
arduino-cli compile -b arduino:mbed_nicla:nicla_vision \
  src/drivers_stack/forest_nicla_vision_ros2/firmware/nicla_sensor_node
arduino-cli upload -b arduino:mbed_nicla:nicla_vision -p /dev/ttyACM0 \
  src/drivers_stack/forest_nicla_vision_ros2/firmware/nicla_sensor_node
```

## Serial protocol (v1)

**Host → device** (ASCII line):

- `PING` → `PONG`
- `STATUS` → `OK camera=...`
- `SNAP` → `OK` + binary frame

**Binary frame** (`NICLAv1`): RGB565 payload, CRC16-CCITT-FALSE, 921600 baud, QQVGA (160×120).

## Host validation

**Phase 1**

```bash
bash scripts/nicla_phase1_validate.sh --snap
```

**Phase 2** (after re-flashing Phase 2 firmware)

```bash
colcon build --packages-select forest_nicla_vision_ros2 --symlink-install
source install/setup.bash
bash scripts/nicla_phase2_validate.sh
ros2 run forest_nicla_vision_ros2 nicla_device_probe --port /dev/ttyACM0 --imu
```

## ROS 2 bringup (sensor layer only)

```bash
source install/setup.bash
ros2 launch forest_nicla_vision_ros2 nicla_vision.launch.py
ros2 topic hz /camera/image_raw
```

Parameters: `config/nicla_vision.yaml` — set `serial_port` if auto-detect fails.

## Permissions

```bash
sudo usermod -aG dialout "$USER"
# log out and back in
```

## Contract

Publishes the stack camera contract:

- `/camera/image_raw` (`sensor_msgs/msg/Image`, `rgb8`)

Does **not** implement navigation, mapping, or perception logic.

## Bandwidth and FPS (why rqt looks “choppy”)

| Layer | Typical limit |
|-------|----------------|
| GC2145 sensor | up to **30 FPS** at QVGA (on-board) |
| USB CDC serial @ **921600** baud | ~**0.75 FPS** for raw RGB565 320×240 (~153 KB/frame) |
| ROS `image_rate_hz` default | **0.75** (matches serial, not a bug) |

The camera **can** capture faster; the bottleneck today is **how much data we push over serial**, not the sensor.

**Ways to increase effective FPS (product roadmap):**

1. **JPEG (or PNG) on the Nicla** before send — often 10–40× smaller → several FPS on the same baud rate; host decodes then runs perception.
2. **Send less data** — semantic mask, bounding boxes, class IDs (TinyML / OpenMV-style on device).
3. **Wi‑Fi** — separate from CDC; can carry Mbps if you use a proper socket/stream protocol (not line-at-a-time serial).
4. **Higher baud** — marginal gain; 921600 is already near practical CDC limits.
5. **USB 3.0 on the Raspberry Pi** — does **not** speed up the Nicla if the board enumerates as **USB 2.0 CDC** (typical). The cable/port is not the cap; **payload size and protocol** are.

**IMU @ 50 Hz on the same serial port:** IMU packets are tiny; rates of **25–50 Hz** are feasible, but each **SNAP** blocks the port for ~1.3 s, so IMU and image **share** one wire — expect gaps during frames.

**On-camera preprocessing:** yes — Nicla Vision (M7/M4) can run edge models and publish **detections / masks / features** instead of full RGB, which is often the best path for forest robotics (less bandwidth, lower latency on the Pi).

## Phase 3 pipeline (current)

| Feature | Topic / behaviour |
|---------|-------------------|
| `camera_info` | `/camera/camera_info` (placeholder K; calibrate later) |
| Health | `/sensors/nicla_serial/connected` (`std_msgs/Bool`) |
| Reconnect | Exponential backoff on USB errors |
| TF (optional) | `base_link` → `nicla_camera_optical_frame`, `nicla_imu_link` |
| IMU signs | `imu_accel_signs` / `imu_gyro_signs` in YAML |

```bash
ros2 launch forest_nicla_vision_ros2 nicla_vision.launch.py
bash scripts/nicla_phase3_validate.sh
```

## Phase 4 (current)

| Item | Detail |
|------|--------|
| JPEG | `SNAP_JPEG` on device, decode on Pi (Pillow), default `image_encoding:=jpeg` |
| FPS | Target **~3–5 Hz** QVGA on USB (vs ~0.75 Hz RGB565) |
| Wi-Fi | TCP port **9876**, same NICLAv1 protocol (`wifi_secrets.h`) |
| CV entry | `nicla_vision_perception.launch.py` → camera + `semantic_segmentation_node` |

### Flash Phase 4 firmware

```bash
arduino-cli lib install "STM32duino LSM6DSOX"
arduino-cli lib install "JPEGENC"
cp .../wifi_secrets.h.example .../wifi_secrets.h   # optional Wi-Fi

SKETCH=~/Projetos/Tese/forest-hybrid-ros2-stack/src/drivers_stack/forest_nicla_vision_ros2/firmware/nicla_sensor_node
arduino-cli compile -b arduino:mbed_nicla:nicla_vision "$SKETCH"
arduino-cli upload -b arduino:mbed_nicla:nicla_vision -p /dev/ttyACM0 "$SKETCH"
```

### Validate

```bash
bash scripts/nicla_phase4_validate.sh
ros2 launch forest_nicla_vision_ros2 nicla_vision.launch.py
```

### Computer vision layer

The **camera driver contract is complete** (`/camera/image_raw`, `/camera/camera_info`).  
Start segmentation on the Nicla stream:

```bash
ros2 launch forest_nicla_vision_ros2 nicla_vision_perception.launch.py \
  onnx_model_path:=/caminho/modelo.onnx
```

Subscribes to `/camera/image_raw` per `forest_semantic_segmentation` — no changes needed in that node.

### Wi-Fi bringup

```bash
# On Nicla: set FOREST_WIFI_SSID/PASS in wifi_secrets.h, re-flash
ros2 launch forest_nicla_vision_ros2 nicla_vision_wifi.launch.py wifi_host:=192.168.x.x
```

Keep **USB as primary** on the robot; Wi-Fi for bench/debug or secondary link.

### Wi-Fi coprocessor firmware (one-time, mandatory)

If serial shows:

`Failed to mount the filesystem containing the WiFi firmware`

the **Wi-Fi chip firmware is not installed** on the board QSPI (factory boards, or after some flashes). This is **not** your hotspot password.

```bash
bash scripts/nicla_install_wifi_firmware.sh
# Serial @ 115200 until "Firmware and certificates updated!"
# Then re-upload nicla_sensor_node.ino and run nicla_wifi_connect.sh
```

### Wi-Fi + phone hotspot (important)

- Hotspot **works** the same as home Wi-Fi if it is **2.4 GHz** (not 5 GHz only).
- Android: Hotspot → **AP band → 2.4 GHz** (or “Maximize compatibility” on iOS).
- The phone does **not** show the Nicla as a “paired device”; check the IP on serial (`INFO wifi_up ip=...`) or in the phone’s “connected devices” list.
- With `FOREST_ENABLE_WIFI 1`, default in `wifi_secrets.h` is **`FOREST_WIFI_AUTO_CONNECT 1`** (joins Wi-Fi on boot). If USB serial breaks on your board, set it to `0` and run `scripts/nicla_wifi_connect.sh` after each reset.

### Wi-Fi antenna (U.FL / “Molex”)

- Connector on the board is **U.FL (IPEX)**. Push the plug **straight down** until it **clicks** — no screwing rotation.
- The antenna puck can be rotated for mechanical clearance; that does **not** affect pairing.
- Without antenna, Wi-Fi may fail or be unstable; always connect before `WIFI_CONNECT`.

## Phase 5 (optional)

- Checkerboard calibration → real intrinsics
- On-device semantic mask (smallest bandwidth)

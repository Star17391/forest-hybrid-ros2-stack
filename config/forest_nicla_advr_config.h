/*
 * Forest — single Nicla Vision (ADVR stack) configuration.
 *
 * Edit this file only, then run:
 *   bash scripts/nicla/advr/apply_config.sh
 *
 * That updates:
 *   - third_party/nicla_vision_drivers/arduino/main/config.h  (firmware)
 *   - src/drivers_stack/forest_nicla_vision_ros2/config/nicla_advr_receiver.yaml (ROS 2)
 */

#ifndef FOREST_NICLA_ADVR_CONFIG_H
#define FOREST_NICLA_ADVR_CONFIG_H

/* --- Wi-Fi (firmware joins on boot) --- */
#define FOREST_NICLA_WIFI_SSID "YourWiFiSSID"
#define FOREST_NICLA_WIFI_PASSWORD "YourWiFiPassword"

/* IP of this PC on the same Wi-Fi as the Nicla (ip -4 addr, no dots).
 * Example: 192, 168, 1, 118  — must match the machine running ros2 launch. */
#define FOREST_NICLA_PC_IP 192, 168, 1, 118

/* tcp or udp (firmware: _TCP_ / _UDP_) */
#define FOREST_NICLA_NETWORK_TYPE tcp

/* --- Camera (firmware) --- */
#define FOREST_NICLA_CAM_FPS 60
#define FOREST_NICLA_USE_CAMERA 1
#define FOREST_NICLA_USE_IMU 1
#define FOREST_NICLA_USE_TOF 0
#define FOREST_NICLA_USE_MIC 0

/* --- ROS 2 receiver (host) --- */
#define FOREST_NICLA_RECEIVER_PORT 8002
#define FOREST_NICLA_CONNECTION_TYPE tcp
#define FOREST_NICLA_PUBLISH_RATE_HZ 500

/* 1 = JPEG from device (recommended); 0 = raw rgb565 */
#define FOREST_NICLA_CAMERA_RECEIVE_COMPRESSED 1
#define FOREST_NICLA_ENABLE_CAMERA_RAW 1
#define FOREST_NICLA_ENABLE_CAMERA_COMPRESSED 0
#define FOREST_NICLA_ENABLE_IMU 1
#define FOREST_NICLA_ENABLE_TOF 0
#define FOREST_NICLA_ENABLE_AUDIO 0
#define FOREST_NICLA_ENABLE_AUDIO_STAMPED 0

/* Topic prefix on device namespace (remapped to /camera, /sensors below) */
#define FOREST_NICLA_ROS_NAME "nicla"

#endif /* FOREST_NICLA_ADVR_CONFIG_H */

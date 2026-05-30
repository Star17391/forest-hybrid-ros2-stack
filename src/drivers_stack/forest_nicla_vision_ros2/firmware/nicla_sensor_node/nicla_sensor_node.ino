/*
 * forest-hybrid-ros2-stack — Nicla Vision sensor node (Phase 4)
 *
 * Features: QVGA camera, LSM6DSOX IMU, optional JPEG SNAP, optional Wi-Fi TCP mirror.
 *
 * arduino-cli lib install "STM32duino LSM6DSOX"
 * arduino-cli lib install "JPEGENC"
 *
 * Copy wifi_secrets.h.example -> wifi_secrets.h to enable Wi-Fi.
 */

#include <Arduino.h>
#include <SPI.h>
#include "camera.h"
#include "gc2145.h"
#include "LSM6DSOXSensor.h"
#include <JPEGENC.h>
#include "wifi_secrets.h"

#if FOREST_ENABLE_WIFI
#include <WiFi.h>
#endif

GC2145 galaxyCore;
Camera cam(galaxyCore);
FrameBuffer fb;

LSM6DSOXSensor imu(&SPI1, PIN_SPI_SS1);
bool imuReady = false;

static const uint32_t kSerialBaud = 921600;
static const int32_t kResolution = CAMERA_R320x240;
static const int32_t kPixelFormat = CAMERA_RGB565;

static const size_t kJpegBufferSize = 65536;
static uint8_t jpegBuffer[kJpegBufferSize];

#if FOREST_ENABLE_WIFI
WiFiServer wifiServer(FOREST_WIFI_TCP_PORT);
WiFiClient wifiClient;
bool wifiClientActive = false;
bool wifiServerStarted = false;
unsigned long wifiConnectStartedMs = 0;
bool wifiConnectRequested = false;
#endif

#pragma pack(push, 1)
struct ImuPayload {
  uint32_t timestamp_ms;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};
#pragma pack(pop)

static const float kGravity = 9.80665f;
static const float kMdpsToRadS = 3.14159265358979323846f / 180000.0f;

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

static void write_u16_le(Stream &stream, uint16_t v) {
  stream.write((uint8_t)(v & 0xFF));
  stream.write((uint8_t)((v >> 8) & 0xFF));
}

static void write_u32_le(Stream &stream, uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    stream.write((uint8_t)((v >> (8 * i)) & 0xFF));
  }
}

static void sendBinaryFrame(Stream &stream, uint8_t type, uint16_t width, uint16_t height,
                            const uint8_t *payload, uint32_t length) {
  stream.write((const uint8_t *)"NICLAv1", 7);
  stream.write(type);
  write_u16_le(stream, width);
  write_u16_le(stream, height);
  write_u32_le(stream, length);
  if (length > 0 && payload != nullptr) {
    stream.write(payload, length);
  }
  const uint16_t crc = crc16_ccitt_false(payload, length);
  write_u16_le(stream, crc);
  stream.flush();
}

static bool initImu() {
  SPI1.begin();
  if (imu.begin() != 0) {
    return false;
  }
  imu.Enable_X();
  imu.Enable_G();
  imu.Set_X_ODR(104.0f);
  imu.Set_G_ODR(104.0f);
  imu.Set_X_FS(2);
  imu.Set_G_FS(2000);
  return true;
}

// Same RGB565 decode as Arduino Camera visualizer (big-endian 16-bit pixels).
static void rgb565BeToRgb888(const uint8_t *src, uint8_t *dst, uint32_t pixelCount) {
  for (uint32_t i = 0; i < pixelCount; ++i) {
    const uint16_t word = ((uint16_t)src[2 * i] << 8) | (uint16_t)src[2 * i + 1];
    dst[3 * i] = (uint8_t)(((word >> 11) & 0x1Fu) << 3);
    dst[3 * i + 1] = (uint8_t)(((word >> 5) & 0x3Fu) << 2);
    dst[3 * i + 2] = (uint8_t)((word & 0x1Fu) << 3);
  }
}

static bool captureRgb565(uint16_t &width, uint16_t &height, const uint8_t *&data,
                          uint32_t &length) {
  if (cam.grabFrame(fb, 5000) != 0) {
    return false;
  }
  width = (uint16_t)cam.getResolutionWidth();
  height = (uint16_t)cam.getResolutionHeight();
  length = cam.frameSize();
  data = fb.getBuffer();
  return data != nullptr && length > 0;
}

static bool encodeJpegRgb888(const uint8_t *rgb888, uint16_t width, uint16_t height,
                             uint32_t &jpegLength) {
  JPEGENC encoder;
  JPEGENCODE state;
  if (encoder.open(jpegBuffer, (int)kJpegBufferSize) != JPEGE_SUCCESS) {
    return false;
  }
  if (encoder.encodeBegin(&state, width, height, JPEGE_PIXEL_RGB888, JPEGE_SUBSAMPLE_420,
                        JPEGE_Q_LOW) != JPEGE_SUCCESS) {
    encoder.close();
    return false;
  }
  if (encoder.addFrame(&state, (uint8_t *)rgb888, width * 3) != JPEGE_SUCCESS) {
    encoder.close();
    return false;
  }
  jpegLength = (uint32_t)encoder.close();
  return jpegLength > 0 && jpegLength <= kJpegBufferSize;
}

static bool encodeJpegFromRgb565(const uint8_t *rgb565, uint16_t width, uint16_t height,
                                 uint32_t &jpegLength) {
  const uint32_t pixels = (uint32_t)width * height;
  const size_t rgb888Bytes = (size_t)pixels * 3UL;
  uint8_t *rgb888Work = (uint8_t *)malloc(rgb888Bytes);
  if (rgb888Work == nullptr) {
    return false;
  }
  rgb565BeToRgb888(rgb565, rgb888Work, pixels);
  const bool ok = encodeJpegRgb888(rgb888Work, width, height, jpegLength);
  free(rgb888Work);
  return ok;
}

static void handleSnap(Stream &out, bool asJpeg) {
  uint16_t width = 0;
  uint16_t height = 0;
  const uint8_t *data = nullptr;
  uint32_t length = 0;

  if (!captureRgb565(width, height, data, length)) {
    out.println(F("ERR snap_failed"));
    return;
  }

  if (!asJpeg) {
    out.println(F("OK"));
    sendBinaryFrame(out, 0x01, width, height, data, length);
    return;
  }

  uint32_t jpegLen = 0;
  const bool jpegOk = encodeJpegFromRgb565(data, width, height, jpegLen);

  if (jpegOk) {
    out.println(F("OK"));
    sendBinaryFrame(out, 0x03, width, height, jpegBuffer, jpegLen);
    return;
  }

  // Last resort: uncompressed RGB565 (slow over Wi-Fi but keeps the stream alive).
  out.println(F("OK jpeg_fallback_rgb565"));
  sendBinaryFrame(out, 0x01, width, height, data, length);
}

static void handleImu(Stream &out) {
  if (!imuReady) {
    out.println(F("ERR imu_not_ready"));
    return;
  }

  int32_t accMg[3] = {0};
  int32_t gyrMdps[3] = {0};
  if (imu.Get_X_Axes(accMg) != 0 || imu.Get_G_Axes(gyrMdps) != 0) {
    out.println(F("ERR imu_read_failed"));
    return;
  }

  ImuPayload sample;
  sample.timestamp_ms = millis();
  sample.ax = accMg[0] * kGravity / 1000.0f;
  sample.ay = accMg[1] * kGravity / 1000.0f;
  sample.az = accMg[2] * kGravity / 1000.0f;
  sample.gx = gyrMdps[0] * kMdpsToRadS;
  sample.gy = gyrMdps[1] * kMdpsToRadS;
  sample.gz = gyrMdps[2] * kMdpsToRadS;

  out.println(F("OK"));
  sendBinaryFrame(out, 0x02, 0, 0, reinterpret_cast<const uint8_t *>(&sample),
                  sizeof(sample));
}

static void handleWifiStatus(Stream &out) {
#if FOREST_ENABLE_WIFI
  const int status = WiFi.status();
  if (status == WL_CONNECTED) {
    out.print(F("OK wifi=connected ip="));
    out.print(WiFi.localIP());
    out.print(F(" port="));
    out.println(FOREST_WIFI_TCP_PORT);
  } else if (wifiConnectRequested || wifiConnectStartedMs != 0) {
    out.println(F("OK wifi=connecting"));
  } else {
    out.println(F("ERR wifi=disconnected"));
  }
#else
  out.println(F("ERR wifi=disabled"));
#endif
}

static void handleCommand(Stream &out, const String &cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd.equalsIgnoreCase("PING")) {
    out.println(F("PONG"));
    return;
  }

  if (cmd.equalsIgnoreCase("STATUS")) {
    out.print(F("OK proto=1 camera=gc2145 res=320x240 fmt=rgb565,jpeg enc=rgb888 imu="));
    out.print(imuReady ? F("lsm6dsox") : F("none"));
#if FOREST_ENABLE_WIFI
    out.print(F(" wifi="));
    out.print(WiFi.status() == WL_CONNECTED ? F("up") : F("down"));
    if (WiFi.status() == WL_CONNECTED) {
      out.print(F(" ip="));
      out.print(WiFi.localIP());
    }
#endif
    out.println();
    return;
  }

  if (cmd.equalsIgnoreCase("SNAP")) {
    handleSnap(out, false);
    return;
  }

  if (cmd.equalsIgnoreCase("SNAP_JPEG") || cmd.equalsIgnoreCase("SNAPJPEG")) {
    handleSnap(out, true);
    return;
  }

  if (cmd.equalsIgnoreCase("IMU")) {
    handleImu(out);
    return;
  }

  if (cmd.equalsIgnoreCase("WIFI_STATUS")) {
    handleWifiStatus(out);
    return;
  }

#if FOREST_ENABLE_WIFI
  if (cmd.equalsIgnoreCase("WIFI_CONNECT") || cmd.equalsIgnoreCase("WIFI_RECONNECT")) {
    wifiConnectRequested = true;
    out.println(F("OK wifi_connect_scheduled"));
    return;
  }
#endif

  out.print(F("ERR unknown_cmd "));
  out.println(cmd);
}

static void pollStream(Stream &stream) {
  if (!stream.available()) {
    return;
  }
  String line = stream.readStringUntil('\n');
  handleCommand(stream, line);
}

#if FOREST_ENABLE_WIFI
static void startWifiConnect() {
  wifiConnectRequested = false;
  wifiConnectStartedMs = millis();
  wifiServerStarted = false;
  wifiClientActive = false;
  WiFi.disconnect();
  delay(100);
  WiFi.begin(FOREST_WIFI_SSID, FOREST_WIFI_PASS);
}

static void serviceWifiConnect() {
  if (!wifiConnectRequested && wifiConnectStartedMs == 0) {
    return;
  }
  if (wifiConnectRequested) {
    startWifiConnect();
  }

  const int status = WiFi.status();
  if (status == WL_CONNECTED) {
    if (!wifiServerStarted) {
      wifiServer.begin();
      wifiServerStarted = true;
      Serial.print(F("INFO wifi_up ip="));
      Serial.println(WiFi.localIP());
    }
    wifiConnectRequested = false;
    return;
  }

  if (millis() - wifiConnectStartedMs > 30000UL) {
    if (wifiConnectRequested || wifiConnectStartedMs != 0) {
      Serial.println(F("INFO wifi_timeout"));
    }
    wifiConnectRequested = false;
    wifiConnectStartedMs = 0;
  }
}

static void pollWifiClient() {
  if (!wifiClientActive) {
    wifiClient = wifiServer.accept();
    if (wifiClient) {
      wifiClientActive = true;
    }
    return;
  }
  if (!wifiClient.connected()) {
    wifiClient.stop();
    wifiClientActive = false;
    return;
  }
  pollStream(wifiClient);
}
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(kSerialBaud);
  // Do not block boot on USB: respond on serial as soon as camera is ready.
  unsigned long usbWaitStart = millis();
  while (!Serial && (millis() - usbWaitStart < 3000UL)) {
    delay(10);
  }

  imuReady = initImu();

  if (!cam.begin(kResolution, kPixelFormat, 30)) {
    Serial.println(F("ERR camera_init"));
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  Serial.println(F("INFO jpeg_encoder=rgb888_lazy"));
  Serial.println(F("READY"));
#if FOREST_ENABLE_WIFI
  Serial.println(F("INFO wifi_use_WIFI_CONNECT"));
#if defined(FOREST_WIFI_AUTO_CONNECT) && (FOREST_WIFI_AUTO_CONNECT)
  if (strlen(FOREST_WIFI_SSID) > 0) {
    wifiConnectRequested = true;
    Serial.println(F("INFO wifi_auto_connect"));
  }
#endif
#endif
}

void loop() {
  pollStream(Serial);
#if FOREST_ENABLE_WIFI
  serviceWifiConnect();
  pollWifiClient();
#endif
}

#include "forest_hybrid_flight/mavlink_client.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <thread>

// MAVLink vendorizado (dialeto ArduPilot). Os headers gerados disparam avisos de
// alinhamento; silenciados no CMake (-Wno-address-of-packed-member).
#include <ardupilotmega/mavlink.h>

namespace forest_hybrid_flight
{

namespace
{
// Identidade desta estação de controlo (GCS), igual ao source_system=255 do Python.
constexpr uint8_t kGcsSystemId = 255;
constexpr uint8_t kGcsCompId = MAV_COMP_ID_MISSIONPLANNER;

// type_mask de SET_POSITION_TARGET_LOCAL_NED: só posição (ignora vel/acel/yaw).
constexpr uint16_t kTmaskPos = 0b0000111111111000;

// Modos do ArduCopter (custom_mode).
bool apm_mode_id(const std::string & mode, uint32_t * out)
{
  if (mode == "STABILIZE") { *out = 0; return true; }
  if (mode == "GUIDED") { *out = 4; return true; }
  if (mode == "LOITER") { *out = 5; return true; }
  if (mode == "RTL") { *out = 6; return true; }
  if (mode == "LAND") { *out = 9; return true; }
  return false;
}

void sleep_s(double s)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

double monotonic_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

MavlinkClient::MavlinkClient(std::string host, int port, LogFn log)
: host_(std::move(host)), port_(port), log_(std::move(log))
{
}

MavlinkClient::~MavlinkClient()
{
  running_.store(false);
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool MavlinkClient::connect(double heartbeat_timeout_s)
{
  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) {
    log_("socket() falhou");
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port_));
  if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
    // Tentar resolver hostname.
    hostent * he = ::gethostbyname(host_.c_str());
    if (he == nullptr) {
      log_("host inválido: " + host_);
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    std::memcpy(&addr.sin_addr, he->h_addr_list[0], static_cast<size_t>(he->h_length));
  }

  if (::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    log_("connect() falhou a " + host_ + ":" + std::to_string(port_));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Desligar Nagle (latência) e dar timeout às leituras (para poder parar o rx loop).
  int one = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 200000;  // 200 ms
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Esperar o primeiro heartbeat do autopiloto (define o target_system).
  const double deadline = monotonic_s() + heartbeat_timeout_s;
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t buf[2048];
  bool got_hb = false;
  while (monotonic_s() < deadline && !got_hb) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      continue;  // timeout ou sem dados ainda
    }
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) != 1) {
        continue;
      }
      if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (hb.autopilot != MAV_AUTOPILOT_INVALID) {
          target_system_ = msg.sysid;
          target_component_ = (msg.compid == 0) ? 1 : msg.compid;
          got_hb = true;
          break;
        }
      }
    }
  }

  if (!got_hb) {
    log_("sem heartbeat do SITL");
    return false;
  }

  log_(
    "MAVLink ligado (sys=" + std::to_string(target_system_) + " comp=" +
    std::to_string(target_component_) + ")");
  connected_.store(true);
  running_.store(true);
  rx_thread_ = std::thread(&MavlinkClient::rx_loop, this);
  return true;
}

void MavlinkClient::rx_loop()
{
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t buf[2048];
  while (running_.load()) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) != 1) {
        continue;
      }
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);
          std::lock_guard<std::mutex> lk(state_mutex_);
          armed_ = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
          break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
          mavlink_gps_raw_int_t g;
          mavlink_msg_gps_raw_int_decode(&msg, &g);
          std::lock_guard<std::mutex> lk(state_mutex_);
          gps_fix_type_ = g.fix_type;
          break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
          mavlink_ekf_status_report_t e;
          mavlink_msg_ekf_status_report_decode(&msg, &e);
          std::lock_guard<std::mutex> lk(state_mutex_);
          ekf_flags_ = e.flags;
          break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT: {
          mavlink_statustext_t st;
          mavlink_msg_statustext_decode(&msg, &st);
          char text[51];
          std::memcpy(text, st.text, 50);
          text[50] = '\0';
          std::lock_guard<std::mutex> lk(state_mutex_);
          statustexts_.emplace_back(text);
          break;
        }
        default:
          break;
      }
    }
  }
}

bool MavlinkClient::send_buffer(const uint8_t * buf, int len)
{
  std::lock_guard<std::mutex> lk(tx_mutex_);
  if (fd_ < 0) {
    return false;
  }
  return ::write(fd_, buf, static_cast<size_t>(len)) == len;
}

void MavlinkClient::drain_statustext()
{
  std::vector<std::string> pending;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    pending.swap(statustexts_);
  }
  for (const auto & t : pending) {
    log_("[sitl] " + t);
  }
}

bool MavlinkClient::poll_until(const std::function<bool()> & pred, double timeout_s)
{
  const double deadline = monotonic_s() + timeout_s;
  while (monotonic_s() < deadline) {
    drain_statustext();
    if (pred()) {
      return true;
    }
    sleep_s(0.1);
  }
  return false;
}

void MavlinkClient::request_data_streams()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_request_data_stream_pack(
    kGcsSystemId, kGcsCompId, &msg,
    target_system_, target_component_, MAV_DATA_STREAM_ALL, 5, 1);
  const int len = mavlink_msg_to_send_buffer(buf, &msg);
  send_buffer(buf, len);
}

bool MavlinkClient::wait_gps(double timeout_s)
{
  return poll_until(
    [this]() {
      std::lock_guard<std::mutex> lk(state_mutex_);
      return gps_fix_type_ >= 3;
    },
    timeout_s);
}

bool MavlinkClient::wait_armable(double timeout_s)
{
  const uint32_t need = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_REL;
  return poll_until(
    [this, need]() {
      std::lock_guard<std::mutex> lk(state_mutex_);
      return (ekf_flags_ & need) == need;
    },
    timeout_s);
}

bool MavlinkClient::set_mode(const std::string & mode)
{
  uint32_t mode_id = 0;
  if (!apm_mode_id(mode, &mode_id)) {
    log_("modo desconhecido: " + mode);
    return false;
  }
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_set_mode_pack(
    kGcsSystemId, kGcsCompId, &msg,
    target_system_, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id);
  const int len = mavlink_msg_to_send_buffer(buf, &msg);
  return send_buffer(buf, len);
}

bool MavlinkClient::is_armed()
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return armed_;
}

bool MavlinkClient::arm_with_retry(double timeout_s)
{
  // Drena PreArm pendentes antes de tentar (mensagens periódicas).
  const double t0 = monotonic_s() + 3.0;
  while (monotonic_s() < t0) {
    drain_statustext();
    sleep_s(0.1);
  }

  const double deadline = monotonic_s() + timeout_s;
  while (monotonic_s() < deadline) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(
      kGcsSystemId, kGcsCompId, &msg,
      target_system_, target_component_, MAV_CMD_COMPONENT_ARM_DISARM, 0,
      1, 0, 0, 0, 0, 0, 0);
    const int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_buffer(buf, len);

    const double t_end = monotonic_s() + 3.0;
    while (monotonic_s() < t_end) {
      drain_statustext();
      if (is_armed()) {
        return true;
      }
      sleep_s(0.1);
    }
  }
  return false;
}

void MavlinkClient::takeoff(double alt_m)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
    kGcsSystemId, kGcsCompId, &msg,
    target_system_, target_component_, MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, static_cast<float>(alt_m));
  const int len = mavlink_msg_to_send_buffer(buf, &msg);
  send_buffer(buf, len);
}

void MavlinkClient::send_position_target_ned(double north, double east, double down)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_set_position_target_local_ned_pack(
    kGcsSystemId, kGcsCompId, &msg,
    0, target_system_, target_component_, MAV_FRAME_LOCAL_NED, kTmaskPos,
    static_cast<float>(north), static_cast<float>(east), static_cast<float>(down),
    0, 0, 0,
    0, 0, 0,
    0, 0);
  const int len = mavlink_msg_to_send_buffer(buf, &msg);
  send_buffer(buf, len);
}

}  // namespace forest_hybrid_flight

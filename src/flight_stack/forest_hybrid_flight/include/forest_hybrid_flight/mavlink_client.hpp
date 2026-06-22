#pragma once

// Cliente MAVLink mínimo para ArduPilot SITL, em C++ puro (socket TCP + MAVLink
// vendorizado). Porta 1:1 a sequência já validada em pymavlink
// (scripts/autopilot/sitl_hover_check.py + hybrid_trajectory_demo.py):
//   connect → heartbeat → request streams → wait GPS → wait EKF armable →
//   set_mode(GUIDED) → arm (com retry) → NAV_TAKEOFF →
//   SET_POSITION_TARGET_LOCAL_NED (modo posição) → set_mode(LAND).
//
// As decisões de posição (chegou ao waypoint, pousou) são tomadas a montante pelo
// nó ROS com base na pose fundida; este cliente só comanda o autopiloto.

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace forest_hybrid_flight
{

class MavlinkClient
{
public:
  using LogFn = std::function<void(const std::string &)>;

  MavlinkClient(std::string host, int port, LogFn log);
  ~MavlinkClient();

  MavlinkClient(const MavlinkClient &) = delete;
  MavlinkClient & operator=(const MavlinkClient &) = delete;

  // Abre o socket TCP e espera o primeiro heartbeat do autopiloto. Inicia o rx loop.
  bool connect(double heartbeat_timeout_s);
  // Sem MAVProxy ninguém pede streams → pedir ALL @ 5 Hz para GPS/EKF/POSITION fluírem.
  void request_data_streams();

  bool wait_gps(double timeout_s);       // fix_type >= 3
  bool wait_armable(double timeout_s);   // EKF: atitude + vel horiz + pos horiz
  bool set_mode(const std::string & mode);  // "GUIDED" | "LAND" (ArduCopter)
  bool arm_with_retry(double timeout_s);
  void takeoff(double alt_m);
  // Alvo de posição em NED (metros). O nó faz a conversão ENU(map)→NED.
  void send_position_target_ned(double north, double east, double down);

  bool is_armed();
  bool connected() const { return connected_.load(); }

  // Posição local do ArduPilot EKF3 (NED, metros, relativa ao datum de arranque).
  // Preenchida pelo rx_loop ao receber LOCAL_POSITION_NED.
  struct LocalPositionNed {
    float x{0};         // North (m)
    float y{0};         // East  (m)
    float z{0};         // Down  (m, negativo = altitude)
    float vx{0};        // m/s
    float vy{0};
    float vz{0};
    uint32_t time_boot_ms{0};
    bool valid{false};  // true após o primeiro LOCAL_POSITION_NED recebido
  };
  LocalPositionNed get_local_position() const;

  // Atitude do ArduPilot EKF3 (radianos, corpo FRD relativo a NED).
  // Preenchida pelo rx_loop ao receber ATTITUDE. A conversão para ENU/base_link
  // (FLU) é feita a jusante (hybrid_hop_executor, ver ap_enu.hpp).
  struct Attitude {
    float roll{0};      // rad (NED)
    float pitch{0};     // rad (NED)
    float yaw{0};       // rad (NED)
    bool valid{false};  // true após o primeiro ATTITUDE recebido
  };
  Attitude get_attitude() const;

private:
  void rx_loop();
  bool send_buffer(const uint8_t * buf, int len);
  void drain_statustext();
  bool poll_until(const std::function<bool()> & pred, double timeout_s);

  std::string host_;
  int port_;
  LogFn log_;
  int fd_{-1};

  // Alvo (autopiloto). Sem MAVProxy: sys 1 comp 1 (MAV_COMP_ID_AUTOPILOT1).
  uint8_t target_system_{1};
  uint8_t target_component_{1};

  std::thread rx_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};

  std::mutex tx_mutex_;
  mutable std::mutex state_mutex_;
  // Estado atualizado pelo rx_loop.
  bool armed_{false};
  int gps_fix_type_{0};
  uint32_t ekf_flags_{0};
  std::vector<std::string> statustexts_;
  LocalPositionNed local_pos_ned_;
  Attitude attitude_;
};

}  // namespace forest_hybrid_flight

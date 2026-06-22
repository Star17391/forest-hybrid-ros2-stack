// hybrid_hop_executor — executa um "salto aéreo" híbrido a pedido.
//
// Dispara com um HybridHopRequest em /forest_gen/hybrid/hop_request (manualmente
// ou pelo mission_manager). Orquestra a FSM de transição (hybrid_transition_manager)
// e o voo via MAVLink (ArduPilot), reportando o progresso em /forest_gen/hybrid/hop_status:
//
//   IDLE → (to_aerial) → arm+takeoff → CRUISE até (land_x,land_y,cruise_alt) →
//   LAND → espera pousar estável → (to_ground) → DONE
//
// As decisões de posição usam a pose fundida (/state/pose_fused). O MAVLink só comanda.
// O ponto de aterragem vem SEMPRE no pedido (quem dispara escolhe um sítio seguro);
// a seleção autónoma de zona segura fica para a camada de perceção (LiDAR 3D).

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "forest_hybrid_flight/ap_enu.hpp"
#include "forest_hybrid_flight/mavlink_client.hpp"
#include "forest_hybrid_msgs/msg/hybrid_hop_request.hpp"
#include "forest_hybrid_msgs/msg/hybrid_hop_status.hpp"
#include "forest_hybrid_msgs/msg/hybrid_transition_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace forest_hybrid_flight
{

using HopStatus = forest_hybrid_msgs::msg::HybridHopStatus;

enum class Phase
{
  IDLE,
  WAIT_AERIAL,
  CRUISE,
  LANDING,
  WAIT_GROUND,
  DONE,
  FAILED,
};

const char * phase_name(Phase p)
{
  switch (p) {
    case Phase::IDLE: return "IDLE";
    case Phase::WAIT_AERIAL: return "WAIT_AERIAL";
    case Phase::CRUISE: return "CRUISE";
    case Phase::LANDING: return "LANDING";
    case Phase::WAIT_GROUND: return "WAIT_GROUND";
    case Phase::DONE: return "DONE";
    case Phase::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

double monotonic_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class HybridHopExecutor : public rclcpp::Node
{
public:
  HybridHopExecutor()
  : rclcpp::Node("hybrid_hop_executor")
  {
    mav_host_ = declare_parameter<std::string>("mavlink_host", "127.0.0.1");
    mav_port_ = declare_parameter<int>("mavlink_port", 5760);
    aerial_xy_tol_ = declare_parameter<double>("aerial_xy_tol_m", 0.55);
    aerial_z_tol_ = declare_parameter<double>("aerial_z_tol_m", 0.3);
    // landed_z_m e airborne_agl_m são AGL (acima do home/takeoff) — independentes do terreno.
    // landed_z_m: altitude AGL abaixo da qual o AP considera pousado; fallback para auto-disarm.
    landed_z_ = declare_parameter<double>("landed_z_m", 0.3);
    airborne_agl_ = declare_parameter<double>("airborne_agl_m", 0.5);
    landed_settle_ = declare_parameter<double>("landed_settle_sec", 1.5);
    gps_timeout_ = declare_parameter<double>("gps_timeout_sec", 60.0);
    ekf_timeout_ = declare_parameter<double>("ekf_timeout_sec", 90.0);
    arm_timeout_ = declare_parameter<double>("arm_timeout_sec", 60.0);
    phase_timeout_ = declare_parameter<double>("phase_timeout_sec", 180.0);

    // Frames e tópico da odometria do AP que ESTE nó publica (a partir da ligação 5760,
    // com stream sustentado). É a fonte única de /ardupilot/local_position_odom para a
    // autoridade map→odom (a porta secundária 5763 não sustentava o stream de
    // LOCAL_POSITION_NED → odometria congelada).
    ap_odom_topic_ = declare_parameter<std::string>(
      "ardupilot_odom_topic", "/ardupilot/local_position_odom");
    ap_map_frame_ = declare_parameter<std::string>("ap_map_frame", "map");
    ap_base_frame_ = declare_parameter<std::string>("ap_base_frame", "marble_hd2/base_link");

    pub_transition_ = create_publisher<std_msgs::msg::String>(
      "/forest_gen/hybrid/transition_request", 10);
    pub_status_ = create_publisher<HopStatus>("/forest_gen/hybrid/hop_status", 10);
    pub_ap_odom_ = create_publisher<nav_msgs::msg::Odometry>(ap_odom_topic_, 10);

    sub_request_ = create_subscription<forest_hybrid_msgs::msg::HybridHopRequest>(
      "/forest_gen/hybrid/hop_request", 10,
      std::bind(&HybridHopExecutor::on_request, this, std::placeholders::_1));
    sub_fsm_ = create_subscription<forest_hybrid_msgs::msg::HybridTransitionStatus>(
      "/forest_gen/hybrid/transition_status", 10,
      std::bind(&HybridHopExecutor::on_fsm, this, std::placeholders::_1));
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/state/pose_fused", 10,
      std::bind(&HybridHopExecutor::on_pose, this, std::placeholders::_1));

    tick_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&HybridHopExecutor::tick, this));
    hb_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&HybridHopExecutor::publish_status_heartbeat, this));
    // Publica /ardupilot/local_position_odom @ 10 Hz a partir da ligação 5760 (stream fiável).
    ap_odom_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&HybridHopExecutor::publish_ap_odom, this));

    // Ligar ao SITL em background (com retry).
    connect_thread_ = std::thread(&HybridHopExecutor::connect_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "hybrid_hop_executor pronto. Publica um HybridHopRequest em "
      "/forest_gen/hybrid/hop_request para saltar. MAVLink: %s:%d",
      mav_host_.c_str(), mav_port_);
  }

  ~HybridHopExecutor() override
  {
    if (connect_thread_.joinable()) {
      connect_thread_.join();
    }
    if (arm_thread_.joinable()) {
      arm_thread_.join();
    }
  }

private:
  // ── Ligação MAVLink ──────────────────────────────────────────────────
  void connect_loop()
  {
    auto log = [this](const std::string & s) { RCLCPP_INFO(get_logger(), "MAVLink: %s", s.c_str()); };
    while (rclcpp::ok() && !mav_connected_.load()) {
      auto client = std::make_unique<MavlinkClient>(mav_host_, mav_port_, log);
      if (client->connect(60.0)) {
        client->request_data_streams();
        mav_ = std::move(client);
        mav_connected_.store(true);
        return;
      }
      RCLCPP_WARN(get_logger(), "MAVLink: ligação falhou; novo tento em 5 s");
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
  }

  // ── Callbacks ────────────────────────────────────────────────────────
  void on_request(const forest_hybrid_msgs::msg::HybridHopRequest::SharedPtr msg)
  {
    if (phase_ != Phase::IDLE && phase_ != Phase::DONE && phase_ != Phase::FAILED) {
      RCLCPP_WARN(
        get_logger(), "Ignoro hop_request '%s': salto em curso (%s)",
        msg->command_id.c_str(), phase_name(phase_));
      return;
    }
    if (!mav_connected_.load()) {
      RCLCPP_ERROR(get_logger(), "Ignoro hop_request: MAVLink ainda não ligado");
      return;
    }
    command_id_ = msg->command_id;
    source_ = msg->source;
    land_x_ = msg->land_x;
    land_y_ = msg->land_y;
    cruise_alt_ = msg->cruise_alt_m;
    // reset de estado do salto
    armed_started_ = false;
    arming_in_progress_.store(false);
    arm_failed_.store(false);
    land_sent_ = false;
    landed_since_.reset();
    RCLCPP_INFO(
      get_logger(), "hop_request '%s' (de %s): aterrar em (%.2f,%.2f) alt=%.2f",
      command_id_.c_str(), source_.c_str(), land_x_, land_y_, cruise_alt_);
    enter_phase(Phase::WAIT_AERIAL, "pedido recebido");
    request_transition("to_aerial");
  }

  void on_fsm(const forest_hybrid_msgs::msg::HybridTransitionStatus::SharedPtr msg)
  {
    fsm_state_ = msg->state_name;
    airborne_ = msg->airborne;
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    x_ = msg->pose.position.x;
    y_ = msg->pose.position.y;
    z_ = msg->pose.position.z;
    have_pose_ = true;
  }

  // ── Helpers ──────────────────────────────────────────────────────────
  void enter_phase(Phase p, const std::string & detail)
  {
    phase_ = p;
    phase_entered_ = monotonic_s();
    detail_ = detail;
    RCLCPP_INFO(get_logger(), "fase → %s (%s)", phase_name(p), detail.c_str());
    publish_status();
  }

  double elapsed() const { return monotonic_s() - phase_entered_; }

  void request_transition(const std::string & cmd)
  {
    std_msgs::msg::String m;
    m.data = cmd;
    pub_transition_->publish(m);
    RCLCPP_INFO(get_logger(), "transition_request: %s", cmd.c_str());
  }

  uint8_t phase_to_state() const
  {
    switch (phase_) {
      case Phase::IDLE: return HopStatus::STATE_IDLE;
      case Phase::DONE: return HopStatus::STATE_DONE;
      case Phase::FAILED: return HopStatus::STATE_FAILED;
      default: return HopStatus::STATE_IN_PROGRESS;
    }
  }

  void publish_status()
  {
    HopStatus s;
    s.state = phase_to_state();
    s.command_id = command_id_;
    s.phase = phase_name(phase_);
    s.detail = detail_;
    pub_status_->publish(s);
  }

  void publish_status_heartbeat() { publish_status(); }

  // Publica a pose do AP (posição + atitude) como Odometry ENU em map/base_link.
  // Fonte: ligação 5760 (stream sustentado). Consumida pela autoridade map→odom no ar.
  void publish_ap_odom()
  {
    if (!mav_connected_.load() || !mav_) {
      return;
    }
    const auto pos = mav_->get_local_position();
    if (!pos.valid) {
      return;  // ainda sem LOCAL_POSITION_NED
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = ap_map_frame_;
    odom.child_frame_id = ap_base_frame_;
    // NED → ENU: East=NED_y, North=NED_x, Up=-NED_z.
    odom.pose.pose.position.x = static_cast<double>(pos.y);
    odom.pose.pose.position.y = static_cast<double>(pos.x);
    odom.pose.pose.position.z = static_cast<double>(-pos.z);
    const auto att = mav_->get_attitude();
    if (att.valid) {
      const auto q = ned_attitude_to_enu_baselink(att.roll, att.pitch, att.yaw);
      odom.pose.pose.orientation.x = q.x;
      odom.pose.pose.orientation.y = q.y;
      odom.pose.pose.orientation.z = q.z;
      odom.pose.pose.orientation.w = q.w;
    } else {
      odom.pose.pose.orientation.w = 1.0;
    }
    odom.twist.twist.linear.x = static_cast<double>(pos.vy);
    odom.twist.twist.linear.y = static_cast<double>(pos.vx);
    odom.twist.twist.linear.z = static_cast<double>(-pos.vz);
    pub_ap_odom_->publish(odom);
  }

  bool at_aerial() const
  {
    // Loop de controlo fechado no referencial do PRÓPRIO ArduPilot (LOCAL_POSITION_NED),
    // não no pose_fused/EKF. Comando e feedback no mesmo referencial (NED, relativo ao
    // home/takeoff) → independente do terreno, do EKF global e do TF map→odom.
    //   Alvo em map (ENU, home=origem do spawn): north = land_y, east = land_x.
    //   AP LOCAL_POSITION_NED: x = North, y = East.
    if (!ap_pos_valid_) {
      return false;
    }
    const double err_n = land_y_ - ap_x_ned_;
    const double err_e = land_x_ - ap_y_ned_;
    const double horiz = std::hypot(err_n, err_e);
    return horiz <= aerial_xy_tol_ &&
           std::abs(cruise_alt_ - ap_z_agl_) <= aerial_z_tol_;
  }

  // ENU(map)→NED: north=map_y, east=map_x, down=-map_z.
  void send_position_target_map(double mx, double my, double mz)
  {
    if (mav_) {
      mav_->send_position_target_ned(my, mx, -mz);
    }
  }

  // Corre numa thread: bloqueia em waits de GPS/EKF/arm.
  void arm_and_takeoff()
  {
    auto log = get_logger();
    if (!mav_) {
      arm_failed_.store(true);
      arming_in_progress_.store(false);
      return;
    }
    RCLCPP_INFO(log, "À espera de fix GPS…");
    if (!mav_->wait_gps(gps_timeout_)) {
      RCLCPP_ERROR(log, "GPS não ficou pronto — abortar takeoff");
      arm_failed_.store(true);
      arming_in_progress_.store(false);
      return;
    }
    RCLCPP_INFO(log, "À espera do EKF convergir (prearm)…");
    if (!mav_->wait_armable(ekf_timeout_)) {
      RCLCPP_WARN(log, "EKF não reportou pronto; tento armar à mesma");
    }
    RCLCPP_INFO(log, "Modo GUIDED…");
    mav_->set_mode("GUIDED");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(log, "A armar ArduPilot…");
    if (!mav_->arm_with_retry(arm_timeout_)) {
      RCLCPP_ERROR(log, "Não armou (ver mensagens PreArm acima)");
      arm_failed_.store(true);
      arming_in_progress_.store(false);
      return;
    }
    RCLCPP_INFO(log, "Armado — takeoff a %.2f m", cruise_alt_);
    mav_->takeoff(cruise_alt_);
    arming_in_progress_.store(false);
  }

  // ── Máquina de estados (20 Hz) ───────────────────────────────────────
  void tick()
  {
    if (phase_ == Phase::IDLE || phase_ == Phase::DONE || phase_ == Phase::FAILED) {
      return;
    }

    if (elapsed() > phase_timeout_) {
      enter_phase(Phase::FAILED, std::string("timeout em ") + phase_name(phase_));
      return;
    }
    if (arm_failed_.load()) {
      enter_phase(Phase::FAILED, "arm/takeoff falhou");
      return;
    }

    // Atualizar a posição do AP (LOCAL_POSITION_NED, NED relativo ao home/takeoff).
    //   AGL = -z_ned;  x_ned = North;  y_ned = East.
    // É a fonte de verdade do loop de controlo do salto — independente do terreno,
    // do EKF global e do TF map→odom (que podem demorar a convergir ou nem fundir).
    if (mav_) {
      const auto pos = mav_->get_local_position();
      if (pos.valid) {
        ap_x_ned_ = static_cast<double>(pos.x);
        ap_y_ned_ = static_cast<double>(pos.y);
        ap_z_agl_ = static_cast<double>(-pos.z);
        ap_pos_valid_ = true;
      }
    }

    switch (phase_) {
      case Phase::WAIT_AERIAL: {
        // A FSM transita AERIAL_READY→AERIAL_FLY quase instantaneamente; aceitar
        // qualquer estado aéreo para não perder a janela de arm.
        const bool fsm_aerial =
          (fsm_state_ == "AERIAL_READY" || fsm_state_ == "AERIAL_FLY" ||
           fsm_state_ == "AERIAL_HOVER");
        if (fsm_aerial && !armed_started_ && !arming_in_progress_.load()) {
          armed_started_ = true;
          arming_in_progress_.store(true);
          if (arm_thread_.joinable()) {
            arm_thread_.join();
          }
          arm_thread_ = std::thread(&HybridHopExecutor::arm_and_takeoff, this);
        } else if (armed_started_ && !arming_in_progress_.load() &&
                   ap_pos_valid_ && ap_z_agl_ >= airborne_agl_) {
          // Armado, takeoff comandado, e o AP confirma que está no ar (AGL fiável,
          // relativo ao home). NÃO depende do airborne_/pose_fused da FSM, que dependem
          // da convergência do EKF global e podem nunca disparar.
          enter_phase(Phase::CRUISE, "no ar");
        }
        break;
      }
      case Phase::CRUISE: {
        if (at_aerial()) {
          // Só LAND. NÃO pedir to_ground aqui (rodaria as hélices para horizontal
          // ainda no ar → perda de sustentação). A transição mecânica só após pousar.
          if (!land_sent_) {
            land_sent_ = true;
            if (mav_) {
              mav_->set_mode("LAND");
            }
            RCLCPP_INFO(get_logger(), "Modo LAND ativado");
            landed_since_.reset();
            enter_phase(Phase::LANDING, "no ponto, a descer");
          }
        } else {
          send_position_target_map(land_x_, land_y_, cruise_alt_);
        }
        break;
      }
      case Phase::LANDING: {
        // Detecção de pouso por duas fontes independentes do terreno:
        //   1. AGL via AP LOCAL_POSITION_NED: ap_z_agl_ < landed_z_ (relativo ao home/takeoff)
        //   2. Auto-disarm do AP: após LAND completo o ArduPilot desarma automaticamente
        //      (fallback para terreno diferente do home onde AGL pode não chegar a 0)
        // NÃO usa z_ do EKF nem airborne_ da FSM — ambos são absolutos no referencial map.
        const bool ap_disarmed = mav_ && !mav_->is_armed();
        const bool agl_low = ap_z_agl_ < landed_z_;
        const bool on_ground = agl_low || ap_disarmed;
        if (on_ground) {
          if (!landed_since_.has_value()) {
            landed_since_ = monotonic_s();
          } else if (monotonic_s() - *landed_since_ >= landed_settle_) {
            request_transition("to_ground");
            enter_phase(Phase::WAIT_GROUND, "no chão, a recolher");
          }
        } else {
          landed_since_.reset();
        }
        break;
      }
      case Phase::WAIT_GROUND: {
        if (fsm_state_ == "GROUND_DRIVE") {
          enter_phase(Phase::DONE, "salto concluído (de volta ao solo)");
        }
        break;
      }
      default:
        break;
    }
  }

  // ── Parâmetros ───────────────────────────────────────────────────────
  std::string mav_host_;
  int mav_port_{5760};
  double aerial_xy_tol_{0.55};
  double aerial_z_tol_{0.3};
  double landed_z_{0.3};      // AGL acima do home; fallback para auto-disarm
  double airborne_agl_{0.5};  // AGL mínimo (acima do home) para considerar em voo → CRUISE
  double landed_settle_{1.5};
  double gps_timeout_{60.0};
  double ekf_timeout_{90.0};
  double arm_timeout_{60.0};
  double phase_timeout_{180.0};

  // ── Estado ───────────────────────────────────────────────────────────
  Phase phase_{Phase::IDLE};
  double phase_entered_{0.0};
  std::string detail_{"ready"};
  std::string command_id_;
  std::string source_;
  double land_x_{0.0}, land_y_{0.0}, cruise_alt_{3.0};

  // pose_fused: só para diagnóstico/RViz — NÃO usado no loop de controlo do salto.
  double x_{0.0}, y_{0.0}, z_{0.35};
  bool have_pose_{false};
  // Posição do AP (LOCAL_POSITION_NED, NED relativo ao home) — fonte de verdade do controlo.
  double ap_x_ned_{0.0};   // North
  double ap_y_ned_{0.0};   // East
  double ap_z_agl_{0.0};   // AGL = -z_ned (acima do home/takeoff)
  bool ap_pos_valid_{false};
  std::string fsm_state_{"GROUND_DRIVE"};
  bool airborne_{false};   // só informativo (vem da FSM)

  bool armed_started_{false};
  std::atomic<bool> arming_in_progress_{false};
  std::atomic<bool> arm_failed_{false};
  bool land_sent_{false};
  std::optional<double> landed_since_;

  std::unique_ptr<MavlinkClient> mav_;
  std::atomic<bool> mav_connected_{false};
  std::thread connect_thread_;
  std::thread arm_thread_;

  std::string ap_odom_topic_, ap_map_frame_, ap_base_frame_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_transition_;
  rclcpp::Publisher<HopStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ap_odom_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::HybridHopRequest>::SharedPtr sub_request_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::HybridTransitionStatus>::SharedPtr sub_fsm_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::TimerBase::SharedPtr ap_odom_timer_;
};

}  // namespace forest_hybrid_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_hybrid_flight::HybridHopExecutor>());
  rclcpp::shutdown();
  return 0;
}

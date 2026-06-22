// Autoridade ÚNICA do TF map→odom — comuta a FONTE por modo (não há 2.º EKF).
//
// Arquitetura (docs: FOREST_TREE_SLAM_DESIGN.md §4–6, LAYER_CONTRACTS.md):
//   GROUND → map→odom = identidade (fallback Fase-1) OU silencioso quando o
//            Tree-SLAM for a autoridade (param ground_mode). O frame map existe SEMPRE.
//   AERIAL → map→odom derivado DIRETAMENTE da pose do ArduPilot (posição + atitude),
//            NÃO de um EKF global. A pose absoluta no ar vem do ArduPilot EKF3 (§6).
//
// Nunca dois publishers de map→odom em simultâneo.
//
// No ar, a pose de base_link no referencial map é:
//   T_map_base = T_map_takeoff · T_apHome_base
// onde:
//   T_map_takeoff = snapshot de map→base no instante do takeoff (GROUND→AERIAL)
//                   ("exporta a pose atual como origin do ArduPilot", design §5.4)
//   T_apHome_base = pose do ArduPilot relativa ao home/takeoff (/ardupilot/local_position_odom)
// e o TF publicado é:
//   T_map_odom = T_map_base · T_odom_base⁻¹   (T_odom_base do EKF local, sempre presente)
//
// Continuidade garantida: no takeoff T_apHome_base ≈ identidade ⇒ T_map_base = T_map_takeoff.
//
// Gatilho de modo (OR): /system/locomotion_mode == MODE_AERIAL  ou  /slam/status == AERIAL.

#include <memory>
#include <optional>
#include <string>

#include "forest_hybrid_msgs/msg/operation_mode.hpp"
#include "forest_hybrid_msgs/msg/slam_status.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using OperationMode = forest_hybrid_msgs::msg::OperationMode;
using SlamStatus    = forest_hybrid_msgs::msg::SlamStatus;

class MapOdomAuthorityNode : public rclcpp::Node
{
public:
  MapOdomAuthorityNode()
  : Node("map_odom_authority_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    map_frame_       = declare_parameter<std::string>("map_frame",       "map");
    odom_frame_      = declare_parameter<std::string>("odom_frame",      "odom");
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "marble_hd2/base_link");
    publish_hz_      = declare_parameter<double>("publish_hz",           30.0);
    tf_timeout_      = declare_parameter<double>("tf_lookup_timeout_sec", 0.1);
    // ground_mode: "identity" = publica map→odom identidade no solo (Fase-1, sem Tree-SLAM);
    //              "silent"   = não publica no solo (Tree-SLAM é a autoridade — Fase 2+).
    ground_mode_     = declare_parameter<std::string>("ground_mode", "identity");
    ap_topic_        = declare_parameter<std::string>(
      "ardupilot_odom_topic", "/ardupilot/local_position_odom");
    // Salvaguarda da regra de ouro independente do ground_mode: se um SLAM
    // anunciar via /slam/status que é o dono de map→odom (owns_map_to_odom),
    // a autoridade CEDE no solo — mesmo que ground_mode="identity" (evita dois
    // publishers em simultâneo se a propagação do launch falhar). Retoma se o
    // anúncio ficar obsoleto há mais de este timeout (SLAM morto/parado).
    slam_authority_timeout_sec_ = declare_parameter<double>("slam_authority_timeout_sec", 1.0);

    // Pose do ArduPilot relativa ao home (ENU, base_link em map@home): fonte aérea.
    sub_ap_ = create_subscription<nav_msgs::msg::Odometry>(
      ap_topic_, 10,
      std::bind(&MapOdomAuthorityNode::on_ap_odom, this, std::placeholders::_1));

    rclcpp::QoS transient_local(1);
    transient_local.transient_local();
    sub_mode_ = create_subscription<OperationMode>(
      "/system/locomotion_mode", transient_local,
      std::bind(&MapOdomAuthorityNode::on_mode, this, std::placeholders::_1));
    sub_slam_ = create_subscription<SlamStatus>(
      "/slam/status", transient_local,
      std::bind(&MapOdomAuthorityNode::on_slam, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MapOdomAuthorityNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "map_odom_authority: autoridade ÚNICA de %s→%s. Solo=%s, Ar=ArduPilot direto (%s).",
      map_frame_.c_str(), odom_frame_.c_str(), ground_mode_.c_str(), ap_topic_.c_str());
  }

private:
  bool is_aerial() const { return locomotion_aerial_ || slam_aerial_; }

  void on_ap_odom(const nav_msgs::msg::Odometry::SharedPtr msg) { last_ap_ = msg; }

  void on_mode(const OperationMode::SharedPtr msg)
  {
    const bool was = locomotion_aerial_;
    locomotion_aerial_ = (msg->mode == OperationMode::MODE_AERIAL);
    if (locomotion_aerial_ != was) {
      RCLCPP_INFO(
        get_logger(), "locomotion_mode → %s  (autoridade map→odom: %s)",
        msg->mode_name.c_str(), locomotion_aerial_ ? "ArduPilot (ar)" : "solo");
    }
  }

  void on_slam(const SlamStatus::SharedPtr msg)
  {
    const bool was = slam_aerial_;
    slam_aerial_ = (msg->mode == SlamStatus::AERIAL);
    if (slam_aerial_ != was) {
      RCLCPP_INFO(get_logger(), "slam_status aerial=%s", slam_aerial_ ? "true" : "false");
    }
    slam_owns_map_to_odom_ = msg->owns_map_to_odom;
    last_slam_status_time_ = now();
    got_slam_status_ = true;
  }

  // Um SLAM está ativamente a reivindicar a autoridade de map→odom?
  // (anúncio recente + owns_map_to_odom=true). Gate de staleness para retomar
  // o controlo se o SLAM deixar de publicar (morto/parado).
  bool slam_claims_authority()
  {
    if (!got_slam_status_ || !slam_owns_map_to_odom_) {
      return false;
    }
    return (now() - last_slam_status_time_).seconds() < slam_authority_timeout_sec_;
  }

  // Lookup TF; devolve std::nullopt em falha (com aviso throttled).
  std::optional<tf2::Transform> lookup(const std::string & target, const std::string & source)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_.lookupTransform(
        target, source, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "TF %s→%s: %s",
        target.c_str(), source.c_str(), ex.what());
      return std::nullopt;
    }
    tf2::Transform out;
    tf2::fromMsg(t.transform, out);
    return out;
  }

  void publish_tf(const tf2::Transform & T_map_odom)
  {
    // Stamp SEMPRE com now() (monotónico). Usar o stamp da mensagem do AP fazia o
    // timestamp regredir/atrasar → TF rejeitava ("TF_OLD_DATA, data from the past") →
    // buracos no map→odom → RViz "No transform base_link→map". O VALOR vem do AP; o
    // tempo é o atual (com use_sim_time, now() devolve o tempo de simulação).
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp    = now();
    out.header.frame_id = map_frame_;
    out.child_frame_id  = odom_frame_;
    tf2::toMsg(T_map_odom, out.transform);

    const auto & t = out.transform.translation;
    const auto & r = out.transform.rotation;
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
        !std::isfinite(r.x) || !std::isfinite(r.y) || !std::isfinite(r.z) ||
        !std::isfinite(r.w))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "TF map→odom NaN/Inf — omitido.");
      return;
    }
    tf_broadcaster_.sendTransform(out);
  }

  void on_timer()
  {
    if (!is_aerial()) {
      // GROUND: a próxima descolagem volta a fazer snapshot.
      took_snapshot_ = false;

      // Regra de ouro: se o Tree-SLAM reivindica a autoridade (owns_map_to_odom
      // via /slam/status), a autoridade CEDE — independentemente de ground_mode.
      // Isto garante um único publisher de map→odom mesmo que o launch não tenha
      // propagado ground_mode="silent" (caso real observado).
      const bool yield = slam_claims_authority();
      if (yield != yielding_to_slam_) {
        yielding_to_slam_ = yield;
        RCLCPP_INFO(
          get_logger(), "GROUND: autoridade map→odom %s (Tree-SLAM owns=%s).",
          yield ? "CEDIDA ao Tree-SLAM" : "retomada", yield ? "true" : "false");
      }
      if (yield) {
        return;  // Tree-SLAM é o único publisher no solo.
      }

      // Sem SLAM dono: comportamento clássico. MANTÉM o último map→odom (não repõe
      // identidade!). Antes do 1.º voo é identidade (robô na origem); depois de aterrar
      // mantém a correção que coloca o robô na pose de aterragem — senão o robô
      // "teletransportava-se" para a origem do salto, porque o odom→base do EKF local
      // não segue o voo (lagartas paradas no ar). É a ponte até o Tree-SLAM fazer
      // RELOCALIZING na aterragem (design §5.4).
      if (ground_mode_ != "silent") {
        publish_tf(last_map_odom_);
      }
      return;  // ground_mode == "silent": Tree-SLAM é a autoridade.
    }

    // AERIAL — pose vem do ArduPilot.
    if (!last_ap_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "AERIAL mas sem %s — aguardar ArduPilot", ap_topic_.c_str());
      return;
    }

    // Snapshot de map→base no takeoff (exporta a pose atual como origem do AP).
    if (!took_snapshot_) {
      const auto T_map_base_now = lookup(map_frame_, base_link_frame_);
      if (!T_map_base_now) {
        return;  // ainda sem map→base; tenta no próximo tick
      }
      T_map_takeoff_ = *T_map_base_now;
      took_snapshot_ = true;
      RCLCPP_INFO(get_logger(), "takeoff: snapshot map→base capturado");
    }

    // T_apHome_base: pose do AP relativa ao home (ENU), já com orientação.
    tf2::Transform T_apHome_base;
    tf2::fromMsg(last_ap_->pose.pose, T_apHome_base);

    // T_odom_base do EKF local (sempre publicado).
    const auto T_odom_base = lookup(odom_frame_, base_link_frame_);
    if (!T_odom_base) {
      return;
    }

    const tf2::Transform T_map_base = T_map_takeoff_ * T_apHome_base;
    last_map_odom_ = T_map_base * T_odom_base->inverse();
    publish_tf(last_map_odom_);  // guarda para manter (hold) quando voltar ao solo
  }

  std::string map_frame_, odom_frame_, base_link_frame_, ground_mode_, ap_topic_;
  double publish_hz_{30.0};
  double tf_timeout_{0.1};
  double slam_authority_timeout_sec_{1.0};

  bool locomotion_aerial_{false};
  bool slam_aerial_{false};
  bool took_snapshot_{false};
  // Autoridade map→odom cedida ao Tree-SLAM no solo (via owns_map_to_odom).
  bool slam_owns_map_to_odom_{false};
  bool got_slam_status_{false};
  bool yielding_to_slam_{false};
  rclcpp::Time last_slam_status_time_{0, 0, RCL_ROS_TIME};
  tf2::Transform T_map_takeoff_{tf2::Transform::getIdentity()};
  // Último map→odom publicado — mantido no solo (identidade até ao 1.º voo; correção de
  // aterragem depois). Atualizado em cada tick aéreo.
  tf2::Transform last_map_odom_{tf2::Transform::getIdentity()};

  nav_msgs::msg::Odometry::SharedPtr last_ap_;

  tf2_ros::Buffer                tf_buffer_;
  tf2_ros::TransformListener     tf_listener_;
  tf2_ros::TransformBroadcaster  tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ap_;
  rclcpp::Subscription<OperationMode>::SharedPtr sub_mode_;
  rclcpp::Subscription<SlamStatus>::SharedPtr    sub_slam_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomAuthorityNode>());
  rclcpp::shutdown();
  return 0;
}

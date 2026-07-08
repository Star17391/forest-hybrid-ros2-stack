#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "forest_hybrid_msgs/msg/mission_status.hpp"
#include "forest_hybrid_msgs/msg/slam_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace forest_nav2_bringup
{

class MissionNav2Bridge : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using RouteGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  MissionNav2Bridge()
  : Node("mission_nav2_bridge")
  {
    declare_parameter<std::string>("mission_goal_topic", "/planning/mission_goal");
    declare_parameter<std::string>("mission_route_topic", "/planning/mission_route");
    declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
    declare_parameter<std::string>("navigate_through_poses_action", "navigate_through_poses");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("blocked_recovery_threshold", 3.0);
    // Gate de localização: por omissão NÃO navega com o SLAM LOST (produção, não
    // conduzir às cegas). require_slam=false desliga o gate → conduz por odom/EKF
    // (MVP / missão pequena sem landmarks; ex.: mundo plano onde o Tree-SLAM está
    // sempre LOST por não haver troncos).
    declare_parameter<bool>("require_slam", true);

    const auto goal_topic = get_parameter("mission_goal_topic").as_string();
    const auto route_topic = get_parameter("mission_route_topic").as_string();
    action_name_ = get_parameter("navigate_to_pose_action").as_string();
    route_action_name_ = get_parameter("navigate_through_poses_action").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    blocked_recovery_threshold_ = get_parameter("blocked_recovery_threshold").as_double();
    require_slam_ = get_parameter("require_slam").as_bool();

    pub_progress_ = create_publisher<std_msgs::msg::Float32>("/planning/progress", 10);
    pub_blocked_ = create_publisher<std_msgs::msg::Bool>("/planning/path_blocked", 10);
    pub_reached_ = create_publisher<std_msgs::msg::Bool>("/planning/goal_reached", 10);

    const auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, goal_qos,
      std::bind(&MissionNav2Bridge::on_mission_goal, this, std::placeholders::_1));

    // Camada de MISSÃO publica a ROTA (PATROL); a camada NAV2 (esta ponte) vai
    // buscá-la e segue-a com NavigateThroughPoses. Separação de camadas: a missão
    // diz "qual a rota", o nav2 trata do "como" (planeador D* Lite + controlador RPP).
    sub_route_ = create_subscription<nav_msgs::msg::Path>(
      route_topic, goal_qos,
      std::bind(&MissionNav2Bridge::on_mission_route, this, std::placeholders::_1));

    sub_mission_status_ = create_subscription<forest_hybrid_msgs::msg::MissionStatus>(
      "/mission/status", 10,
      std::bind(&MissionNav2Bridge::on_mission_status, this, std::placeholders::_1));

    sub_slam_status_ = create_subscription<forest_hybrid_msgs::msg::SlamStatus>(
      "/slam/status", 10,
      std::bind(&MissionNav2Bridge::on_slam_status, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    route_action_client_ =
      rclcpp_action::create_client<NavigateThroughPoses>(this, route_action_name_);

    RCLCPP_INFO(
      get_logger(),
      "mission_nav2_bridge: %s -> %s ; %s -> %s (map=%s)",
      goal_topic.c_str(), action_name_.c_str(),
      route_topic.c_str(), route_action_name_.c_str(), map_frame_.c_str());
  }

private:
  void on_mission_goal(const geometry_msgs::msg::PoseStamped & goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (require_slam_ && slam_lost_) {
      RCLCPP_WARN(get_logger(), "SLAM LOST — ignoring mission goal");
      publish_blocked_locked(true);
      return;
    }

    pending_goal_ = goal;
    if (pending_goal_.header.frame_id.empty()) {
      pending_goal_.header.frame_id = map_frame_;
    }
    have_pending_goal_ = true;
    user_cancel_ = false;
    dispatch_goal_locked();
  }

  void on_mission_route(const nav_msgs::msg::Path & route)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (route.poses.empty()) {
      RCLCPP_WARN(get_logger(), "mission_route vazio — ignorado");
      return;
    }
    if (require_slam_ && slam_lost_) {
      RCLCPP_WARN(get_logger(), "SLAM LOST — ignoring mission route");
      publish_blocked_locked(true);
      return;
    }
    pending_route_ = route;
    have_pending_route_ = true;
    user_cancel_ = false;
    dispatch_route_locked();
  }

  void on_mission_status(const forest_hybrid_msgs::msg::MissionStatus::SharedPtr msg)
  {
    using MS = forest_hybrid_msgs::msg::MissionStatus;
    // Cancelar SÓ em paragens reais. COMPLETED/IDLE NÃO são abortos: o
    // mission_manager tem deteção de chegada por distância que publica COMPLETED
    // (e depois IDLE) e cancelava a NavigateToPose ANTES de o nav2 reportar
    // SUCCEEDED → a ponte nunca publicava /planning/goal_reached → o `forest test
    // goto` dava TIMEOUT apesar de o robô chegar. Deixar o nav2 chegar ao seu
    // SUCCEEDED (é a autoridade do goal_reached). Goal novo cancela via
    // on_mission_goal; paragens reais via HOLDING/EMERGENCY/FAILED.
    if (
      msg->state == MS::STATE_HOLDING || msg->state == MS::STATE_EMERGENCY ||
      msg->state == MS::STATE_FAILED)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      user_cancel_ = true;
      cancel_active_goal_locked();
    }
  }

  void on_slam_status(const forest_hybrid_msgs::msg::SlamStatus::SharedPtr msg)
  {
    using SS = forest_hybrid_msgs::msg::SlamStatus;
    const bool lost = (msg->mode == SS::LOST);
    bool should_cancel = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (require_slam_ && lost && !slam_lost_) {
        should_cancel = true;
      }
      slam_lost_ = lost;
    }
    if (should_cancel) {
      RCLCPP_ERROR(get_logger(), "/slam/status LOST — cancel Nav2 and hold");
      std::lock_guard<std::mutex> lock(mutex_);
      user_cancel_ = true;
      cancel_active_goal_locked();
      publish_blocked_locked(true);
    }
  }

  void dispatch_goal_locked()
  {
    if (!have_pending_goal_) {
      return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "NavigateToPose action server not ready — goal queued");
      return;
    }

    cancel_active_goal_locked();

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = pending_goal_;
    nav_goal.pose.header.stamp = now();

    initial_distance_ = -1.0;
    last_recoveries_ = 0;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr & gh) { on_goal_response(gh); };
    opts.feedback_callback =
      [this](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> fb) { on_feedback(fb); };
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) { on_result(result); };

    action_client_->async_send_goal(nav_goal, opts);
    have_pending_goal_ = false;
    goal_in_flight_ = true;
    RCLCPP_INFO(
      get_logger(), "NavigateToPose goal (%.2f, %.2f) frame=%s",
      nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y,
      nav_goal.pose.header.frame_id.c_str());
  }

  void dispatch_route_locked()
  {
    if (!have_pending_route_) {
      return;
    }
    if (!route_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "NavigateThroughPoses server not ready — route queued");
      return;
    }

    cancel_active_goal_locked();

    NavigateThroughPoses::Goal nav_goal;
    nav_goal.poses.reserve(pending_route_.poses.size());
    for (auto pose : pending_route_.poses) {
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = map_frame_;
      }
      pose.header.stamp = now();
      nav_goal.poses.push_back(pose);
    }

    initial_distance_ = -1.0;
    last_recoveries_ = 0;

    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this](const RouteGoalHandle::SharedPtr & gh) { on_route_response(gh); };
    opts.feedback_callback =
      [this](
        RouteGoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> fb) {
        on_route_feedback(fb);
      };
    opts.result_callback =
      [this](const RouteGoalHandle::WrappedResult & result) { on_route_result(result); };

    route_action_client_->async_send_goal(nav_goal, opts);
    have_pending_route_ = false;
    RCLCPP_INFO(
      get_logger(), "NavigateThroughPoses route (%zu poses)", nav_goal.poses.size());
  }

  void on_route_response(const RouteGoalHandle::SharedPtr & gh)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gh) {
      RCLCPP_ERROR(get_logger(), "NavigateThroughPoses route rejected");
      if (!user_cancel_) {
        publish_blocked_locked(true);
      }
      return;
    }
    route_active_goal_ = gh;
  }

  void on_route_feedback(const std::shared_ptr<const NavigateThroughPoses::Feedback> fb)
  {
    std_msgs::msg::Float32 progress;
    progress.data = 0.0F;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (initial_distance_ < 0.0 && fb->distance_remaining > 0.01f) {
        initial_distance_ = fb->distance_remaining;
      }
      if (initial_distance_ > 0.01) {
        const double frac =
          1.0 - static_cast<double>(fb->distance_remaining) / initial_distance_;
        progress.data = static_cast<float>(std::clamp(frac, 0.0, 1.0));
      }
      last_recoveries_ = fb->number_of_recoveries;
    }
    pub_progress_->publish(progress);
  }

  void on_route_result(const RouteGoalHandle::WrappedResult & result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    route_active_goal_.reset();
    if (user_cancel_) {
      RCLCPP_INFO(get_logger(), "NavigateThroughPoses canceled (mission state)");
      return;
    }
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        publish_reached_locked();
        RCLCPP_INFO(get_logger(), "NavigateThroughPoses succeeded -> goal_reached");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(
          get_logger(), "NavigateThroughPoses aborted (recoveries=%d) -> path_blocked",
          last_recoveries_);
        publish_blocked_locked(true);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        if (last_recoveries_ >= static_cast<int16_t>(blocked_recovery_threshold_)) {
          publish_blocked_locked(true);
        }
        break;
      default:
        publish_blocked_locked(true);
        break;
    }
  }

  void on_goal_response(const GoalHandle::SharedPtr & gh)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gh) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose goal rejected");
      goal_in_flight_ = false;
      if (!user_cancel_) {
        publish_blocked_locked(true);
      }
      return;
    }
    active_goal_ = gh;
  }

  void on_feedback(const std::shared_ptr<const NavigateToPose::Feedback> fb)
  {
    std_msgs::msg::Float32 progress;
    progress.data = 0.0F;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (initial_distance_ < 0.0 && fb->distance_remaining > 0.01f) {
        initial_distance_ = fb->distance_remaining;
      }
      if (initial_distance_ > 0.01) {
        const double frac =
          1.0 - static_cast<double>(fb->distance_remaining) / initial_distance_;
        progress.data = static_cast<float>(std::clamp(frac, 0.0, 1.0));
      }
      last_recoveries_ = fb->number_of_recoveries;
    }
    pub_progress_->publish(progress);
  }

  void on_result(const GoalHandle::WrappedResult & result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_in_flight_ = false;
    active_goal_.reset();

    if (user_cancel_) {
      RCLCPP_INFO(get_logger(), "NavigateToPose canceled (mission state)");
      return;
    }

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        publish_reached_locked();
        RCLCPP_INFO(get_logger(), "NavigateToPose succeeded -> goal_reached");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(
          get_logger(), "NavigateToPose aborted (recoveries=%d) -> path_blocked",
          last_recoveries_);
        publish_blocked_locked(true);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        if (last_recoveries_ >= static_cast<int16_t>(blocked_recovery_threshold_)) {
          publish_blocked_locked(true);
        }
        break;
      default:
        publish_blocked_locked(true);
        break;
    }
  }

  void cancel_active_goal_locked()
  {
    if (active_goal_) {
      action_client_->async_cancel_goal(active_goal_);
      active_goal_.reset();
    }
    if (route_active_goal_) {
      route_action_client_->async_cancel_goal(route_active_goal_);
      route_active_goal_.reset();
    }
    goal_in_flight_ = false;
  }

  void publish_reached_locked()
  {
    std_msgs::msg::Bool reached;
    reached.data = true;
    pub_reached_->publish(reached);
    std_msgs::msg::Float32 done;
    done.data = 1.0F;
    pub_progress_->publish(done);
  }

  void publish_blocked_locked(bool blocked)
  {
    std_msgs::msg::Bool msg;
    msg.data = blocked;
    pub_blocked_->publish(msg);
  }

  std::mutex mutex_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_progress_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_blocked_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_reached_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_route_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::MissionStatus>::SharedPtr sub_mission_status_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::SlamStatus>::SharedPtr sub_slam_status_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr route_action_client_;

  GoalHandle::SharedPtr active_goal_;
  RouteGoalHandle::SharedPtr route_active_goal_;
  geometry_msgs::msg::PoseStamped pending_goal_;
  nav_msgs::msg::Path pending_route_;
  bool have_pending_goal_{false};
  bool have_pending_route_{false};
  bool goal_in_flight_{false};
  bool user_cancel_{false};
  bool slam_lost_{false};
  bool require_slam_{true};
  double initial_distance_{-1.0};
  int16_t last_recoveries_{0};
  double blocked_recovery_threshold_{3.0};
  std::string action_name_;
  std::string route_action_name_;
  std::string map_frame_;
};

}  // namespace forest_nav2_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_nav2_bringup::MissionNav2Bridge>());
  rclcpp::shutdown();
  return 0;
}

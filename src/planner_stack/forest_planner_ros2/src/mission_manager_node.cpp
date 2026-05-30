#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "forest_hybrid_msgs/msg/mission_ack.hpp"
#include "forest_hybrid_msgs/msg/mission_command.hpp"
#include "forest_hybrid_msgs/msg/mission_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace forest_planner_ros2
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double yaw_from_quaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void quaternion_from_yaw(double yaw, geometry_msgs::msg::Quaternion * q)
{
  q->x = 0.0;
  q->y = 0.0;
  q->z = std::sin(yaw * 0.5);
  q->w = std::cos(yaw * 0.5);
}

double shortest_angle_diff(double a, double b)
{
  double d = a - b;
  while (d > kPi) {
    d -= 2.0 * kPi;
  }
  while (d < -kPi) {
    d += 2.0 * kPi;
  }
  return d;
}

bool is_unspecified_orientation(const geometry_msgs::msg::Quaternion & q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-6) {
    return true;
  }
  const double wn = q.w / n;
  const double xn = q.x / n;
  const double yn = q.y / n;
  const double zn = q.z / n;
  return std::abs(wn - 1.0) < 0.01 && std::abs(xn) < 0.01 && std::abs(yn) < 0.01 && std::abs(zn) < 0.01;
}

}  // namespace

class MissionManagerNode final : public rclcpp::Node
{
public:
  MissionManagerNode() : Node("mission_manager_node")
  {
    declare_parameter<int>("max_replans", 3);
    declare_parameter<double>("goal_tolerance_m", 0.5);
    declare_parameter<double>("goal_tolerance_heading_deg", 25.0);
    declare_parameter<bool>("allow_auto_aerial_on_block", false);
    declare_parameter<std::string>("pose_topic", "/state/pose_fused");
    declare_parameter<bool>("allow_goal_reached_topic_shortcut", true);

    max_replans_ = std::max(1, static_cast<int>(get_parameter("max_replans").as_int()));
    goal_tolerance_m_ = std::max(0.01, get_parameter("goal_tolerance_m").as_double());
    const double heading_deg = std::max(0.1, get_parameter("goal_tolerance_heading_deg").as_double());
    goal_tolerance_heading_rad_ = heading_deg * kPi / 180.0;
    allow_auto_aerial_on_block_ = get_parameter("allow_auto_aerial_on_block").as_bool();
    pose_topic_ = get_parameter("pose_topic").as_string();
    allow_goal_reached_shortcut_ = get_parameter("allow_goal_reached_topic_shortcut").as_bool();

    cmd_sub_ = create_subscription<forest_hybrid_msgs::msg::MissionCommand>(
      "/mission/command", rclcpp::QoS(20),
      std::bind(&MissionManagerNode::on_command, this, std::placeholders::_1));
    ack_sub_ = create_subscription<forest_hybrid_msgs::msg::MissionAck>(
      "/mission/ack", rclcpp::QoS(10),
      std::bind(&MissionManagerNode::on_ack, this, std::placeholders::_1));
    progress_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/planning/progress", rclcpp::QoS(10),
      std::bind(&MissionManagerNode::on_progress, this, std::placeholders::_1));
    blocked_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/planning/path_blocked", rclcpp::QoS(10),
      std::bind(&MissionManagerNode::on_path_blocked, this, std::placeholders::_1));
    reached_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/planning/goal_reached", rclcpp::QoS(10),
      std::bind(&MissionManagerNode::on_goal_reached, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10),
      std::bind(&MissionManagerNode::on_pose, this, std::placeholders::_1));

    status_pub_ = create_publisher<forest_hybrid_msgs::msg::MissionStatus>("/mission/status", rclcpp::QoS(10));
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_goal",
      rclcpp::QoS(1).transient_local().reliable());
    route_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/planning/mission_route",
      rclcpp::QoS(1).transient_local().reliable());

    tick_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&MissionManagerNode::on_tick, this));

    publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_IDLE, "mission_manager ready");
  }

private:
  struct RuntimeMission
  {
    forest_hybrid_msgs::msg::MissionCommand command;
    size_t waypoint_index{0};
    uint32_t retries_used{0};
    float progress{0.0F};
  };

  static int command_priority(uint8_t command_type)
  {
    using C = forest_hybrid_msgs::msg::MissionCommand;
    if (command_type == C::CMD_EMERGENCY_STOP) {
      return 300;
    }
    if (command_type == C::CMD_RETURN_HOME) {
      return 200;
    }
    if (command_type == C::CMD_HOLD) {
      return 100;
    }
    return 10;
  }

  bool preempts_current(uint8_t incoming_type) const
  {
    if (!active_mission_.has_value()) {
      return true;
    }
    const int incoming = command_priority(incoming_type);
    const int current = command_priority(active_mission_->command.command_type);
    return incoming > current;
  }

  bool validate_command(const forest_hybrid_msgs::msg::MissionCommand & cmd, std::string & reason) const
  {
    using C = forest_hybrid_msgs::msg::MissionCommand;
    if (cmd.command_type == C::CMD_CLEAR_EMERGENCY_LATCH) {
      return true;
    }
    if (cmd.frame_type != C::FRAME_MAP && cmd.frame_type != C::FRAME_RELATIVE) {
      reason = "frame_type must be FRAME_MAP (0) or FRAME_RELATIVE (1) only";
      return false;
    }
    if (cmd.command_type == C::CMD_PATROL_WAYPOINTS) {
      if (cmd.frame_type != C::FRAME_MAP) {
        reason = "PATROL_WAYPOINTS requires FRAME_MAP (GNSS must be converted upstream)";
        return false;
      }
      if (
        cmd.waypoint_x.empty() || cmd.waypoint_y.empty() || cmd.waypoint_z.empty() ||
        cmd.waypoint_x.size() != cmd.waypoint_y.size() ||
        cmd.waypoint_y.size() != cmd.waypoint_z.size())
      {
        reason = "PATROL_WAYPOINTS requires x/y/z arrays with same non-zero length";
        return false;
      }
      if (
        !cmd.waypoint_yaw.empty() &&
        cmd.waypoint_yaw.size() != cmd.waypoint_x.size())
      {
        reason = "waypoint_yaw must be empty or same length as waypoint_x";
        return false;
      }
    }
    return true;
  }

  void enqueue_or_preempt(const forest_hybrid_msgs::msg::MissionCommand & cmd)
  {
    using C = forest_hybrid_msgs::msg::MissionCommand;
    if (cmd.command_type == C::CMD_EMERGENCY_STOP) {
      queue_.clear();
      RuntimeMission estop;
      estop.command = cmd;
      activate_mission(estop, "emergency_stop");
      return;
    }
    if (preempts_current(cmd.command_type)) {
      if (active_mission_.has_value()) {
        queue_.push_front(*active_mission_);
      }
      RuntimeMission fresh;
      fresh.command = cmd;
      activate_mission(fresh, "preempt");
      return;
    }

    RuntimeMission queued;
    queued.command = cmd;
    queue_.push_back(queued);
    publish_status(current_state_, "queued command: " + cmd.command_id);
  }

  void activate_mission(const RuntimeMission & mission, const std::string & origin)
  {
    active_mission_ = mission;
    waiting_return_home_ack_ = false;
    waiting_aerial_permission_ = false;
    last_published_goal_key_.clear();
    force_goal_republish_ = false;
    active_map_goal_.reset();

    using C = forest_hybrid_msgs::msg::MissionCommand;
    const auto & cmd = active_mission_->command;
    if (cmd.command_type == C::CMD_EMERGENCY_STOP) {
      emergency_latched_ = true;
    }
    if (cmd.command_type == C::CMD_RETURN_HOME) {
      waiting_return_home_ack_ = true;
      publish_status(
        forest_hybrid_msgs::msg::MissionStatus::STATE_WAITING_ACK,
        "RETURN_HOME requires operator ACK (" + origin + ")");
      return;
    }
    execute_active_command();
  }

  std::string current_publication_key() const
  {
    if (!active_mission_.has_value()) {
      return {};
    }
    using C = forest_hybrid_msgs::msg::MissionCommand;
    const auto & cmd = active_mission_->command;
    if (cmd.command_type == C::CMD_PATROL_WAYPOINTS) {
      return cmd.command_id + ":" + std::to_string(active_mission_->waypoint_index);
    }
    return cmd.command_id + ":single";
  }

  bool build_map_goal(double * mx, double * my, double * mz, std::string & detail) const
  {
    if (!active_mission_.has_value()) {
      return false;
    }
    using C = forest_hybrid_msgs::msg::MissionCommand;
    const auto & cmd = active_mission_->command;
    if (cmd.command_type == C::CMD_PATROL_WAYPOINTS) {
      const size_t idx = std::min(active_mission_->waypoint_index, cmd.waypoint_x.size() - 1U);
      *mx = cmd.waypoint_x[idx];
      *my = cmd.waypoint_y[idx];
      *mz = cmd.waypoint_z[idx];
      detail = "PATROL waypoint " + std::to_string(idx + 1) + "/" + std::to_string(cmd.waypoint_x.size());
      return true;
    }
    if (cmd.frame_type == C::FRAME_MAP) {
      *mx = cmd.target_x;
      *my = cmd.target_y;
      *mz = cmd.target_z;
      detail = "executing command";
      return true;
    }
    // FRAME_RELATIVE: freeze into map using latest fused pose
    if (!last_pose_.has_value()) {
      detail = "waiting fused pose to resolve FRAME_RELATIVE goal";
      return false;
    }
    if (last_pose_->header.frame_id != "map") {
      detail = "FRAME_RELATIVE requires pose in 'map' frame (got " + last_pose_->header.frame_id + ")";
      return false;
    }
    const double px = last_pose_->pose.position.x;
    const double py = last_pose_->pose.position.y;
    const double pz = last_pose_->pose.position.z;
    const double yaw = yaw_from_quaternion(
      last_pose_->pose.orientation.x, last_pose_->pose.orientation.y,
      last_pose_->pose.orientation.z, last_pose_->pose.orientation.w);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    *mx = px + cy * cmd.target_x - sy * cmd.target_y;
    *my = py + sy * cmd.target_x + cy * cmd.target_y;
    *mz = pz + cmd.target_z;
    detail = "executing command (RELATIVE resolved to map)";
    return true;
  }

  double desired_heading_rad(double goal_mx, double goal_my, const geometry_msgs::msg::Pose & robot_map) const
  {
    using C = forest_hybrid_msgs::msg::MissionCommand;
    if (!active_mission_.has_value()) {
      return yaw_from_quaternion(
        robot_map.orientation.x, robot_map.orientation.y, robot_map.orientation.z, robot_map.orientation.w);
    }
    const auto & cmd = active_mission_->command;
    if (cmd.command_type == C::CMD_PATROL_WAYPOINTS) {
      const size_t idx = std::min(active_mission_->waypoint_index, cmd.waypoint_x.size() - 1U);
      if (idx > 0U) {
        const double dx = cmd.waypoint_x[idx] - cmd.waypoint_x[idx - 1U];
        const double dy = cmd.waypoint_y[idx] - cmd.waypoint_y[idx - 1U];
        if (dx * dx + dy * dy < 1e-12) {
          return yaw_from_quaternion(
            robot_map.orientation.x, robot_map.orientation.y, robot_map.orientation.z, robot_map.orientation.w);
        }
        return std::atan2(dy, dx);
      }
      const double dx = goal_mx - robot_map.position.x;
      const double dy = goal_my - robot_map.position.y;
      if (dx * dx + dy * dy < 1e-12) {
        return yaw_from_quaternion(
          robot_map.orientation.x, robot_map.orientation.y, robot_map.orientation.z, robot_map.orientation.w);
      }
      return std::atan2(dy, dx);
    }
    const double dx = goal_mx - robot_map.position.x;
    const double dy = goal_my - robot_map.position.y;
    if (dx * dx + dy * dy < 1e-12) {
      return yaw_from_quaternion(
        robot_map.orientation.x, robot_map.orientation.y, robot_map.orientation.z, robot_map.orientation.w);
    }
    return std::atan2(dy, dx);
  }

  void execute_active_command()
  {
    if (!active_mission_.has_value()) {
      publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_IDLE, "idle");
      return;
    }

    using C = forest_hybrid_msgs::msg::MissionCommand;
    const auto & cmd = active_mission_->command;

    if (cmd.command_type == C::CMD_EMERGENCY_STOP) {
      publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_EMERGENCY, "EMERGENCY_STOP active");
      return;
    }
    if (cmd.command_type == C::CMD_HOLD) {
      publish_status(
        forest_hybrid_msgs::msg::MissionStatus::STATE_HOLDING,
        "HOLD active (mode unchanged; controller decides low-level behavior)");
      return;
    }
    if (cmd.command_type == C::CMD_PATROL_WAYPOINTS) {
      publish_patrol_route(cmd);
      return;
    }

    double mx = 0.0;
    double my = 0.0;
    double mz = 0.0;
    std::string detail;
    if (!build_map_goal(&mx, &my, &mz, detail)) {
      active_map_goal_.reset();
      publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_EXECUTING, detail);
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "map";
    goal.pose.position.x = mx;
    goal.pose.position.y = my;
    goal.pose.position.z = mz;

    double yaw_des = 0.0;
    if (cmd.command_type == C::CMD_PATROL_WAYPOINTS) {
      const size_t widx = std::min(active_mission_->waypoint_index, cmd.waypoint_x.size() - 1U);
      if (!cmd.waypoint_yaw.empty() && cmd.waypoint_yaw.size() == cmd.waypoint_x.size()) {
        yaw_des = cmd.waypoint_yaw[widx];
      } else if (last_pose_.has_value() && last_pose_->header.frame_id == "map") {
        yaw_des = desired_heading_rad(mx, my, last_pose_->pose);
      }
    } else if (cmd.use_target_yaw) {
      yaw_des = cmd.target_yaw_rad;
    } else if (last_pose_.has_value() && last_pose_->header.frame_id == "map") {
      yaw_des = desired_heading_rad(mx, my, last_pose_->pose);
    }
    quaternion_from_yaw(yaw_des, &goal.pose.orientation);

    const std::string key = current_publication_key();
    const bool should_publish = (key != last_published_goal_key_) || force_goal_republish_;
    force_goal_republish_ = false;
    if (!should_publish) {
      return;
    }

    goal_pub_->publish(goal);
    last_published_goal_key_ = key;
    active_map_goal_ = goal;
    RCLCPP_INFO(get_logger(), "Published /planning/mission_goal once (key=%s)", key.c_str());
    publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_EXECUTING, detail);
  }

  void publish_patrol_route(const forest_hybrid_msgs::msg::MissionCommand & cmd)
  {
    const std::string key = cmd.command_id + ":route";
    if (key == last_published_goal_key_ && !force_goal_republish_) {
      return;
    }
    force_goal_republish_ = false;

    nav_msgs::msg::Path route;
    route.header.stamp = now();
    route.header.frame_id = "map";

    for (size_t i = 0; i < cmd.waypoint_x.size(); ++i) {
      geometry_msgs::msg::PoseStamped wp;
      wp.header = route.header;
      wp.pose.position.x = cmd.waypoint_x[i];
      wp.pose.position.y = cmd.waypoint_y[i];
      wp.pose.position.z = cmd.waypoint_z[i];

      double yaw_des = 0.0;
      if (!cmd.waypoint_yaw.empty() && cmd.waypoint_yaw.size() == cmd.waypoint_x.size()) {
        yaw_des = cmd.waypoint_yaw[i];
      } else if (i + 1 < cmd.waypoint_x.size()) {
        const double dx = cmd.waypoint_x[i + 1] - cmd.waypoint_x[i];
        const double dy = cmd.waypoint_y[i + 1] - cmd.waypoint_y[i];
        yaw_des = std::atan2(dy, dx);
      } else if (i > 0) {
        const double dx = cmd.waypoint_x[i] - cmd.waypoint_x[i - 1];
        const double dy = cmd.waypoint_y[i] - cmd.waypoint_y[i - 1];
        yaw_des = std::atan2(dy, dx);
      } else if (last_pose_.has_value()) {
        yaw_des = desired_heading_rad(wp.pose.position.x, wp.pose.position.y, last_pose_->pose);
      }
      quaternion_from_yaw(yaw_des, &wp.pose.orientation);
      route.poses.push_back(wp);
    }

    route_pub_->publish(route);
    last_published_goal_key_ = key;
    active_map_goal_ = route.poses.back();

    RCLCPP_INFO(
      get_logger(), "Published /planning/mission_route (%zu waypoints, key=%s)",
      route.poses.size(), key.c_str());
    publish_status(
      forest_hybrid_msgs::msg::MissionStatus::STATE_EXECUTING,
      "PATROL route with " + std::to_string(route.poses.size()) + " waypoints");
  }

  void complete_active(const std::string & detail)
  {
    publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_COMPLETED, detail);
    active_mission_.reset();
    last_published_goal_key_.clear();
    active_map_goal_.reset();
    pump_queue();
  }

  void fail_active_and_hold(const std::string & detail)
  {
    publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_FAILED, detail);
    RuntimeMission hold_mission;
    active_mission_ = hold_mission;
    active_mission_->command.command_id = "auto_hold_after_fail";
    active_mission_->command.command_type = forest_hybrid_msgs::msg::MissionCommand::CMD_HOLD;
    last_published_goal_key_.clear();
    active_map_goal_.reset();
    execute_active_command();
  }

  void request_aerial_permission()
  {
    waiting_aerial_permission_ = true;
    publish_status(
      forest_hybrid_msgs::msg::MissionStatus::STATE_WAITING_ACK,
      "Ground path blocked. Send /mission/ack approved=true to allow aerial mode.");
  }

  void on_command(const forest_hybrid_msgs::msg::MissionCommand::SharedPtr msg)
  {
    using C = forest_hybrid_msgs::msg::MissionCommand;
    if (emergency_latched_) {
      if (msg->command_type != C::CMD_CLEAR_EMERGENCY_LATCH) {
        publish_status(
          forest_hybrid_msgs::msg::MissionStatus::STATE_FAILED,
          "EMERGENCY latched: no mission until CMD_CLEAR_EMERGENCY_LATCH (6) after safe manual reset");
        return;
      }
      std::string reason;
      if (!validate_command(*msg, reason)) {
        publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_FAILED, "invalid command: " + reason);
        return;
      }
      emergency_latched_ = false;
      active_mission_.reset();
      last_published_goal_key_.clear();
      force_goal_republish_ = false;
      active_map_goal_.reset();
      queue_.clear();
      publish_status(
        forest_hybrid_msgs::msg::MissionStatus::STATE_IDLE,
        "emergency latch cleared — system ready for new missions");
      return;
    }

    if (msg->command_type == C::CMD_CLEAR_EMERGENCY_LATCH) {
      publish_status(
        forest_hybrid_msgs::msg::MissionStatus::STATE_FAILED,
        "CMD_CLEAR_EMERGENCY_LATCH only valid after EMERGENCY_STOP latched the stack");
      return;
    }

    std::string reason;
    if (!validate_command(*msg, reason)) {
      publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_FAILED, "invalid command: " + reason);
      return;
    }
    enqueue_or_preempt(*msg);
  }

  void on_ack(const forest_hybrid_msgs::msg::MissionAck::SharedPtr msg)
  {
    if (!active_mission_.has_value()) {
      return;
    }
    if (!msg->approved) {
      publish_status(
        forest_hybrid_msgs::msg::MissionStatus::STATE_HOLDING,
        "ACK denied: " + msg->reason + " -> HOLD");
      active_mission_->command.command_type = forest_hybrid_msgs::msg::MissionCommand::CMD_HOLD;
      last_published_goal_key_.clear();
      active_map_goal_.reset();
      execute_active_command();
      return;
    }

    if (waiting_return_home_ack_) {
      waiting_return_home_ack_ = false;
      last_published_goal_key_.clear();
      execute_active_command();
      return;
    }
    if (waiting_aerial_permission_) {
      waiting_aerial_permission_ = false;
      last_published_goal_key_.clear();
      execute_active_command();
    }
  }

  void on_progress(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!active_mission_.has_value()) {
      return;
    }
    active_mission_->progress = std::clamp(msg->data, 0.0F, 1.0F);
    publish_status(current_state_, current_detail_);
  }

  void on_goal_reached(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!allow_goal_reached_shortcut_ || !active_mission_.has_value() || !msg->data) {
      return;
    }
    advance_after_goal_reached("goal_reached topic shortcut");
  }

  void advance_after_goal_reached(const std::string & detail)
  {
    if (!active_mission_.has_value()) {
      return;
    }
    using C = forest_hybrid_msgs::msg::MissionCommand;
    if (active_mission_->command.command_type == C::CMD_PATROL_WAYPOINTS) {
      complete_active(detail);
      return;
    }
    complete_active(detail);
  }

  bool check_pose_arrival(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!active_map_goal_.has_value()) {
      return false;
    }
    if (pose.header.frame_id != active_map_goal_->header.frame_id) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "pose frame_id '%s' != goal frame_id '%s' — arrival check skipped",
        pose.header.frame_id.c_str(), active_map_goal_->header.frame_id.c_str());
      return false;
    }
    const double gx = active_map_goal_->pose.position.x;
    const double gy = active_map_goal_->pose.position.y;
    const double rx = pose.pose.position.x;
    const double ry = pose.pose.position.y;
    // Planar arrival: ignore Z (terrain height comes from physics, not mission panel).
    const double dist = std::hypot(gx - rx, gy - ry);
    if (dist > goal_tolerance_m_) {
      return false;
    }

    const double yaw_robot = yaw_from_quaternion(
      pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    double yaw_des = yaw_from_quaternion(
      active_map_goal_->pose.orientation.x, active_map_goal_->pose.orientation.y,
      active_map_goal_->pose.orientation.z, active_map_goal_->pose.orientation.w);
    if (is_unspecified_orientation(active_map_goal_->pose.orientation)) {
      yaw_des = desired_heading_rad(gx, gy, pose.pose);
    }
    if (std::abs(shortest_angle_diff(yaw_robot, yaw_des)) > goal_tolerance_heading_rad_) {
      return false;
    }
    return true;
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_pose_ = *msg;
    if (!active_mission_.has_value()) {
      return;
    }
    using C = forest_hybrid_msgs::msg::MissionCommand;
    const auto t = active_mission_->command.command_type;
    if (
      t == C::CMD_HOLD || t == C::CMD_EMERGENCY_STOP || waiting_return_home_ack_ ||
      waiting_aerial_permission_)
    {
      return;
    }
    // Defer (re)publish for RELATIVE until we have pose
    if (active_mission_->command.frame_type == C::FRAME_RELATIVE && last_published_goal_key_.empty()) {
      execute_active_command();
    }
    if (!active_map_goal_.has_value()) {
      return;
    }
    if (check_pose_arrival(*msg)) {
      advance_after_goal_reached(
        "goal reached (planar XY within " + std::to_string(goal_tolerance_m_) +
        " m and heading within tolerance)");
    }
  }

  void on_path_blocked(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!active_mission_.has_value() || !msg->data) {
      return;
    }
    using C = forest_hybrid_msgs::msg::MissionCommand;
    const auto type = active_mission_->command.command_type;
    if (
      type == C::CMD_HOLD || type == C::CMD_EMERGENCY_STOP ||
      waiting_return_home_ack_ || waiting_aerial_permission_)
    {
      return;
    }

    active_mission_->retries_used++;
    if (active_mission_->retries_used <= static_cast<uint32_t>(max_replans_)) {
      publish_status(
        forest_hybrid_msgs::msg::MissionStatus::STATE_EXECUTING,
        "path blocked -> replanning attempt " + std::to_string(active_mission_->retries_used) + "/" +
        std::to_string(max_replans_));
      force_goal_republish_ = true;
      execute_active_command();
      return;
    }

    if (allow_auto_aerial_on_block_) {
      execute_active_command();
      return;
    }
    request_aerial_permission();
  }

  void on_tick()
  {
    if (!active_mission_.has_value() && !queue_.empty()) {
      pump_queue();
    }
  }

  void pump_queue()
  {
    if (queue_.empty()) {
      publish_status(forest_hybrid_msgs::msg::MissionStatus::STATE_IDLE, "queue empty");
      return;
    }
    const auto next = queue_.front();
    queue_.pop_front();
    activate_mission(next, "queue");
  }

  void publish_status(uint8_t state, const std::string & detail)
  {
    current_state_ = state;
    current_detail_ = detail;
    forest_hybrid_msgs::msg::MissionStatus msg;
    msg.state = state;
    msg.detail = detail;
    msg.progress = active_mission_.has_value() ? active_mission_->progress : 0.0F;
    msg.retries_used = active_mission_.has_value() ? active_mission_->retries_used : 0U;
    if (active_mission_.has_value()) {
      msg.active_command_id = active_mission_->command.command_id;
      msg.active_command_type = active_mission_->command.command_type;
    } else {
      msg.active_command_id = "";
      msg.active_command_type = forest_hybrid_msgs::msg::MissionCommand::CMD_HOLD;
    }
    status_pub_->publish(msg);
  }

  int64_t max_replans_{3};
  double goal_tolerance_m_{0.5};
  double goal_tolerance_heading_rad_{0.21};
  bool allow_auto_aerial_on_block_{false};
  bool waiting_return_home_ack_{false};
  bool waiting_aerial_permission_{false};
  bool force_goal_republish_{false};
  bool allow_goal_reached_shortcut_{false};
  std::string pose_topic_;
  std::string last_published_goal_key_;
  std::string current_detail_;

  uint8_t current_state_{forest_hybrid_msgs::msg::MissionStatus::STATE_IDLE};
  bool emergency_latched_{false};
  std::optional<RuntimeMission> active_mission_;
  std::deque<RuntimeMission> queue_;
  std::optional<geometry_msgs::msg::PoseStamped> last_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> active_map_goal_;

  rclcpp::Subscription<forest_hybrid_msgs::msg::MissionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::MissionAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blocked_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<forest_hybrid_msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace forest_planner_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_planner_ros2::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}

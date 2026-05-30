#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "forest_navigation_ros2/global_planner/global_planner.hpp"
#include "forest_navigation_ros2/local_planner/local_planner.hpp"
#include "forest_navigation_ros2/navigation_metrics.hpp"
#include "forest_navigation_ros2/navigation_viz.hpp"
#include "forest_navigation_ros2/tracking_controller/tracking_controller_interface.hpp"
#include "forest_navigation_ros2/trajectory_sampler/trajectory_sampler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace forest_navigation_ros2
{

namespace
{
double distance_xy(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}
}  // namespace

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode()
  : Node("navigation_node"),
    global_planner_(),
    local_planner_(),
    sampler_(0.25),
    viz_()
  {
    declare_parameter<std::string>("controller_type", "nmpc");
    declare_parameter<double>("nmpc_horizon_tf", 2.0);
    declare_parameter<int>("nmpc_horizon_n", 25);
    declare_parameter<std::string>("pose_topic", "/state/pose_fused");
    declare_parameter<std::string>("mission_goal_topic", "/planning/mission_goal");
    declare_parameter<std::string>("mission_route_topic", "/planning/mission_route");
    declare_parameter<std::string>("cmd_vel_topic", "/forest_gen/cmd_vel");
    declare_parameter<std::string>("metrics_csv_path", "/tmp/forest_navigation_metrics.csv");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("trajectory_step_m", 0.25);
    declare_parameter<double>("lookahead_m", 1.2);
    declare_parameter<double>("max_linear_vel", 0.5);
    declare_parameter<double>("max_angular_vel", 1.0);
    declare_parameter<double>("goal_tolerance_m", 0.35);
    declare_parameter<double>("waypoint_tolerance_m", 0.6);
    declare_parameter<double>("goal_heading_tolerance_rad", 0.35);
    declare_parameter<double>("approach_velocity_scaling_dist", 2.0);
    declare_parameter<double>("regulated_linear_scaling_min_radius", 0.9);
    declare_parameter<bool>("use_velocity_regulated_linear_scaling", true);
    declare_parameter<std::string>("map_frame", "map");

    const auto pose_topic = get_parameter("pose_topic").as_string();
    const auto goal_topic = get_parameter("mission_goal_topic").as_string();
    const auto route_topic = get_parameter("mission_route_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    waypoint_tolerance_m_ = std::max(0.01, get_parameter("waypoint_tolerance_m").as_double());

    sampler_.set_step_m(get_parameter("trajectory_step_m").as_double());

    const auto controller_type = get_parameter("controller_type").as_string();
    PurePursuitController pursuit(
      get_parameter("lookahead_m").as_double(),
      get_parameter("max_linear_vel").as_double(),
      get_parameter("max_angular_vel").as_double(),
      get_parameter("goal_tolerance_m").as_double(),
      get_parameter("goal_heading_tolerance_rad").as_double());
    pursuit.set_approach_velocity_scaling_dist(
      get_parameter("approach_velocity_scaling_dist").as_double());
    pursuit.set_regulated_linear_scaling_min_radius(
      get_parameter("regulated_linear_scaling_min_radius").as_double());
    pursuit.set_use_velocity_regulated_linear_scaling(
      get_parameter("use_velocity_regulated_linear_scaling").as_bool());

    NmpcSkidSteerController::Params nmpc_params;
    nmpc_params.max_linear_vel = get_parameter("max_linear_vel").as_double();
    nmpc_params.max_angular_vel = get_parameter("max_angular_vel").as_double();
    nmpc_params.goal_tolerance_m = get_parameter("goal_tolerance_m").as_double();
    nmpc_params.goal_heading_tolerance_rad =
      get_parameter("goal_heading_tolerance_rad").as_double();
    nmpc_params.horizon_tf = get_parameter("nmpc_horizon_tf").as_double();
    nmpc_params.horizon_n = get_parameter("nmpc_horizon_n").as_int();

    tracking_ = TrackingController(controller_type, std::move(pursuit), nmpc_params);

    metrics_ = std::make_unique<NavigationMetricsLogger>(
      get_parameter("metrics_csv_path").as_string());

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    pub_traj_ = create_publisher<nav_msgs::msg::Path>("/planning/local_trajectory", 10);
    pub_progress_ = create_publisher<std_msgs::msg::Float32>("/planning/progress", 10);
    pub_blocked_ = create_publisher<std_msgs::msg::Bool>("/planning/path_blocked", 10);
    pub_goal_reached_ = create_publisher<std_msgs::msg::Bool>("/planning/goal_reached", 10);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planning/debug/markers", 10);
    pub_sparse_ = create_publisher<nav_msgs::msg::Path>("/planning/debug/sparse_path", 10);
    pub_waypoints_ = create_publisher<nav_msgs::msg::Path>("/planning/debug/waypoints", 10);

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 20,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_ = *msg;
        const bool first_pose = !have_pose_;
        have_pose_ = true;
        if (first_pose && have_pending_goal_) {
          have_pending_goal_ = false;
          start_single_goal(pending_goal_);
        }
        if (first_pose && have_pending_route_) {
          have_pending_route_ = false;
          start_route(pending_route_);
        }
      });

    const auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, goal_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_mission_goal(*msg); });
    sub_route_ = create_subscription<nav_msgs::msg::Path>(
      route_topic, goal_qos,
      [this](const nav_msgs::msg::Path::SharedPtr msg) { on_mission_route(*msg); });

    const double hz = std::max(5.0, get_parameter("control_rate_hz").as_double());
    timer_ = create_timer(
      std::chrono::duration<double>(1.0 / hz),
      [this]() { control_tick(); });

    RCLCPP_INFO(
      get_logger(),
      "Navigation: goal=%s route=%s pose=%s → controller=%s%s",
      goal_topic.c_str(), route_topic.c_str(), pose_topic.c_str(),
      tracking_.active_type().c_str(),
      tracking_.using_fallback() ? " (fallback RPP)" : "");
  }

private:
  bool at_waypoint(const geometry_msgs::msg::PoseStamped & wp) const
  {
    return distance_xy(pose_.pose.position, wp.pose.position) < waypoint_tolerance_m_;
  }

  void init_waypoint_queue()
  {
    waypoint_queue_.clear();
    for (const auto & wp : route_waypoints_) {
      waypoint_queue_.push_back(wp);
    }
    total_waypoints_ = waypoint_queue_.size();
    advance_waypoint_queue(true);
  }

  void advance_waypoint_queue(bool allow_log = false)
  {
    while (!waypoint_queue_.empty() && at_waypoint(waypoint_queue_.front())) {
      const auto wp = waypoint_queue_.front();
      waypoint_queue_.pop_front();
      if (allow_log) {
        RCLCPP_INFO(
          get_logger(),
          "Waypoint (%.2f, %.2f) visitado — restam %zu/%zu",
          wp.pose.position.x, wp.pose.position.y,
          waypoint_queue_.size(), total_waypoints_);
      }
    }
    publish_remaining_waypoints();
  }

  void publish_remaining_waypoints()
  {
    nav_msgs::msg::Path wp_vis;
    wp_vis.header.frame_id = map_frame_;
    wp_vis.header.stamp = get_clock()->now();
    wp_vis.poses.assign(waypoint_queue_.begin(), waypoint_queue_.end());
    pub_waypoints_->publish(wp_vis);
  }

  float waypoint_progress(float path_progress) const
  {
    if (total_waypoints_ == 0) {
      return path_progress;
    }
    const float visited = static_cast<float>(total_waypoints_ - waypoint_queue_.size());
    return std::clamp(visited / static_cast<float>(total_waypoints_), 0.0f, 1.0f);
  }

  void on_mission_goal(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!have_pose_) {
      pending_goal_ = goal;
      have_pending_goal_ = true;
      RCLCPP_INFO(get_logger(), "Goal em fila até haver pose em %s", map_frame_.c_str());
      return;
    }
    start_single_goal(goal);
  }

  void on_mission_route(const nav_msgs::msg::Path & route)
  {
    if (route.poses.empty()) {
      RCLCPP_WARN(get_logger(), "mission_route vazio — ignorado");
      return;
    }
    if (!have_pose_) {
      pending_route_ = route;
      have_pending_route_ = true;
      RCLCPP_INFO(get_logger(), "Route (%zu WP) em fila até haver pose", route.poses.size());
      return;
    }
    start_route(route);
  }

  void start_single_goal(const geometry_msgs::msg::PoseStamped & goal)
  {
    geometry_msgs::msg::PoseStamped start = pose_;
    start.header.frame_id = map_frame_;
    geometry_msgs::msg::PoseStamped g = goal;
    if (g.header.frame_id.empty()) {
      g.header.frame_id = map_frame_;
    }

    route_waypoints_ = {g};
    begin_tracking(global_planner_.plan(start, g), g, "single");
  }

  void start_route(const nav_msgs::msg::Path & route)
  {
    geometry_msgs::msg::PoseStamped start = pose_;
    start.header.frame_id = map_frame_;

    route_waypoints_.clear();
    for (const auto & wp : route.poses) {
      geometry_msgs::msg::PoseStamped p = wp;
      if (p.header.frame_id.empty()) {
        p.header.frame_id = map_frame_;
      }
      route_waypoints_.push_back(p);
    }

    geometry_msgs::msg::PoseStamped final_goal = route_waypoints_.back();
    begin_tracking(global_planner_.plan_polyline(start, route_waypoints_), final_goal, "route");
  }

  void begin_tracking(
    const nav_msgs::msg::Path & sparse_in,
    const geometry_msgs::msg::PoseStamped & final_goal,
    const std::string & leg_kind)
  {
    sparse_path_ = local_planner_.refine(sparse_in);
    dense_path_ = sampler_.densify(sparse_path_);
    active_goal_ = final_goal;
    nav_active_ = dense_path_.poses.size() >= 2;
    trace_.clear();
    leg_id_ = leg_kind + "_" + std::to_string(final_goal.header.stamp.sec);
    metrics_->start_leg(leg_id_);
    tracking_.reset();
    init_waypoint_queue();

    pub_traj_->publish(dense_path_);
    pub_sparse_->publish(sparse_path_);

    std_msgs::msg::Bool not_blocked;
    not_blocked.data = false;
    pub_blocked_->publish(not_blocked);
    leg_start_ = get_clock()->now();

    RCLCPP_INFO(
      get_logger(),
      "Tracking %s: %zu vértices esparsos → %zu densos, alvo final (%.2f, %.2f), "
      "%zu WP na fila",
      leg_kind.c_str(), sparse_path_.poses.size(), dense_path_.poses.size(),
      final_goal.pose.position.x, final_goal.pose.position.y,
      waypoint_queue_.size());
  }

  void control_tick()
  {
    if (have_pending_goal_ && have_pose_) {
      have_pending_goal_ = false;
      start_single_goal(pending_goal_);
    }
    if (have_pending_route_ && have_pose_) {
      have_pending_route_ = false;
      start_route(pending_route_);
    }

    if (!nav_active_ || !have_pose_ || dense_path_.poses.empty()) {
      return;
    }

    advance_waypoint_queue();

    const auto cmd = tracking_.compute(pose_, dense_path_);
    const double t_sec = (get_clock()->now() - leg_start_).seconds();

    trace_.push_back(pose_.pose.position);

    std_msgs::msg::Float32 progress;
    progress.data = waypoint_progress(static_cast<float>(cmd.progress_along_path));
    pub_progress_->publish(progress);
    metrics_->log_sample(t_sec, pose_, cmd);

    auto markers = viz_.make_markers(
      map_frame_, get_clock()->now(), pose_, active_goal_, sparse_path_, dense_path_,
      route_waypoints_, cmd.lookahead_point, trace_);
    pub_markers_->publish(markers);

    geometry_msgs::msg::Twist twist;
    const bool all_waypoints_visited = waypoint_queue_.empty();
    if (cmd.goal_reached && all_waypoints_visited) {
      nav_active_ = false;
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      pub_cmd_->publish(twist);

      std_msgs::msg::Bool gr;
      gr.data = true;
      pub_goal_reached_->publish(gr);
      progress.data = 1.0f;
      pub_progress_->publish(progress);
      metrics_->end_leg();
      RCLCPP_INFO(
        get_logger(),
        "Trajetória concluída em %.1f s (dist final %.3f m, %zu WP visitados)",
        t_sec, cmd.goal_distance, total_waypoints_);
      return;
    }

    twist.linear.x = cmd.linear_x;
    twist.angular.z = cmd.angular_z;
    pub_cmd_->publish(twist);
  }

  GlobalPlanner global_planner_;
  LocalPlanner local_planner_;
  TrajectorySampler sampler_;
  TrackingController tracking_{"pure_pursuit", PurePursuitController(1.2, 0.5, 1.0, 0.35, 0.35), {}};
  NavigationViz viz_;
  std::unique_ptr<NavigationMetricsLogger> metrics_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_sparse_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_waypoints_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_progress_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_blocked_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_route_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::PoseStamped active_goal_;
  geometry_msgs::msg::PoseStamped pending_goal_;
  nav_msgs::msg::Path pending_route_;
  nav_msgs::msg::Path sparse_path_;
  nav_msgs::msg::Path dense_path_;
  std::vector<geometry_msgs::msg::PoseStamped> route_waypoints_;
  std::deque<geometry_msgs::msg::PoseStamped> waypoint_queue_;
  std::vector<geometry_msgs::msg::Point> trace_;

  bool have_pose_{false};
  bool have_pending_goal_{false};
  bool have_pending_route_{false};
  bool nav_active_{false};
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string leg_id_;
  size_t total_waypoints_{0};
  double waypoint_tolerance_m_{0.6};
  rclcpp::Time leg_start_{0, 0, RCL_ROS_TIME};
};

}  // namespace forest_navigation_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_navigation_ros2::NavigationNode>());
  rclcpp::shutdown();
  return 0;
}

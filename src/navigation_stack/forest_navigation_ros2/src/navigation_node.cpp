#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "forest_navigation_ros2/global_planner/global_planner.hpp"
#include "forest_navigation_ros2/local_planner/local_planner.hpp"
#include "forest_navigation_ros2/navigation_metrics.hpp"
#include "forest_navigation_ros2/navigation_viz.hpp"
#include "forest_navigation_ros2/tracking_controller/pure_pursuit.hpp"
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

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode()
  : Node("navigation_node"),
    global_planner_(),
    local_planner_(),
    sampler_(0.25),
    pursuit_(1.0, 0.5, 1.0, 0.05, 0.15),
    viz_()
  {
    declare_parameter<std::string>("pose_topic", "/state/pose_fused");
    declare_parameter<std::string>("mission_goal_topic", "/planning/mission_goal");
    declare_parameter<std::string>("cmd_vel_topic", "/forest_gen/cmd_vel");
    declare_parameter<std::string>("metrics_csv_path", "/tmp/forest_navigation_metrics.csv");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("trajectory_step_m", 0.25);
    declare_parameter<double>("lookahead_m", 1.0);
    declare_parameter<double>("max_linear_vel", 0.35);
    declare_parameter<double>("max_angular_vel", 0.8);
    declare_parameter<double>("goal_tolerance_m", 0.05);
    declare_parameter<double>("goal_heading_tolerance_rad", 0.15);
    declare_parameter<double>("approach_radius_m", 0.6);
    declare_parameter<std::string>("map_frame", "map");

    const auto pose_topic = get_parameter("pose_topic").as_string();
    const auto goal_topic = get_parameter("mission_goal_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();

    sampler_.set_step_m(get_parameter("trajectory_step_m").as_double());
    pursuit_.set_lookahead_m(get_parameter("lookahead_m").as_double());
    pursuit_.set_max_linear_vel(get_parameter("max_linear_vel").as_double());
    pursuit_.set_max_angular_vel(get_parameter("max_angular_vel").as_double());
    pursuit_.set_goal_tolerance_m(get_parameter("goal_tolerance_m").as_double());
    pursuit_.set_goal_heading_tolerance_rad(get_parameter("goal_heading_tolerance_rad").as_double());
    pursuit_.set_approach_radius_m(get_parameter("approach_radius_m").as_double());

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

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 20,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_ = *msg;
        const bool first_pose = !have_pose_;
        have_pose_ = true;
        if (first_pose && have_pending_goal_) {
          have_pending_goal_ = false;
          start_leg(pending_goal_);
        }
      });

    const auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, goal_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_mission_goal(*msg); });

    const double hz = std::max(5.0, get_parameter("control_rate_hz").as_double());
    const auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = create_timer(
      period,
      [this]() { control_tick(); });

    RCLCPP_INFO(
      get_logger(),
      "Navigation MVP: goal=%s pose=%s cmd=%s (Pure Pursuit + trajectory sampler)",
      goal_topic.c_str(), pose_topic.c_str(), cmd_vel_topic_.c_str());
  }

private:
  void on_mission_goal(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!have_pose_) {
      pending_goal_ = goal;
      have_pending_goal_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Mission goal recebido — à espera de pose em %s (goal em fila)",
        map_frame_.c_str());
      return;
    }
    start_leg(goal);
  }

  void start_leg(const geometry_msgs::msg::PoseStamped & goal)
  {
    geometry_msgs::msg::PoseStamped start = pose_;
    start.header.frame_id = map_frame_;
    geometry_msgs::msg::PoseStamped g = goal;
    if (g.header.frame_id.empty()) {
      g.header.frame_id = map_frame_;
    }

    sparse_path_ = global_planner_.plan(start, g);
    sparse_path_ = local_planner_.refine(sparse_path_);
    dense_path_ = sampler_.densify(sparse_path_);
    active_goal_ = g;
    tracking_ = !dense_path_.poses.empty();
    trace_.clear();
    leg_id_ = std::to_string(goal.header.stamp.sec) + "_" + std::to_string(goal.header.stamp.nanosec);
    metrics_->start_leg(leg_id_);

    pub_traj_->publish(dense_path_);
    pub_sparse_->publish(sparse_path_);
    std_msgs::msg::Bool not_blocked;
    not_blocked.data = false;
    pub_blocked_->publish(not_blocked);

    leg_start_ = get_clock()->now();

    RCLCPP_INFO(
      get_logger(), "Nova perna: %zu pontos densos, goal (%.2f, %.2f)",
      dense_path_.poses.size(), g.pose.position.x, g.pose.position.y);
  }

  void control_tick()
  {
    if (have_pending_goal_ && have_pose_) {
      have_pending_goal_ = false;
      start_leg(pending_goal_);
    }

    if (!tracking_ || !have_pose_ || dense_path_.poses.empty()) {
      return;
    }

    const auto cmd = pursuit_.compute(pose_, dense_path_);
    const double t_sec = (get_clock()->now() - leg_start_).seconds();

    trace_.push_back(pose_.pose.position);

    std_msgs::msg::Float32 progress;
    progress.data = static_cast<float>(cmd.progress_along_path);
    pub_progress_->publish(progress);

    metrics_->log_sample(t_sec, pose_, cmd);

    auto markers = viz_.make_markers(
      map_frame_, get_clock()->now(), pose_, active_goal_, sparse_path_, dense_path_,
      cmd.lookahead_point, trace_);
    pub_markers_->publish(markers);

    geometry_msgs::msg::Twist twist;
    if (cmd.goal_reached) {
      tracking_ = false;
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      pub_cmd_->publish(twist);

      std_msgs::msg::Bool gr;
      gr.data = true;
      pub_goal_reached_->publish(gr);
      progress.data = 1.0f;
      pub_progress_->publish(progress);
      metrics_->end_leg();
      RCLCPP_INFO(get_logger(), "Perna concluída (Pure Pursuit goal_reached) em %.1f s", t_sec);
      return;
    }

    twist.linear.x = cmd.linear_x;
    twist.angular.z = cmd.angular_z;
    pub_cmd_->publish(twist);
  }

  GlobalPlanner global_planner_;
  LocalPlanner local_planner_;
  TrajectorySampler sampler_;
  PurePursuitController pursuit_;
  NavigationViz viz_;
  std::unique_ptr<NavigationMetricsLogger> metrics_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_sparse_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_progress_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_blocked_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::PoseStamped active_goal_;
  geometry_msgs::msg::PoseStamped pending_goal_;
  nav_msgs::msg::Path sparse_path_;
  nav_msgs::msg::Path dense_path_;
  std::vector<geometry_msgs::msg::Point> trace_;

  bool have_pose_{false};
  bool have_pending_goal_{false};
  bool tracking_{false};
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string leg_id_;
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

#include <algorithm>
#include <cmath>
#include <limits>

#include "forest_navigation_ros2/tracking_controller/pure_pursuit.hpp"

namespace forest_navigation_ros2
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalize_angle(double a)
{
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a < -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

double path_length(const nav_msgs::msg::Path & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    const double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    len += std::hypot(dx, dy);
  }
  return len;
}

double distance_xy(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}
}  // namespace

PurePursuitController::PurePursuitController(
  double lookahead_m,
  double max_linear_vel,
  double max_angular_vel,
  double goal_tolerance_m,
  double goal_heading_tolerance_rad)
: lookahead_m_(std::max(0.1, lookahead_m)),
  max_linear_vel_(std::max(0.05, max_linear_vel)),
  max_angular_vel_(std::max(0.05, max_angular_vel)),
  goal_tolerance_m_(std::max(0.01, goal_tolerance_m)),
  goal_heading_tolerance_rad_(std::max(0.01, goal_heading_tolerance_rad)),
  approach_radius_m_(0.6),
  approach_linear_gain_(0.9),
  approach_min_vel_(0.04)
{
}

void PurePursuitController::set_goal_tolerance_m(double v)
{
  goal_tolerance_m_ = std::max(0.01, v);
}

void PurePursuitController::set_goal_heading_tolerance_rad(double v)
{
  goal_heading_tolerance_rad_ = std::max(0.01, v);
}

void PurePursuitController::set_approach_radius_m(double v)
{
  approach_radius_m_ = std::max(goal_tolerance_m_ * 2.0, v);
}

void PurePursuitController::set_lookahead_m(double v) { lookahead_m_ = std::max(0.1, v); }
void PurePursuitController::set_max_linear_vel(double v) { max_linear_vel_ = std::max(0.05, v); }
void PurePursuitController::set_max_angular_vel(double v) { max_angular_vel_ = std::max(0.05, v); }

PursuitCommand PurePursuitController::compute(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path) const
{
  PursuitCommand out;
  if (path.poses.empty()) {
    return out;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = yaw_from_quat(
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);

  const auto & goal_pt = path.poses.back().pose.position;
  out.goal_distance = distance_xy(pose.pose.position, goal_pt);

  const double goal_yaw = yaw_from_quat(
    path.poses.back().pose.orientation.x,
    path.poses.back().pose.orientation.y,
    path.poses.back().pose.orientation.z,
    path.poses.back().pose.orientation.w);
  out.heading_error = normalize_angle(goal_yaw - yaw);

  if (out.goal_distance < goal_tolerance_m_ &&
    std::abs(out.heading_error) < goal_heading_tolerance_rad_)
  {
    out.goal_reached = true;
    out.progress_along_path = 1.0;
    return out;
  }

  // Fase terminal: homing proporcional à distância (evita parar a ~0.5 m do goal).
  if (out.goal_distance < approach_radius_m_) {
    const double bearing = std::atan2(goal_pt.y - y, goal_pt.x - x);
    const double steer_err = normalize_angle(bearing - yaw);
    out.lookahead_point = goal_pt;

    double v = std::min(max_linear_vel_, approach_linear_gain_ * out.goal_distance);
    if (out.goal_distance > goal_tolerance_m_ * 2.0) {
      v = std::max(approach_min_vel_, v);
    } else {
      v = std::min(v, max_linear_vel_ * 0.25);
    }

    double w = 2.0 * steer_err;
    if (out.goal_distance < goal_tolerance_m_ * 4.0) {
      w = 0.6 * steer_err + 0.4 * out.heading_error;
    }
    w = std::clamp(w, -max_angular_vel_, max_angular_vel_);

    out.linear_x = v;
    out.angular_z = w;
    out.progress_along_path = std::clamp(
      1.0 - out.goal_distance / std::max(approach_radius_m_, 1e-3), 0.0, 0.99);
    return out;
  }

  size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double d = distance_xy(pose.pose.position, path.poses[i].pose.position);
    if (d < closest_dist) {
      closest_dist = d;
      closest_idx = i;
    }
  }
  out.lateral_error = closest_dist;

  double acc = 0.0;
  size_t lookahead_idx = closest_idx;
  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    if (i > closest_idx) {
      const double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
      const double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
      acc += std::hypot(dx, dy);
    }
    lookahead_idx = i;
    if (acc >= lookahead_m_) {
      break;
    }
  }

  const auto & lp = path.poses[lookahead_idx].pose.position;
  out.lookahead_point = lp;

  const double alpha = normalize_angle(std::atan2(lp.y - y, lp.x - x) - yaw);
  const double ld = std::max(lookahead_m_, 0.1);
  const double curvature = 2.0 * std::sin(alpha) / ld;

  double v = max_linear_vel_;
  if (out.goal_distance < 2.0 * lookahead_m_) {
    v = std::min(v, max_linear_vel_ * (out.goal_distance / (2.0 * lookahead_m_)));
  }
  v = std::max(0.05, v);

  double w = v * curvature;
  w = std::clamp(w, -max_angular_vel_, max_angular_vel_);

  out.linear_x = v;
  out.angular_z = w;

  const double total = std::max(1e-3, path_length(path));
  double traveled = 0.0;
  for (size_t i = 1; i <= closest_idx && i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    const double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    traveled += std::hypot(dx, dy);
  }
  out.progress_along_path = std::clamp(traveled / total, 0.0, 1.0);

  return out;
}

}  // namespace forest_navigation_ros2

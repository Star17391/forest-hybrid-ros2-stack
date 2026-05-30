#include <algorithm>
#include <cmath>
#include <limits>

#include "forest_navigation_ros2/tracking_controller/pure_pursuit.hpp"

namespace forest_navigation_ros2
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinCarrotDist = 0.05;

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

double distance_xy(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double path_length(const nav_msgs::msg::Path & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    len += distance_xy(
      path.poses[i - 1].pose.position,
      path.poses[i].pose.position);
  }
  return len;
}

double remaining_path_length(const nav_msgs::msg::Path & path, size_t from_idx)
{
  double len = 0.0;
  for (size_t i = from_idx + 1; i < path.poses.size(); ++i) {
    len += distance_xy(
      path.poses[i - 1].pose.position,
      path.poses[i].pose.position);
  }
  return len;
}

struct SegmentProjection
{
  double distance;
  size_t segment_start_idx;
  double t;
};

SegmentProjection project_on_path(
  double px, double py,
  const nav_msgs::msg::Path & path,
  size_t start_idx)
{
  SegmentProjection best;
  best.distance = std::numeric_limits<double>::max();
  best.segment_start_idx = start_idx;
  best.t = 0.0;

  for (size_t i = start_idx; i + 1 < path.poses.size(); ++i) {
    const double ax = path.poses[i].pose.position.x;
    const double ay = path.poses[i].pose.position.y;
    const double bx = path.poses[i + 1].pose.position.x;
    const double by = path.poses[i + 1].pose.position.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double seg_len_sq = dx * dx + dy * dy;

    double t;
    if (seg_len_sq < 1e-12) {
      t = 0.0;
    } else {
      t = std::clamp(((px - ax) * dx + (py - ay) * dy) / seg_len_sq, 0.0, 1.0);
    }

    const double proj_x = ax + t * dx;
    const double proj_y = ay + t * dy;
    const double dist = std::hypot(px - proj_x, py - proj_y);

    if (dist < best.distance) {
      best.distance = dist;
      best.segment_start_idx = i;
      best.t = t;
    }
  }

  if (best.distance == std::numeric_limits<double>::max() && !path.poses.empty()) {
    const double d = std::hypot(
      px - path.poses[start_idx].pose.position.x,
      py - path.poses[start_idx].pose.position.y);
    best.distance = d;
    best.segment_start_idx = start_idx;
    best.t = 0.0;
  }

  return best;
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
  approach_velocity_scaling_dist_(std::max(lookahead_m_, 2.0 * lookahead_m_)),
  regulated_linear_scaling_min_radius_(std::max(0.1, lookahead_m_))
{
}

void PurePursuitController::reset()
{
  last_closest_idx_ = 0;
}

void PurePursuitController::set_goal_tolerance_m(double v)
{
  goal_tolerance_m_ = std::max(0.01, v);
}

void PurePursuitController::set_goal_heading_tolerance_rad(double v)
{
  goal_heading_tolerance_rad_ = std::max(0.01, v);
}

void PurePursuitController::set_approach_velocity_scaling_dist(double v)
{
  approach_velocity_scaling_dist_ = std::max(lookahead_m_, v);
}

void PurePursuitController::set_regulated_linear_scaling_min_radius(double v)
{
  regulated_linear_scaling_min_radius_ = std::max(0.05, v);
}

void PurePursuitController::set_use_velocity_regulated_linear_scaling(bool v)
{
  use_velocity_regulated_linear_scaling_ = v;
}

void PurePursuitController::set_lookahead_m(double v)
{
  lookahead_m_ = std::max(0.1, v);
}

void PurePursuitController::set_max_linear_vel(double v)
{
  max_linear_vel_ = std::max(0.05, v);
}

void PurePursuitController::set_max_angular_vel(double v)
{
  max_angular_vel_ = std::max(0.05, v);
}

PursuitCommand PurePursuitController::compute(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path)
{
  PursuitCommand out;
  if (path.poses.size() < 2) {
    return out;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = yaw_from_quat(
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);

  const auto & goal_pose = path.poses.back();
  const auto & goal_pt = goal_pose.pose.position;
  out.goal_distance = distance_xy(pose.pose.position, goal_pt);

  const double goal_yaw = yaw_from_quat(
    goal_pose.pose.orientation.x,
    goal_pose.pose.orientation.y,
    goal_pose.pose.orientation.z,
    goal_pose.pose.orientation.w);
  out.heading_error = normalize_angle(goal_yaw - yaw);

  if (last_closest_idx_ >= path.poses.size()) {
    last_closest_idx_ = 0;
  }

  const auto proj = project_on_path(x, y, path, last_closest_idx_);
  size_t closest_idx = proj.segment_start_idx;
  if (proj.t > 0.5 && closest_idx + 1 < path.poses.size()) {
    closest_idx = proj.segment_start_idx + 1;
  }
  last_closest_idx_ = closest_idx;
  out.lateral_error = proj.distance;
  out.remaining_path_m = remaining_path_length(path, closest_idx);

  if (out.goal_distance < goal_tolerance_m_ &&
    std::abs(out.heading_error) < goal_heading_tolerance_rad_)
  {
    out.goal_reached = true;
    out.progress_along_path = 1.0;
    out.remaining_path_m = 0.0;
    return out;
  }

  // Lookahead along path arc from closest point.
  double arc = 0.0;
  size_t carrot_idx = closest_idx;
  for (size_t i = closest_idx + 1; i < path.poses.size(); ++i) {
    arc += distance_xy(
      path.poses[i - 1].pose.position,
      path.poses[i].pose.position);
    carrot_idx = i;
    if (arc >= lookahead_m_) {
      break;
    }
  }

  const auto & carrot = path.poses[carrot_idx].pose.position;
  out.lookahead_point = carrot;

  const double carrot_dist = std::max(
    distance_xy(pose.pose.position, carrot),
    kMinCarrotDist);

  const double alpha = normalize_angle(std::atan2(carrot.y - y, carrot.x - x) - yaw);
  const double curvature = 2.0 * std::sin(alpha) / carrot_dist;

  // --- Velocity regulation (Nav2 RPP style) ---
  double v = max_linear_vel_;

  if (use_velocity_regulated_linear_scaling_ && std::abs(curvature) > 1e-6) {
    const double radius = 1.0 / std::abs(curvature);
    if (radius < regulated_linear_scaling_min_radius_) {
      v = max_linear_vel_ * (radius / regulated_linear_scaling_min_radius_);
    }
  }

  if (approach_velocity_scaling_dist_ > 1e-6) {
    const double approach_scale = std::clamp(
      out.remaining_path_m / approach_velocity_scaling_dist_,
      0.0, 1.0);
    v = std::min(v, max_linear_vel_ * approach_scale);
  }

  if (std::abs(curvature) > 1e-6) {
    v = std::min(v, max_angular_vel_ / std::abs(curvature));
  }

  v = std::clamp(v, 0.0, max_linear_vel_);

  double w = v * curvature;
  w = std::clamp(w, -max_angular_vel_, max_angular_vel_);

  out.linear_x = v;
  out.angular_z = w;

  const double total = std::max(1e-3, path_length(path));
  double traveled = 0.0;
  for (size_t i = 1; i <= closest_idx && i < path.poses.size(); ++i) {
    traveled += distance_xy(
      path.poses[i - 1].pose.position,
      path.poses[i].pose.position);
  }
  out.progress_along_path = std::clamp(traveled / total, 0.0, 1.0);

  return out;
}

}  // namespace forest_navigation_ros2

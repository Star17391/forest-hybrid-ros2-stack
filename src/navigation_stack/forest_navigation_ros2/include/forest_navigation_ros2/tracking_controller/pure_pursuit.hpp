#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace forest_navigation_ros2
{

struct PursuitCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
  geometry_msgs::msg::Point lookahead_point;
  double lateral_error{0.0};
  double heading_error{0.0};
  double goal_distance{0.0};
  double progress_along_path{0.0};
  bool goal_reached{false};
};

/// Seguidor geométrico Pure Pursuit sobre path densificado.
class PurePursuitController
{
public:
  PurePursuitController(
    double lookahead_m,
    double max_linear_vel,
    double max_angular_vel,
    double goal_tolerance_m,
    double goal_heading_tolerance_rad);

  PursuitCommand compute(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path) const;

  void set_lookahead_m(double v);
  void set_max_linear_vel(double v);
  void set_max_angular_vel(double v);
  void set_goal_tolerance_m(double v);
  void set_goal_heading_tolerance_rad(double v);
  void set_approach_radius_m(double v);

private:
  double lookahead_m_;
  double max_linear_vel_;
  double max_angular_vel_;
  double goal_tolerance_m_;
  double goal_heading_tolerance_rad_;
  double approach_radius_m_;
  double approach_linear_gain_;
  double approach_min_vel_;
};

}  // namespace forest_navigation_ros2

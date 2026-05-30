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
  double remaining_path_m{0.0};
  double progress_along_path{0.0};
  bool goal_reached{false};
};

/// Regulated Pure Pursuit (unicycle / skid-steer).
///
/// Geometria clássica (Coulter 1992): κ = 2 sin(α) / L_d, ω = v κ.
/// Regulação de velocidade (Nav2 RPP): reduz v com curvatura e com distância
/// ao fim do path. Sem modos paralelos nem controlo P improvisado no fim.
class PurePursuitController
{
public:
  PurePursuitController(
    double lookahead_m,
    double max_linear_vel,
    double max_angular_vel,
    double goal_tolerance_m,
    double goal_heading_tolerance_rad);

  /// Reinicia o índice monótono no path (nova perna).
  void reset();

  PursuitCommand compute(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path);

  void set_lookahead_m(double v);
  void set_max_linear_vel(double v);
  void set_max_angular_vel(double v);
  void set_goal_tolerance_m(double v);
  void set_goal_heading_tolerance_rad(double v);
  void set_approach_velocity_scaling_dist(double v);
  void set_regulated_linear_scaling_min_radius(double v);
  void set_use_velocity_regulated_linear_scaling(bool v);

private:
  double lookahead_m_;
  double max_linear_vel_;
  double max_angular_vel_;
  double goal_tolerance_m_;
  double goal_heading_tolerance_rad_;
  double approach_velocity_scaling_dist_;
  double regulated_linear_scaling_min_radius_;
  bool use_velocity_regulated_linear_scaling_{true};
  size_t last_closest_idx_{0};
};

}  // namespace forest_navigation_ros2

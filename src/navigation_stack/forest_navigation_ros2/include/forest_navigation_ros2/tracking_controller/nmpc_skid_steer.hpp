#pragma once

#include <memory>
#include <optional>
#include <string>

#include "forest_navigation_ros2/tracking_controller/pure_pursuit.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace forest_navigation_ros2
{

/// Local NMPC (acados) — unicycle cmd_vel tracking on dense_path.
class NmpcSkidSteerController
{
public:
  struct Params
  {
    double max_linear_vel{0.5};
    double max_angular_vel{1.0};
    double goal_tolerance_m{0.35};
    double goal_heading_tolerance_rad{0.35};
    double horizon_tf{2.0};
    int horizon_n{25};
  };

  explicit NmpcSkidSteerController(const Params & params);
  ~NmpcSkidSteerController();

  NmpcSkidSteerController(const NmpcSkidSteerController &) = delete;
  NmpcSkidSteerController & operator=(const NmpcSkidSteerController &) = delete;

  bool is_ready() const;
  std::string last_error() const;

  void reset();

  /// Same contract as PurePursuitController (cmd_vel + progress metrics).
  PursuitCommand compute(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path);

  double last_solve_ms() const { return last_solve_ms_; }
  int last_acados_status() const { return last_acados_status_; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Params params_;
  PurePursuitController fallback_;
  double last_solve_ms_{0.0};
  int last_acados_status_{0};
  std::string last_error_;
  size_t last_closest_idx_{0};
  bool have_prev_u_{false};
  double prev_v_{0.0};
  double prev_w_{0.0};
};

}  // namespace forest_navigation_ros2

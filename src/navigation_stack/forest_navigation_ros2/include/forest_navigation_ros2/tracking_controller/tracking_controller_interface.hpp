#pragma once

#include <memory>
#include <string>

#include "forest_navigation_ros2/tracking_controller/nmpc_skid_steer.hpp"
#include "forest_navigation_ros2/tracking_controller/pure_pursuit.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace forest_navigation_ros2
{

/// Select Pure Pursuit or NMPC local controller.
class TrackingController
{
public:
  explicit TrackingController(
    const std::string & controller_type,
    PurePursuitController pursuit,
    NmpcSkidSteerController::Params nmpc_params);

  void reset();

  PursuitCommand compute(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path);

  std::string active_type() const { return active_type_; }
  bool using_fallback() const { return using_fallback_; }
  double last_solve_ms() const;

private:
  std::string active_type_;
  PurePursuitController pursuit_;
  std::unique_ptr<NmpcSkidSteerController> nmpc_;
  bool using_fallback_{false};
};

}  // namespace forest_navigation_ros2

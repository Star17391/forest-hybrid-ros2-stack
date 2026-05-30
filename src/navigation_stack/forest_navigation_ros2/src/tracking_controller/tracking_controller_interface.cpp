#include "forest_navigation_ros2/tracking_controller/tracking_controller_interface.hpp"

#include <algorithm>

namespace forest_navigation_ros2
{

TrackingController::TrackingController(
  const std::string & controller_type,
  PurePursuitController pursuit,
  NmpcSkidSteerController::Params nmpc_params)
: active_type_(controller_type),
  pursuit_(std::move(pursuit))
{
  std::string lowered = controller_type;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), ::tolower);

  if (lowered == "nmpc") {
    nmpc_ = std::make_unique<NmpcSkidSteerController>(nmpc_params);
    if (!nmpc_->is_ready()) {
      using_fallback_ = true;
    }
  } else {
    active_type_ = "pure_pursuit";
  }
}

void TrackingController::reset()
{
  pursuit_.reset();
  if (nmpc_) {
    nmpc_->reset();
  }
  using_fallback_ = false;
  if (nmpc_ && !nmpc_->is_ready()) {
    using_fallback_ = true;
  }
}

PursuitCommand TrackingController::compute(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path)
{
  if (nmpc_ && nmpc_->is_ready()) {
    using_fallback_ = false;
    return nmpc_->compute(pose, path);
  }
  using_fallback_ = true;
  return pursuit_.compute(pose, path);
}

double TrackingController::last_solve_ms() const
{
  return nmpc_ ? nmpc_->last_solve_ms() : 0.0;
}

}  // namespace forest_navigation_ros2

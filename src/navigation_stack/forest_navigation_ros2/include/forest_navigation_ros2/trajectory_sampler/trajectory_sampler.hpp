#pragma once

#include "nav_msgs/msg/path.hpp"

namespace forest_navigation_ros2
{

/// Densifica polilinhas para o Pure Pursuit (interpolação linear no MVP).
class TrajectorySampler
{
public:
  explicit TrajectorySampler(double step_m = 0.25);

  nav_msgs::msg::Path densify(const nav_msgs::msg::Path & sparse_path) const;

  void set_step_m(double step_m);

private:
  double step_m_;
};

}  // namespace forest_navigation_ros2

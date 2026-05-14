#pragma once

#include "nav_msgs/msg/path.hpp"

namespace forest_navigation_ros2
{

/// MVP: pass-through (noop). Futuro: costmap / obstacle avoidance.
class LocalPlanner
{
public:
  nav_msgs::msg::Path refine(const nav_msgs::msg::Path & global_path) const;
};

}  // namespace forest_navigation_ros2

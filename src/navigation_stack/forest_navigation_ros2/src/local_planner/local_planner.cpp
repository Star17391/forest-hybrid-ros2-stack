#include "forest_navigation_ros2/local_planner/local_planner.hpp"

namespace forest_navigation_ros2
{

nav_msgs::msg::Path LocalPlanner::refine(const nav_msgs::msg::Path & global_path) const
{
  return global_path;
}

}  // namespace forest_navigation_ros2

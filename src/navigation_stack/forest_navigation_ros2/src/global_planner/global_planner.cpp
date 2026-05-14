#include "forest_navigation_ros2/global_planner/global_planner.hpp"

namespace forest_navigation_ros2
{

namespace
{
geometry_msgs::msg::PoseStamped make_pose(
  const geometry_msgs::msg::PoseStamped & ref,
  double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped p = ref;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.0;
  p.pose.orientation.z = 0.0;
  p.pose.orientation.w = 1.0;
  return p;
}
}  // namespace

nav_msgs::msg::Path GlobalPlanner::plan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header = goal.header;
  path.poses.push_back(start);
  path.poses.push_back(goal);
  return path;
}

nav_msgs::msg::Path GlobalPlanner::plan_through_waypoints(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header = goal.header;
  path.poses.push_back(start);
  for (const auto & wp : waypoints) {
    path.poses.push_back(wp);
  }
  path.poses.push_back(goal);
  return path;
}

}  // namespace forest_navigation_ros2

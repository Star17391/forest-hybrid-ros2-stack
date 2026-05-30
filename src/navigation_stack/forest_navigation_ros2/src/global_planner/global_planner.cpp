#include <cmath>

#include "forest_navigation_ros2/global_planner/global_planner.hpp"
#include "std_msgs/msg/header.hpp"

namespace forest_navigation_ros2
{

namespace
{
void quaternion_from_yaw(double yaw, geometry_msgs::msg::Quaternion * q)
{
  q->x = 0.0;
  q->y = 0.0;
  q->z = std::sin(yaw * 0.5);
  q->w = std::cos(yaw * 0.5);
}

geometry_msgs::msg::PoseStamped make_pose(
  const std_msgs::msg::Header & header,
  double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header = header;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  quaternion_from_yaw(yaw, &p.pose.orientation);
  return p;
}

double segment_yaw(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  if (dx * dx + dy * dy < 1e-12) {
    return 0.0;
  }
  return std::atan2(dy, dx);
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

nav_msgs::msg::Path GlobalPlanner::plan_polyline(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<geometry_msgs::msg::PoseStamped> & via_points) const
{
  nav_msgs::msg::Path path;
  if (via_points.empty()) {
    return path;
  }

  path.header = via_points.front().header;
  if (path.header.frame_id.empty()) {
    path.header.frame_id = start.header.frame_id;
  }

  std::vector<geometry_msgs::msg::Point> pts;
  pts.push_back(start.pose.position);
  for (const auto & wp : via_points) {
    pts.push_back(wp.pose.position);
  }

  path.poses.push_back(start);
  for (size_t i = 1; i < pts.size(); ++i) {
    const double yaw = (i + 1 < pts.size()) ?
      segment_yaw(pts[i], pts[i + 1]) :
      segment_yaw(pts[i - 1], pts[i]);
    path.poses.push_back(make_pose(path.header, pts[i].x, pts[i].y, pts[i].z, yaw));
  }

  return path;
}

}  // namespace forest_navigation_ros2

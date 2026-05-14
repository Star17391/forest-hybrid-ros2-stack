#include "forest_navigation_ros2/navigation_viz.hpp"

#include "visualization_msgs/msg/marker.hpp"

namespace forest_navigation_ros2
{

namespace
{
visualization_msgs::msg::Marker make_marker_base(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  int id,
  int type)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "forest_navigation";
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.12;
  m.scale.y = 0.12;
  m.scale.z = 0.12;
  return m;
}
}  // namespace

visualization_msgs::msg::MarkerArray NavigationViz::make_markers(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & sparse_path,
  const nav_msgs::msg::Path & dense_path,
  const geometry_msgs::msg::Point & lookahead,
  const std::vector<geometry_msgs::msg::Point> & trace) const
{
  visualization_msgs::msg::MarkerArray arr;
  int id = 0;

  auto goal_m = make_marker_base(frame_id, stamp, id++, visualization_msgs::msg::Marker::SPHERE);
  goal_m.pose = goal.pose;
  goal_m.scale.x = 0.35;
  goal_m.scale.y = 0.35;
  goal_m.scale.z = 0.35;
  goal_m.color.r = 0.1f;
  goal_m.color.g = 0.9f;
  goal_m.color.b = 0.2f;
  goal_m.color.a = 1.0f;
  arr.markers.push_back(goal_m);

  auto pose_m = make_marker_base(frame_id, stamp, id++, visualization_msgs::msg::Marker::ARROW);
  pose_m.pose = pose.pose;
  pose_m.scale.x = 0.6;
  pose_m.scale.y = 0.12;
  pose_m.scale.z = 0.12;
  pose_m.color.r = 0.2f;
  pose_m.color.g = 0.5f;
  pose_m.color.b = 1.0f;
  pose_m.color.a = 1.0f;
  arr.markers.push_back(pose_m);

  auto sparse_line = make_marker_base(frame_id, stamp, id++, visualization_msgs::msg::Marker::LINE_STRIP);
  sparse_line.scale.x = 0.06;
  sparse_line.color.r = 1.0f;
  sparse_line.color.g = 0.6f;
  sparse_line.color.b = 0.0f;
  sparse_line.color.a = 0.9f;
  for (const auto & p : sparse_path.poses) {
    sparse_line.points.push_back(p.pose.position);
  }
  arr.markers.push_back(sparse_line);

  auto dense_line = make_marker_base(frame_id, stamp, id++, visualization_msgs::msg::Marker::LINE_STRIP);
  dense_line.scale.x = 0.03;
  dense_line.color.r = 0.2f;
  dense_line.color.g = 0.8f;
  dense_line.color.b = 1.0f;
  dense_line.color.a = 0.85f;
  for (const auto & p : dense_path.poses) {
    dense_line.points.push_back(p.pose.position);
  }
  arr.markers.push_back(dense_line);

  auto la = make_marker_base(frame_id, stamp, id++, visualization_msgs::msg::Marker::SPHERE);
  la.pose.position = lookahead;
  la.scale.x = 0.22;
  la.scale.y = 0.22;
  la.scale.z = 0.22;
  la.color.r = 1.0f;
  la.color.g = 0.1f;
  la.color.b = 0.8f;
  la.color.a = 1.0f;
  arr.markers.push_back(la);

  auto trace_line = make_marker_base(frame_id, stamp, id++, visualization_msgs::msg::Marker::LINE_STRIP);
  trace_line.scale.x = 0.04;
  trace_line.color.r = 0.9f;
  trace_line.color.g = 0.9f;
  trace_line.color.b = 0.1f;
  trace_line.color.a = 0.95f;
  trace_line.points = trace;
  arr.markers.push_back(trace_line);

  return arr;
}

}  // namespace forest_navigation_ros2

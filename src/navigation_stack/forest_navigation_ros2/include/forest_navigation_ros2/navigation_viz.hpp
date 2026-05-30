#pragma once

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace forest_navigation_ros2
{

/// Marcadores RViz para debug de planeamento e Pure Pursuit.
class NavigationViz
{
public:
  visualization_msgs::msg::MarkerArray make_markers(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Path & sparse_path,
    const nav_msgs::msg::Path & dense_path,
    const std::vector<geometry_msgs::msg::PoseStamped> & route_waypoints,
    const geometry_msgs::msg::Point & lookahead,
    const std::vector<geometry_msgs::msg::Point> & trace) const;
};

}  // namespace forest_navigation_ros2

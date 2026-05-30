#pragma once

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace forest_navigation_ros2
{

/// Gera path esparso (polilinha) entre pose actual e objectivo(s).
class GlobalPlanner
{
public:
  /// Path com poses em ``map``: início → goal (segmento recto).
  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// Polilinha contínua: start → via_points[0] → … → via_points[N-1].
  /// Orientação de cada vértice = tangente ao segmento de saída (último = tangente de entrada).
  nav_msgs::msg::Path plan_polyline(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<geometry_msgs::msg::PoseStamped> & via_points) const;
};

}  // namespace forest_navigation_ros2

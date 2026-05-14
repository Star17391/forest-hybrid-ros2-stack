#pragma once

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace forest_navigation_ros2
{

/// Gera path esparso (polilinha) entre pose actual e objectivo.
/// MVP: segmento recto; v2: múltiplos waypoints.
class GlobalPlanner
{
public:
  /// Path com poses em ``frame_id`` (tipicamente ``map``): início → goal.
  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// v2: polilinha através de waypoints (inclui start e goal nas extremidades).
  nav_msgs::msg::Path plan_through_waypoints(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    const geometry_msgs::msg::PoseStamped & goal) const;
};

}  // namespace forest_navigation_ros2

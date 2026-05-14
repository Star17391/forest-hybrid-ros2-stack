#include <cmath>

#include "forest_navigation_ros2/trajectory_sampler/trajectory_sampler.hpp"

namespace forest_navigation_ros2
{

TrajectorySampler::TrajectorySampler(double step_m)
: step_m_(std::max(0.05, step_m))
{
}

void TrajectorySampler::set_step_m(double step_m)
{
  step_m_ = std::max(0.05, step_m);
}

nav_msgs::msg::Path TrajectorySampler::densify(const nav_msgs::msg::Path & sparse_path) const
{
  nav_msgs::msg::Path dense;
  if (sparse_path.poses.empty()) {
    return dense;
  }
  dense.header = sparse_path.header;

  for (size_t i = 0; i + 1 < sparse_path.poses.size(); ++i) {
    const auto & a = sparse_path.poses[i];
    const auto & b = sparse_path.poses[i + 1];
    const double dx = b.pose.position.x - a.pose.position.x;
    const double dy = b.pose.position.y - a.pose.position.y;
    const double dz = b.pose.position.z - a.pose.position.z;
    const double seg_len = std::hypot(dx, dy);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / step_m_)));
    for (int s = 0; s < steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped p;
      p.header = sparse_path.header;
      p.pose.position.x = a.pose.position.x + t * dx;
      p.pose.position.y = a.pose.position.y + t * dy;
      p.pose.position.z = a.pose.position.z + t * dz;
      p.pose.orientation = a.pose.orientation;
      if (i + 1 == sparse_path.poses.size() - 1 && s == steps - 1) {
        p.pose.orientation = b.pose.orientation;
      }
      dense.poses.push_back(p);
    }
  }
  dense.poses.push_back(sparse_path.poses.back());
  return dense;
}

}  // namespace forest_navigation_ros2

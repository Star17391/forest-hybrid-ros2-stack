#include "nav2_dstar_lite_planner/dstar_lite_planner.hpp"

#include <cmath>
#include <utility>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_dstar_lite_planner
{

void DStarLitePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("DStarLitePlanner: lifecycle node unavailable");
  }

  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smooth_iterations", rclcpp::ParameterValue(2.0));

  tolerance_ = node->get_parameter(name_ + ".tolerance").as_double();
  allow_unknown_ = node->get_parameter(name_ + ".allow_unknown").as_bool();
  smooth_iterations_ = node->get_parameter(name_ + ".smooth_iterations").as_double();

  RCLCPP_INFO(
    logger_, "DStarLitePlanner configured (frame=%s, allow_unknown=%s)",
    global_frame_.c_str(), allow_unknown_ ? "true" : "false");
}

void DStarLitePlanner::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_cost_snapshot_.clear();
  last_goal_x_ = -1;
  last_goal_y_ = -1;
}

void DStarLitePlanner::activate() {}

void DStarLitePlanner::deactivate() {}

bool DStarLitePlanner::world_to_map(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return false;
  }
  return true;
}

void DStarLitePlanner::map_to_world(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

void DStarLitePlanner::sync_costmap(bool force_rebuild)
{
  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const double origin_x = costmap_->getOriginX();
  const double origin_y = costmap_->getOriginY();
  const double resolution = costmap_->getResolution();

  const bool geometry_changed =
    force_rebuild || width != last_width_ || height != last_height_ ||
    std::abs(origin_x - last_origin_x_) > 1e-6 ||
    std::abs(origin_y - last_origin_y_) > 1e-6 ||
    std::abs(resolution - last_resolution_) > 1e-6;

  if (geometry_changed) {
    dstar_.reset(static_cast<int>(width), static_cast<int>(height));
    last_cost_snapshot_.assign(width * height, 255);
    last_width_ = width;
    last_height_ = height;
    last_origin_x_ = origin_x;
    last_origin_y_ = origin_y;
    last_resolution_ = resolution;
    last_goal_x_ = -1;
    last_goal_y_ = -1;
  }

  unsigned char * map = costmap_->getCharMap();
  const size_t n = static_cast<size_t>(width) * height;
  for (size_t i = 0; i < n; ++i) {
    unsigned char c = map[i];
    // NO_INFORMATION (255) >= kLethal(254): o core trata-o como LETAL. Logo, com
    // allow_unknown=true tem de ser convertido para TRAVERSÁVEL (livre=0), senão um
    // costmap por-povoar fica todo bloqueado e o planeador falha ("no path found").
    // Sem allow_unknown, o desconhecido é letal (conservador).
    if (c == 255) {
      c = allow_unknown_ ? 0 : DStarLite::kLethal;
    }
    if (geometry_changed || last_cost_snapshot_[i] != c) {
      const int x = static_cast<int>(i % width);
      const int y = static_cast<int>(i / width);
      dstar_.set_cell_cost(x, y, c);
      last_cost_snapshot_[i] = c;
    }
  }
}

void DStarLitePlanner::smooth_path(nav_msgs::msg::Path & path) const
{
  const int iterations = std::max(0, static_cast<int>(smooth_iterations_));
  if (iterations <= 0 || path.poses.size() < 3) {
    return;
  }

  for (int it = 0; it < iterations; ++it) {
    for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
      auto & p = path.poses[i].pose.position;
      const auto & prev = path.poses[i - 1].pose.position;
      const auto & next = path.poses[i + 1].pose.position;
      p.x = 0.25 * prev.x + 0.5 * p.x + 0.25 * next.x;
      p.y = 0.25 * prev.y + 0.5 * p.y + 0.25 * next.y;
    }
  }
}

nav_msgs::msg::Path DStarLitePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  (void)tf_;

  std::lock_guard<std::mutex> lock(mutex_);
  sync_costmap(false);

  unsigned int sx = 0;
  unsigned int sy = 0;
  unsigned int gx = 0;
  unsigned int gy = 0;

  if (!world_to_map(start.pose.position.x, start.pose.position.y, sx, sy)) {
    throw nav2_core::PlannerException("DStarLitePlanner: start outside costmap");
  }
  if (!world_to_map(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
  // Try tolerance ring around goal
    bool found = false;
    const int tol_cells = std::max(1, static_cast<int>(tolerance_ / costmap_->getResolution()));
    for (int r = 0; r <= tol_cells && !found; ++r) {
      for (int dx = -r; dx <= r && !found; ++dx) {
        for (int dy = -r; dy <= r && !found; ++dy) {
          if (std::abs(dx) != r && std::abs(dy) != r) {
            continue;
          }
          const int nx = static_cast<int>(gx) + dx;
          const int ny = static_cast<int>(gy) + dy;
          if (nx < 0 || ny < 0 ||
            nx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
            ny >= static_cast<int>(costmap_->getSizeInCellsY()))
          {
            continue;
          }
          const unsigned char c = costmap_->getCost(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
          if (c < DStarLite::kLethal) {
            gx = static_cast<unsigned int>(nx);
            gy = static_cast<unsigned int>(ny);
            found = true;
          }
        }
      }
    }
    if (!found) {
      throw nav2_core::PlannerException("DStarLitePlanner: goal outside costmap");
    }
  }

  const bool goal_moved =
    static_cast<int>(gx) != last_goal_x_ || static_cast<int>(gy) != last_goal_y_;
  if (goal_moved) {
    dstar_.set_goal(static_cast<int>(gx), static_cast<int>(gy));
    last_goal_x_ = static_cast<int>(gx);
    last_goal_y_ = static_cast<int>(gy);
  }

  dstar_.set_start(static_cast<int>(sx), static_cast<int>(sy));

  if (!dstar_.compute_shortest_path(cancel_checker)) {
    throw nav2_core::PlannerException("DStarLitePlanner: no path found");
  }

  std::vector<DStarLite::Cell> cells;
  if (!dstar_.extract_path(cells) || cells.empty()) {
    throw nav2_core::PlannerException("DStarLitePlanner: path extraction failed");
  }

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  path.poses.reserve(cells.size());

  for (const auto & cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    map_to_world(static_cast<unsigned int>(cell.x), static_cast<unsigned int>(cell.y),
      pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = start.pose.position.z;
    pose.pose.orientation = start.pose.orientation;
    path.poses.push_back(pose);
  }

  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }

  smooth_path(path);
  return path;
}

}  // namespace nav2_dstar_lite_planner

PLUGINLIB_EXPORT_CLASS(nav2_dstar_lite_planner::DStarLitePlanner, nav2_core::GlobalPlanner)

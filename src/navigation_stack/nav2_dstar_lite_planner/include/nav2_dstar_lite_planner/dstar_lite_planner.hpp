#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_dstar_lite_planner/dstar_lite.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_dstar_lite_planner
{

class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DStarLitePlanner() = default;
  ~DStarLitePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  bool world_to_map(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  void map_to_world(unsigned int mx, unsigned int my, double & wx, double & wy) const;
  void sync_costmap(bool force_rebuild);
  void smooth_path(nav_msgs::msg::Path & path) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("DStarLitePlanner")};
  rclcpp::Clock::SharedPtr clock_;

  double tolerance_{0.5};
  bool allow_unknown_{true};
  double smooth_iterations_{2.0};

  DStarLite dstar_;
  std::mutex mutex_;
  unsigned int last_width_{0};
  unsigned int last_height_{0};
  double last_origin_x_{0.0};
  double last_origin_y_{0.0};
  double last_resolution_{0.0};
  int last_goal_x_{-1};
  int last_goal_y_{-1};
  std::vector<unsigned char> last_cost_snapshot_;
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#pragma once

#include <fstream>
#include <mutex>
#include <string>

#include "forest_navigation_ros2/tracking_controller/pure_pursuit.hpp"

namespace forest_navigation_ros2
{

/// Registo CSV para análise experimental (plots offline).
class NavigationMetricsLogger
{
public:
  explicit NavigationMetricsLogger(const std::string & csv_path);

  void start_leg(const std::string & leg_id);
  void log_sample(
    double t_sec,
    const geometry_msgs::msg::PoseStamped & pose,
    const PursuitCommand & cmd);
  void end_leg();

  const std::string & path() const { return csv_path_; }

private:
  std::string csv_path_;
  std::string leg_id_;
  std::ofstream stream_;
  std::mutex mutex_;
  bool header_written_{false};
};

}  // namespace forest_navigation_ros2

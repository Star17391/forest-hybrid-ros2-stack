#include "forest_navigation_ros2/navigation_metrics.hpp"

#include <cmath>
#include <filesystem>

namespace forest_navigation_ros2
{

namespace
{
double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

NavigationMetricsLogger::NavigationMetricsLogger(const std::string & csv_path)
: csv_path_(csv_path)
{
  std::filesystem::path p(csv_path_);
  if (p.has_parent_path()) {
    std::filesystem::create_directories(p.parent_path());
  }
}

void NavigationMetricsLogger::start_leg(const std::string & leg_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  leg_id_ = leg_id;
  if (stream_.is_open()) {
    stream_.close();
  }
  stream_.open(csv_path_, std::ios::out | std::ios::app);
  if (!header_written_) {
    stream_ << "leg_id,t_sec,x,y,yaw_rad,goal_dist_m,lateral_error_m,heading_error_rad,"
            << "v_cmd,w_cmd,progress\n";
    header_written_ = true;
  }
}

void NavigationMetricsLogger::log_sample(
  double t_sec,
  const geometry_msgs::msg::PoseStamped & pose,
  const PursuitCommand & cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_.is_open()) {
    return;
  }
  const double yaw = yaw_from_quat(
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);
  stream_ << leg_id_ << "," << t_sec << ","
          << pose.pose.position.x << "," << pose.pose.position.y << "," << yaw << ","
          << cmd.goal_distance << "," << cmd.lateral_error << "," << cmd.heading_error << ","
          << cmd.linear_x << "," << cmd.angular_z << "," << cmd.progress_along_path << "\n";
}

void NavigationMetricsLogger::end_leg()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_.is_open()) {
    stream_.flush();
  }
}

}  // namespace forest_navigation_ros2

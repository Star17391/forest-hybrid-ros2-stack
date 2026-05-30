// Preprocess LaserScan: range limits, optional IMU gravity leveling (Enhanced Hector style).
// See docs/FOREST_SLAM_BIBLIOGRAPHY.md — Camada 0 shared geometric pipeline.
// TODO(motion_comp): scan deskew from wheel odometry when |v| > threshold.

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "forest_sensors_cpp/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion quat_from_rpy(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

void rotate_point(double & x, double & y, double & z, const tf2::Matrix3x3 & r)
{
  const double ox = x;
  const double oy = y;
  const double oz = z;
  x = r[0][0] * ox + r[0][1] * oy + r[0][2] * oz;
  y = r[1][0] * ox + r[1][1] * oy + r[1][2] * oz;
  z = r[2][0] * ox + r[2][1] * oy + r[2][2] * oz;
}

}  // namespace

class LaserscanPreprocessNode : public rclcpp::Node
{
public:
  LaserscanPreprocessNode()
  : Node("laserscan_preprocess_node")
  {
    scan_in_topic_ = declare_parameter<std::string>("scan_in_topic", "/scan");
    scan_out_topic_ = declare_parameter<std::string>("scan_out_topic", "/sensors/lidar/scan");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensors/imu/data");
    min_range_m_ = declare_parameter<double>("min_range_m", 0.12);
    max_range_m_ = declare_parameter<double>("max_range_m", 10.0);
    enable_imu_leveling_ = declare_parameter<bool>("enable_imu_leveling", true);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.15);
    laser_x_ = declare_parameter<double>("laser_x", 0.40);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_z_ = declare_parameter<double>("laser_z", 0.36);
    laser_roll_ = declare_parameter<double>("laser_roll", 0.0);
    laser_pitch_ = declare_parameter<double>("laser_pitch", 0.436332313);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);
    enable_motion_compensation_ = declare_parameter<bool>("enable_motion_compensation", false);
    motion_comp_speed_threshold_mps_ =
      declare_parameter<double>("motion_comp_speed_threshold_mps", 0.8);

    if (enable_motion_compensation_) {
      RCLCPP_WARN(
        get_logger(),
        "Motion compensation requested but not implemented yet (threshold %.2f m/s)",
        motion_comp_speed_threshold_mps_);
    }

    build_laser_extrinsics();

    const auto qos = forest_sensors_cpp::best_effort_sensor_qos();
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_out_topic_, qos);
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_in_topic_, qos,
      std::bind(&LaserscanPreprocessNode::on_scan, this, std::placeholders::_1));

    if (enable_imu_leveling_) {
      sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos,
        std::bind(&LaserscanPreprocessNode::on_imu, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "LaserScan preprocess %s -> %s  range [%.2f, %.2f] m  imu_leveling=%s",
      scan_in_topic_.c_str(), scan_out_topic_.c_str(), min_range_m_, max_range_m_,
      enable_imu_leveling_ ? "on" : "off");
  }

private:
  void build_laser_extrinsics()
  {
    const auto q_msg = quat_from_rpy(laser_roll_, laser_pitch_, laser_yaw_);
    const tf2::Quaternion q_laser_in_base(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    r_base_laser_ = tf2::Matrix3x3(q_laser_in_base);
    r_laser_base_ = r_base_laser_.transpose();
    t_base_laser_ = tf2::Vector3(laser_x_, laser_y_, laser_z_);
  }

  bool update_attitude_from_imu(const sensor_msgs::msg::Imu & msg)
  {
  const double w = msg.orientation.w;
  const double x = msg.orientation.x;
  const double y = msg.orientation.y;
  const double z = msg.orientation.z;
  const double n2 = w * w + x * x + y * y + z * z;
  const bool orientation_unknown = msg.orientation_covariance[0] < 0.0;

  if (std::isfinite(n2) && n2 > 1e-6 && !orientation_unknown) {
    tf2::Quaternion q(x, y, z, w);
    if (std::abs(q.length2() - 1.0) > 1e-2) {
      q.normalize();
    }
    r_world_base_ = tf2::Matrix3x3(q);
  } else {
    // Gazebo IMU: quaternion often invalid; estimate roll/pitch from gravity (accel).
    const double ax = msg.linear_acceleration.x;
    const double ay = msg.linear_acceleration.y;
    const double az = msg.linear_acceleration.z;
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
      return false;
    }
    const double g_norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (g_norm < 4.0) {
      return false;
    }
    const double roll = std::atan2(ay, az);
    const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    const double yaw = 0.0;
    const auto q_msg = quat_from_rpy(roll, pitch, yaw);
    const tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    r_world_base_ = tf2::Matrix3x3(q);
  }

  double roll{};
  double pitch{};
  double yaw{};
  r_world_base_.getRPY(roll, pitch, yaw);
  const tf2::Quaternion q_level(0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5));
  r_world_level_ = tf2::Matrix3x3(q_level);
  return true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!update_attitude_from_imu(*msg)) {
      return;
    }
    last_imu_stamp_ = rclcpp::Time(msg->header.stamp);
    has_imu_ = true;
  }

  bool imu_fresh_unlocked(const rclcpp::Time & scan_stamp) const
  {
    if (!has_imu_) {
      return false;
    }
    const rclcpp::Time now = get_clock()->now();
    if ((now - last_imu_stamp_).seconds() > imu_timeout_sec_) {
      return false;
    }
    // Scan (10 Hz) can be slightly older than latest IMU (100 Hz) — use |Δt|.
    const double sync_err = std::abs((scan_stamp - last_imu_stamp_).seconds());
    return sync_err <= imu_timeout_sec_;
  }

  void level_point_in_laser_frame(double & x, double & y, double & z) const
  {
    // laser -> base
    rotate_point(x, y, z, r_base_laser_);
    x += t_base_laser_.x();
    y += t_base_laser_.y();
    z += t_base_laser_.z();

    // Remove roll/pitch of base relative to gravity (keep yaw).
    tf2::Matrix3x3 r_level_base = r_world_level_.transpose() * r_world_base_;
    rotate_point(x, y, z, r_level_base);

    // base (leveled) -> laser
    x -= t_base_laser_.x();
    y -= t_base_laser_.y();
    z -= t_base_laser_.z();
    rotate_point(x, y, z, r_laser_base_);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    sensor_msgs::msg::LaserScan out = *scan;
    out.range_min = static_cast<float>(min_range_m_);
    out.range_max = static_cast<float>(max_range_m_);

    const rclcpp::Time scan_stamp(scan->header.stamp);
    bool level_scan = false;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      level_scan = enable_imu_leveling_ && imu_fresh_unlocked(scan_stamp);
    }

    if (enable_imu_leveling_ && !level_scan) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "IMU leveling off (no IMU yet, timeout, or stamp skew); range filter only. "
        "Check: ros2 topic hz /sensors/imu/data_raw /sensors/imu/data");
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);

    for (std::size_t i = 0; i < out.ranges.size(); ++i) {
      float r = out.ranges[i];
      if (!std::isfinite(r)) {
        out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      if (level_scan) {
        const float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
        double x = static_cast<double>(r) * std::cos(angle);
        double y = static_cast<double>(r) * std::sin(angle);
        double z = 0.0;
        level_point_in_laser_frame(x, y, z);
        r = static_cast<float>(std::hypot(x, y));
      }

      if (!std::isfinite(r) || r < static_cast<float>(min_range_m_) ||
        r > static_cast<float>(max_range_m_))
      {
        out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      } else {
        out.ranges[i] = r;
      }
    }

    pub_->publish(out);
  }

  std::string scan_in_topic_;
  std::string scan_out_topic_;
  std::string imu_topic_;
  double min_range_m_{};
  double max_range_m_{};
  bool enable_imu_leveling_{true};
  double imu_timeout_sec_{};
  double laser_x_{};
  double laser_y_{};
  double laser_z_{};
  double laser_roll_{};
  double laser_pitch_{};
  double laser_yaw_{};
  bool enable_motion_compensation_{false};
  double motion_comp_speed_threshold_mps_{};

  tf2::Matrix3x3 r_base_laser_;
  tf2::Matrix3x3 r_laser_base_;
  tf2::Vector3 t_base_laser_;

  mutable std::mutex imu_mutex_;
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Matrix3x3 r_world_base_;
  tf2::Matrix3x3 r_world_level_;
  bool has_imu_{false};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserscanPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}

// Fuse left/right track speeds into differential-drive odometry in ``odom``.
// Prefer Gazebo track_cmd_vel (Float64); fallback to track Odometry twist.

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double finite_or_zero(double v)
{
  return std::isfinite(v) ? v : 0.0;
}

double track_linear_speed(const nav_msgs::msg::Odometry & odom)
{
  const auto & t = odom.twist.twist;
  if (std::isfinite(t.linear.x) && std::abs(t.linear.x) > 1e-9) {
    return t.linear.x;
  }
  if (std::isfinite(t.linear.y) && std::abs(t.linear.y) > 1e-9) {
    return t.linear.y;
  }
  if (std::isfinite(t.linear.z) && std::abs(t.linear.z) > 1e-9) {
    return t.linear.z;
  }
  return 0.0;
}

}  // namespace

class TrackedWheelOdometryNode : public rclcpp::Node
{
public:
  TrackedWheelOdometryNode()
  : Node("tracked_wheel_odometry_node")
  {
    left_odom_topic_ =
      declare_parameter<std::string>("left_odom_topic", "/forest_gen/gz/left_track_odometry");
    right_odom_topic_ =
      declare_parameter<std::string>("right_odom_topic", "/forest_gen/gz/right_track_odometry");
    left_cmd_topic_ = declare_parameter<std::string>(
      "left_cmd_topic", "/model/marble_hd2/link/left_track/track_cmd_vel");
    right_cmd_topic_ = declare_parameter<std::string>(
      "right_cmd_topic", "/model/marble_hd2/link/right_track/track_cmd_vel");
    use_track_cmd_vel_ = declare_parameter<bool>("use_track_cmd_vel", true);
    output_topic_ = declare_parameter<std::string>("output_topic", "/sensors/wheel_odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "marble_hd2/base_link");
    track_separation_m_ = declare_parameter<double>("track_separation_m", 0.44);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    velocity_scale_ = declare_parameter<double>("velocity_scale", 1.0);

    rclcpp::QoS qos(rclcpp::KeepLast(30));
    qos.reliable();

    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
    sub_l_ = create_subscription<nav_msgs::msg::Odometry>(
      left_odom_topic_, qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_left_ = msg;
      });
    sub_r_ = create_subscription<nav_msgs::msg::Odometry>(
      right_odom_topic_, qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_right_ = msg;
      });

    if (use_track_cmd_vel_) {
      sub_l_cmd_ = create_subscription<std_msgs::msg::Float64>(
        left_cmd_topic_, qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          last_vl_cmd_ = msg->data;
          have_vl_cmd_ = std::isfinite(msg->data);
        });
      sub_r_cmd_ = create_subscription<std_msgs::msg::Float64>(
        right_cmd_topic_, qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          last_vr_cmd_ = msg->data;
          have_vr_cmd_ = std::isfinite(msg->data);
        });
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrackedWheelOdometryNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Wheel odom -> %s  tracks=%.2f m  cmd_vel=%s", output_topic_.c_str(),
      track_separation_m_, use_track_cmd_vel_ ? "yes" : "no");
  }

private:
  bool sample_track_speeds(double & vl, double & vr)
  {
    nav_msgs::msg::Odometry::SharedPtr left;
    nav_msgs::msg::Odometry::SharedPtr right;
    bool have_vl_cmd{};
    bool have_vr_cmd{};
    double vl_cmd{};
    double vr_cmd{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      left = last_left_;
      right = last_right_;
      have_vl_cmd = have_vl_cmd_;
      have_vr_cmd = have_vr_cmd_;
      vl_cmd = last_vl_cmd_;
      vr_cmd = last_vr_cmd_;
    }

    if (use_track_cmd_vel_ && have_vl_cmd && have_vr_cmd) {
      vl = velocity_scale_ * vl_cmd;
      vr = velocity_scale_ * vr_cmd;
      return true;
    }

    if (!left || !right) {
      return false;
    }

    vl = velocity_scale_ * track_linear_speed(*left);
    vr = velocity_scale_ * track_linear_speed(*right);
    return true;
  }

  void on_timer()
  {
    double vl{};
    double vr{};
    if (!sample_track_speeds(vl, vr)) {
      return;
    }

    vl = finite_or_zero(vl);
    vr = finite_or_zero(vr);
    constexpr double kMaxTrackSpeed = 3.0;
    vl = std::clamp(vl, -kMaxTrackSpeed, kMaxTrackSpeed);
    vr = std::clamp(vr, -kMaxTrackSpeed, kMaxTrackSpeed);

    const double v = 0.5 * (vl + vr);
    const double omega = std::clamp(
      (vr - vl) / std::max(1e-3, track_separation_m_), -4.0, 4.0);

    if (!std::isfinite(v) || !std::isfinite(omega)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping wheel odom publish: non-finite v/omega (vl=%.3f vr=%.3f)", vl, vr);
      return;
    }

    // Use sim stamp from Gazebo track odom — NOT get_clock()->now() (wall/sim jump poisava EKF).
    nav_msgs::msg::Odometry::SharedPtr ref_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ref_odom = last_left_ ? last_left_ : last_right_;
    }
    if (!ref_odom) {
      return;
    }
    const rclcpp::Time now(ref_odom->header.stamp);
    if (now.nanoseconds() <= 0) {
      return;
    }

    if (have_last_pub_) {
      const double dt = (now - last_pub_time_).seconds();
      // Sim pause, /clock jump, or RViz lag: never integrate huge dt (causa NaN no EKF/TF).
      if (dt <= 0.0 || dt > 0.25) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Wheel odom: skip integrate dt=%.3f s (reset pose integrator)", dt);
        x_ = y_ = yaw_ = 0.0;
        have_last_pub_ = false;
        last_pub_time_ = now;
        return;
      }
      const double cy = std::cos(yaw_);
      const double sy = std::sin(yaw_);
      x_ += v * dt * cy;
      y_ += v * dt * sy;
      yaw_ += omega * dt;
      while (yaw_ > kPi) {
        yaw_ -= 2.0 * kPi;
      }
      while (yaw_ < -kPi) {
        yaw_ += 2.0 * kPi;
      }
    } else {
      have_last_pub_ = true;
    }
    last_pub_time_ = now;

    if (!std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(yaw_)) {
      x_ = y_ = yaw_ = 0.0;
      RCLCPP_WARN(get_logger(), "Wheel odom state reset (NaN guard)");
    }

    nav_msgs::msg::Odometry out;
    out.header.stamp = now;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.position.x = x_;
    out.pose.pose.position.y = y_;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = quat_from_yaw(yaw_);
    out.twist.twist.linear.x = v;
    out.twist.twist.linear.y = 0.0;
    out.twist.twist.angular.z = omega;

    for (int i = 0; i < 36; ++i) {
      out.pose.covariance[static_cast<size_t>(i)] = 0.0;
      out.twist.covariance[static_cast<size_t>(i)] = 0.0;
    }
    out.pose.covariance[0] = 1e6;
    out.pose.covariance[7] = 1e6;
    out.pose.covariance[35] = 1e6;
    out.twist.covariance[0] = 0.1;
    out.twist.covariance[7] = 1.0;
    out.twist.covariance[14] = 1.0;
    out.twist.covariance[21] = 1.0;
    out.twist.covariance[28] = 1.0;
    out.twist.covariance[35] = 0.15;

    pub_->publish(out);
  }

  std::string left_odom_topic_;
  std::string right_odom_topic_;
  std::string left_cmd_topic_;
  std::string right_cmd_topic_;
  bool use_track_cmd_vel_{true};
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double track_separation_m_{};
  double publish_rate_hz_{};
  double velocity_scale_{};

  std::mutex mutex_;
  nav_msgs::msg::Odometry::SharedPtr last_left_;
  nav_msgs::msg::Odometry::SharedPtr last_right_;
  double last_vl_cmd_{0.0};
  double last_vr_cmd_{0.0};
  bool have_vl_cmd_{false};
  bool have_vr_cmd_{false};

  bool have_last_pub_{false};
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_l_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_r_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_l_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_r_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackedWheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}

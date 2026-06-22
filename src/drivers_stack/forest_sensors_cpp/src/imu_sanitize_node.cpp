// Prepare Gazebo / raw IMU for robot_localization: mark invalid orientation, sane covariances.

#include <cmath>
#include <memory>
#include <string>

#include "forest_sensors_cpp/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

bool quat_valid(const sensor_msgs::msg::Imu & imu)
{
  const double w = imu.orientation.w;
  const double x = imu.orientation.x;
  const double y = imu.orientation.y;
  const double z = imu.orientation.z;
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  const double n2 = w * w + x * x + y * y + z * z;
  return n2 > 1e-6;
}

void set_covariance_diagonal(std::array<double, 9> & cov, double value)
{
  cov.fill(0.0);
  cov[0] = value;
  cov[4] = value;
  cov[8] = value;
}

bool cov_valid(double v)
{
  return std::isfinite(v) && v > 1e-9;
}

void ensure_covariance(std::array<double, 9> & cov, double fallback)
{
  if (!cov_valid(cov[0])) {
    set_covariance_diagonal(cov, fallback);
  }
}

}  // namespace

class ImuSanitizeNode : public rclcpp::Node
{
public:
  ImuSanitizeNode()
  : Node("imu_sanitize_node")
  {
    in_topic_ = declare_parameter<std::string>("imu_in_topic", "/sensors/imu/data_raw");
    out_topic_ = declare_parameter<std::string>("imu_out_topic", "/sensors/imu/data");
    default_ang_cov_ = declare_parameter<double>("default_angular_velocity_covariance", 0.02);
    default_ori_cov_ = declare_parameter<double>("default_orientation_covariance", 0.05);
    default_lin_cov_ = declare_parameter<double>("default_linear_acceleration_covariance", 0.1);

    const auto qos = forest_sensors_cpp::best_effort_sensor_qos();
    pub_ = create_publisher<sensor_msgs::msg::Imu>(out_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      in_topic_, qos,
      std::bind(&ImuSanitizeNode::on_imu, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "IMU sanitize %s -> %s", in_topic_.c_str(), out_topic_.c_str());
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double g_in = std::sqrt(
      msg->linear_acceleration.x * msg->linear_acceleration.x +
      msg->linear_acceleration.y * msg->linear_acceleration.y +
      msg->linear_acceleration.z * msg->linear_acceleration.z);

    if (!logged_first_sample_ && g_in > 4.0) {
      logged_first_sample_ = true;
      const double n2 = msg->orientation.w * msg->orientation.w +
        msg->orientation.x * msg->orientation.x +
        msg->orientation.y * msg->orientation.y +
        msg->orientation.z * msg->orientation.z;
      RCLCPP_INFO(
        get_logger(),
        "IMU raw sample: frame=%s |g|=%.3f ang_cov[0]=%.3e ori_cov[0]=%.3e quat_norm=%.4f",
        msg->header.frame_id.c_str(), g_in, msg->angular_velocity_covariance[0],
        msg->orientation_covariance[0], std::sqrt(n2));
    }

    sensor_msgs::msg::Imu out = *msg;

    if (!quat_valid(out)) {
      out.orientation.x = 0.0;
      out.orientation.y = 0.0;
      out.orientation.z = 0.0;
      out.orientation.w = 1.0;
      out.orientation_covariance[0] = -1.0;
    } else {
      // Gazebo: valid quat but orientation_cov=0 → infinite trust in robot_localization.
      ensure_covariance(out.orientation_covariance, default_ori_cov_);
    }

    const auto & w = out.angular_velocity;
    if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping IMU sample with NaN angular_velocity");
      return;
    }

    // Gazebo often sends 0 covariance (= infinite trust) -> EKF NaN. Force sane minimum.
    ensure_covariance(out.angular_velocity_covariance, default_ang_cov_);

    const auto & a = out.linear_acceleration;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z)) {
      out.linear_acceleration.x = 0.0;
      out.linear_acceleration.y = 0.0;
      out.linear_acceleration.z = 9.81;
    }
    // Accel linear: não fundir no EKF (2D legacy + SE3 gyro-only por agora).
    // robot_localization: cov=-1 → medição ignorada para esse eixo.
    // IMPORTANTE: marcar os 3 eixos — só [0]=-1 deixava ay/az com cov=0 do Gazebo.
    out.linear_acceleration_covariance.fill(-1.0);

    const double g = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (g < 4.0 || g > 15.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU gravity magnitude %.2f m/s^2 unusual (expected ~9.81)", g);
    }

    pub_->publish(out);
  }

  std::string in_topic_;
  std::string out_topic_;
  double default_ang_cov_{};
  double default_ori_cov_{};
  double default_lin_cov_{};

  bool logged_first_sample_{false};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSanitizeNode>());
  rclcpp::shutdown();
  return 0;
}

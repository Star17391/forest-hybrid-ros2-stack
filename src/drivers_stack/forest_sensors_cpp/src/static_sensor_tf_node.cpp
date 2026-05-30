#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

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

}  // namespace

class StaticSensorTfNode : public rclcpp::Node
{
public:
  StaticSensorTfNode()
  : Node("static_sensor_tf_node")
  {
    parent_frame_ =
      declare_parameter<std::string>("parent_frame", "marble_hd2/base_link");
    child_frame_ = declare_parameter<std::string>("child_frame", "laser");
    x_ = declare_parameter<double>("x", 0.0);
    y_ = declare_parameter<double>("y", 0.0);
    z_ = declare_parameter<double>("z", 0.15);
    roll_ = declare_parameter<double>("roll", 0.0);
    pitch_ = declare_parameter<double>("pitch", 0.0);
    yaw_ = declare_parameter<double>("yaw", 0.0);
    const double period = std::max(0.5, declare_parameter<double>("republish_period_sec", 5.0));

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_once();
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&StaticSensorTfNode::publish_once, this));

    if (parent_frame_ == "base_link" && parent_frame_.find('/') == std::string::npos) {
      RCLCPP_WARN(
        get_logger(),
        "parent_frame is bare 'base_link' — extrinsics YAML may not have loaded; "
        "expected marble_hd2/base_link (TF tree will break in RViz)");
    }
    RCLCPP_INFO(
      get_logger(),
      "Static TF %s -> %s  xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) rad",
      parent_frame_.c_str(), child_frame_.c_str(), x_, y_, z_, roll_, pitch_, yaw_);
  }

private:
  void publish_once()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = child_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = z_;
    tf.transform.rotation = quat_from_rpy(roll_, pitch_, yaw_);
    broadcaster_->sendTransform(tf);
  }

  std::string parent_frame_;
  std::string child_frame_;
  double x_{};
  double y_{};
  double z_{};
  double roll_{};
  double pitch_{};
  double yaw_{};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticSensorTfNode>());
  rclcpp::shutdown();
  return 0;
}

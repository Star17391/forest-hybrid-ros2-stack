// Publishes /state/pose_fused (map frame) from TF map -> base_link.
// Odometry contract topic /state/odometry is produced by robot_localization (remapped).

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class StateContractNode : public rclcpp::Node
{
public:
  StateContractNode()
  : Node("state_contract_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "marble_hd2/base_link");
    publish_hz_ = declare_parameter<double>("publish_hz", 20.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/state/pose_fused", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StateContractNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "Publishing /state/pose_fused from TF %s -> %s",
      map_frame_.c_str(), base_frame_.c_str());
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "pose_fused TF %s -> %s: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = tf.header.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    pub_pose_->publish(pose);
  }

  std::string map_frame_;
  std::string base_frame_;
  double publish_hz_{};
  double tf_timeout_sec_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateContractNode>());
  rclcpp::shutdown();
  return 0;
}

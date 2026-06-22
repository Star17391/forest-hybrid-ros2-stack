#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace forest_nav2_bringup
{

/// Relays Nav2 cmd_vel to the stack contract topic /forest_gen/cmd_vel (sole actuator feed).
class CmdVelContractRelay : public rclcpp::Node
{
public:
  CmdVelContractRelay()
  : Node("cmd_vel_contract_relay")
  {
    declare_parameter<std::string>("input_topic", "cmd_vel");
    declare_parameter<std::string>("output_topic", "/forest_gen/cmd_vel");

    const auto in_topic = get_parameter("input_topic").as_string();
    const auto out_topic = get_parameter("output_topic").as_string();

    pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic, 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      in_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { pub_->publish(*msg); });

    RCLCPP_INFO(get_logger(), "cmd_vel relay %s -> %s", in_topic.c_str(), out_topic.c_str());
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

}  // namespace forest_nav2_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_nav2_bringup::CmdVelContractRelay>());
  rclcpp::shutdown();
  return 0;
}

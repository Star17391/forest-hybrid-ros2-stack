#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace forest_robot_supervisor
{

class OperationModeNode final : public rclcpp::Node
{
public:
  OperationModeNode() : Node("operation_mode_node")
  {
    declare_parameter<std::string>("operation_mode", "ground");
    pub_ = create_publisher<std_msgs::msg::String>(
      "/robot/operation_mode", rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(500ms, std::bind(&OperationModeNode::on_timer, this));
    on_timer();
  }

private:
  void on_timer()
  {
    std_msgs::msg::String msg;
    msg.data = get_parameter("operation_mode").as_string();
    if (msg.data != "ground" && msg.data != "aerial") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "operation_mode must be 'ground' or 'aerial', got '%s' — clamping to ground",
        msg.data.c_str());
      msg.data = "ground";
    }
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace forest_robot_supervisor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_robot_supervisor::OperationModeNode>());
  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <memory>
#include <string>

#include "forest_hybrid_msgs/msg/operation_mode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace forest_robot_supervisor
{

class OperationModeNode final : public rclcpp::Node
{
public:
  OperationModeNode() : Node("operation_mode_node")
  {
    declare_parameter<std::string>("operation_mode", "ground");
    pub_ = create_publisher<forest_hybrid_msgs::msg::OperationMode>(
      "/system/locomotion_mode", rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(500ms, std::bind(&OperationModeNode::on_timer, this));
    on_timer();
  }

private:
  void on_timer()
  {
    forest_hybrid_msgs::msg::OperationMode msg;
    const auto configured_mode = get_parameter("operation_mode").as_string();
    if (configured_mode == "ground") {
      msg.mode = forest_hybrid_msgs::msg::OperationMode::MODE_GROUND;
      msg.mode_name = "ground";
    } else if (configured_mode == "aerial") {
      msg.mode = forest_hybrid_msgs::msg::OperationMode::MODE_AERIAL;
      msg.mode_name = "aerial";
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "operation_mode must be 'ground' or 'aerial', got '%s' — clamping to ground",
        configured_mode.c_str());
      msg.mode = forest_hybrid_msgs::msg::OperationMode::MODE_GROUND;
      msg.mode_name = "ground";
    }
    pub_->publish(msg);
  }

  rclcpp::Publisher<forest_hybrid_msgs::msg::OperationMode>::SharedPtr pub_;
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

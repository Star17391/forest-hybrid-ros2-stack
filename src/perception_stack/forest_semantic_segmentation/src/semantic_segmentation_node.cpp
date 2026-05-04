#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace forest_semantic_segmentation
{

class SemanticSegmentationNode final : public rclcpp::Node
{
public:
  SemanticSegmentationNode() : Node("semantic_segmentation_node")
  {
    aerial_.store(false);
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot/operation_mode", rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) { on_mode(msg); });
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_image(msg); });
    pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/segmentation/class_mask", rclcpp::SensorDataQoS());
  }

private:
  void on_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    aerial_.store(msg->data == "aerial");
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (aerial_.load()) {
      return;
    }
    if (msg->width == 0 || msg->height == 0) {
      return;
    }
    // Baseline: máscara placeholder (classe 0). Substituir por ONNX / inferência.
    sensor_msgs::msg::Image out;
    out.header = msg->header;
    out.height = msg->height;
    out.width = msg->width;
    out.encoding = "mono8";
    out.is_bigendian = msg->is_bigendian;
    out.step = msg->width;
    out.data.assign(static_cast<size_t>(out.height) * static_cast<size_t>(out.step), 0u);
    pub_->publish(out);
  }

  std::atomic<bool> aerial_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace forest_semantic_segmentation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_semantic_segmentation::SemanticSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}

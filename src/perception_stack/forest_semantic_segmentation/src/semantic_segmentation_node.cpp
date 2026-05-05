#include <atomic>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "forest_hybrid_msgs/msg/operation_mode.hpp"
#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace forest_semantic_segmentation
{

class SemanticSegmentationNode final : public rclcpp::Node
{
public:
  SemanticSegmentationNode() : Node("semantic_segmentation_node")
  {
    declare_parameter<std::string>("onnx_model_path", "");
    declare_parameter<int>("model_input_width", 512);
    declare_parameter<int>("model_input_height", 384);
    declare_parameter<bool>("rgb_input", true);

    model_input_width_ = std::max(1, static_cast<int>(get_parameter("model_input_width").as_int()));
    model_input_height_ = std::max(1, static_cast<int>(get_parameter("model_input_height").as_int()));
    rgb_input_ = get_parameter("rgb_input").as_bool();
    const auto onnx_path = get_parameter("onnx_model_path").as_string();
    try_load_model(onnx_path);

    aerial_.store(false);
    mode_sub_ = create_subscription<forest_hybrid_msgs::msg::OperationMode>(
      "/system/locomotion_mode", rclcpp::QoS(10).transient_local(),
      [this](const forest_hybrid_msgs::msg::OperationMode::SharedPtr msg) { on_mode(msg); });
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_image(msg); });
    pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/perception/semantic_mask", rclcpp::SensorDataQoS());
  }

private:
  void try_load_model(const std::string & onnx_path)
  {
    if (onnx_path.empty()) {
      RCLCPP_WARN(get_logger(), "onnx_model_path is empty; publishing zero masks.");
      return;
    }
    if (!std::filesystem::exists(onnx_path)) {
      RCLCPP_WARN(get_logger(), "ONNX model not found at %s; publishing zero masks.", onnx_path.c_str());
      return;
    }
    try {
      net_ = cv::dnn::readNetFromONNX(onnx_path);
      if (net_.empty()) {
        RCLCPP_WARN(get_logger(), "Failed loading ONNX model; publishing zero masks.");
        return;
      }
      model_loaded_ = true;
      RCLCPP_INFO(
        get_logger(), "Loaded ONNX model: %s (input: %dx%d)",
        onnx_path.c_str(), model_input_width_, model_input_height_);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(get_logger(), "ONNX load error: %s", e.what());
    }
  }

  void on_mode(const forest_hybrid_msgs::msg::OperationMode::SharedPtr msg)
  {
    aerial_.store(msg->mode == forest_hybrid_msgs::msg::OperationMode::MODE_AERIAL);
  }

  sensor_msgs::msg::Image create_empty_mask(const sensor_msgs::msg::Image & src) const
  {
    sensor_msgs::msg::Image out;
    out.header = src.header;
    out.height = src.height;
    out.width = src.width;
    out.encoding = "mono8";
    out.is_bigendian = src.is_bigendian;
    out.step = src.width;
    out.data.assign(static_cast<size_t>(out.height) * static_cast<size_t>(out.step), 0u);
    return out;
  }

  bool convert_image_to_bgr(const sensor_msgs::msg::Image & msg, cv::Mat & bgr) const
  {
    const int width = static_cast<int>(msg.width);
    const int height = static_cast<int>(msg.height);
    if (width <= 0 || height <= 0 || msg.data.empty()) {
      return false;
    }

    if (msg.encoding == "rgb8") {
      cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()), msg.step);
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return true;
    }
    if (msg.encoding == "bgr8") {
      cv::Mat tmp(height, width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()), msg.step);
      bgr = tmp.clone();
      return true;
    }
    if (msg.encoding == "mono8") {
      cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t *>(msg.data.data()), msg.step);
      cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      return true;
    }
    return false;
  }

  sensor_msgs::msg::Image infer_mask(const sensor_msgs::msg::Image & msg)
  {
    cv::Mat bgr;
    if (!convert_image_to_bgr(msg, bgr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Unsupported image encoding: %s", msg.encoding.c_str());
      return create_empty_mask(msg);
    }

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(model_input_width_, model_input_height_));
    if (rgb_input_) {
      cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    }

    cv::Mat blob = cv::dnn::blobFromImage(
      resized, 1.0 / 255.0, cv::Size(model_input_width_, model_input_height_),
      cv::Scalar(), false, false, CV_32F);

    try {
      net_.setInput(blob);
      cv::Mat out = net_.forward();  // [1, C, H, W]
      if (out.dims != 4 || out.size[0] != 1) {
        return create_empty_mask(msg);
      }
      const int classes = out.size[1];
      const int out_h = out.size[2];
      const int out_w = out.size[3];
      cv::Mat mask(out_h, out_w, CV_8UC1, cv::Scalar(0));

      for (int y = 0; y < out_h; ++y) {
        for (int x = 0; x < out_w; ++x) {
          int best_class = 0;
          float best_score = -1e30F;
          for (int c = 0; c < classes; ++c) {
            const float score = out.ptr<float>(0, c, y)[x];
            if (score > best_score) {
              best_score = score;
              best_class = c;
            }
          }
          mask.at<uint8_t>(y, x) = static_cast<uint8_t>(std::clamp(best_class, 0, 255));
        }
      }

      cv::Mat mask_full;
      cv::resize(mask, mask_full, cv::Size(static_cast<int>(msg.width), static_cast<int>(msg.height)), 0, 0, cv::INTER_NEAREST);
      sensor_msgs::msg::Image ros_mask;
      ros_mask.header = msg.header;
      ros_mask.height = static_cast<uint32_t>(mask_full.rows);
      ros_mask.width = static_cast<uint32_t>(mask_full.cols);
      ros_mask.encoding = "mono8";
      ros_mask.is_bigendian = msg.is_bigendian;
      ros_mask.step = static_cast<uint32_t>(mask_full.cols);
      ros_mask.data.assign(mask_full.data, mask_full.data + (mask_full.rows * mask_full.cols));
      return ros_mask;
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "ONNX inference error: %s", e.what());
      return create_empty_mask(msg);
    }
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (aerial_.load()) {
      return;
    }
    if (msg->width == 0 || msg->height == 0) {
      return;
    }
    sensor_msgs::msg::Image out;
    if (model_loaded_) {
      out = infer_mask(*msg);
    } else {
      out = create_empty_mask(*msg);
    }
    pub_->publish(out);
  }

  std::atomic<bool> aerial_{false};
  bool model_loaded_{false};
  bool rgb_input_{true};
  int model_input_width_{512};
  int model_input_height_{384};
  cv::dnn::Net net_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::OperationMode>::SharedPtr mode_sub_;
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

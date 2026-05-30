#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "forest_sensors_cpp/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LaserscanToPointcloud2Node : public rclcpp::Node
{
public:
  LaserscanToPointcloud2Node()
  : Node("laserscan_to_pointcloud2_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/sensors/lidar/points");
    min_range_m_ = declare_parameter<double>("min_range_m", 0.12);
    max_range_m_ = declare_parameter<double>("max_range_m", 10.0);

    const auto qos = forest_sensors_cpp::best_effort_sensor_qos();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, qos,
      std::bind(&LaserscanToPointcloud2Node::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "%s -> %s (range %.2f..%.2f m)", scan_topic_.c_str(),
      cloud_topic_.c_str(), min_range_m_, max_range_m_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (scan->ranges.empty()) {
      return;
    }

    const double range_min = std::max(
      static_cast<double>(scan->range_min), min_range_m_);
    const double range_max = std::min(
      static_cast<double>(scan->range_max), max_range_m_);

    std::vector<std::array<float, 3>> points;
    points.reserve(scan->ranges.size());

    float angle = scan->angle_min;
    for (const float r : scan->ranges) {
      if (std::isfinite(r) && r >= static_cast<float>(range_min) &&
        r <= static_cast<float>(range_max))
      {
        points.push_back(
          {r * std::cos(angle), r * std::sin(angle), 0.0f});
      }
      angle += scan->angle_increment;
    }

    if (points.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan->header;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      3,
      "x", 0, sensor_msgs::msg::PointField::FLOAT32,
      "y", 4, sensor_msgs::msg::PointField::FLOAT32,
      "z", 8, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto & pt : points) {
      *iter_x = pt[0];
      *iter_y = pt[1];
      *iter_z = pt[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud.width = static_cast<uint32_t>(points.size());
    cloud.row_step = cloud.point_step * cloud.width;
    pub_->publish(cloud);
  }

  std::string scan_topic_;
  std::string cloud_topic_;
  double min_range_m_{};
  double max_range_m_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserscanToPointcloud2Node>());
  rclcpp::shutdown();
  return 0;
}

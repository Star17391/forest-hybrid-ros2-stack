// LiDAR scan classification — Fase 1 Palacín v1 (line-fit ground) + legacy min-z fallback.
// See docs/FOREST_SLAM_BIBLIOGRAPHY.md and docs/LOCALIZATION_SLAM_ARCHITECTURE.md § Fase 1.

#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "forest_lidar_preprocess_cpp/palacin_ground_line.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "forest_sensors_cpp/qos_profiles.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

constexpr uint8_t kLabelInvalid = 0;
constexpr uint8_t kLabelGround = 1;
constexpr uint8_t kLabelOther = 2;
constexpr uint8_t kLabelHole = 3;
constexpr uint8_t kLabelObstacle = 4;

struct ClassifiedPoint
{
  float x_laser{};
  float y_laser{};
  double x_class{};
  double y_class{};
  double z_class{};
  float range{};
  bool beam_valid{false};
  uint8_t label{kLabelInvalid};
};

}  // namespace

class LidarScanClassifyNode : public rclcpp::Node
{
public:
  LidarScanClassifyNode()
  : Node("lidar_scan_classify_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_type_ = declare_parameter<std::string>("input_type", "scan");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/sensors/lidar/scan");
    pointcloud_topic_ =
      declare_parameter<std::string>("pointcloud_topic", "/sensors/lidar/points");
    classify_near_max_range_m_ =
      declare_parameter<double>("classify_near_max_range_m", 8.0);
    max_points_per_cloud_ = declare_parameter<int>("max_points_per_cloud", 120000);
    classification_frame_ =
      declare_parameter<std::string>("classification_frame", "marble_hd2/base_link");
    classify_mode_ = declare_parameter<std::string>("classify_mode", "palacin_v1");
    labeled_cloud_topic_ =
      declare_parameter<std::string>("labeled_cloud_topic", "/perception/lidar/points_labeled");
    labeled_cloud_output_frame_ =
      declare_parameter<std::string>("labeled_cloud_output_frame", "");
    scan_ground_topic_ =
      declare_parameter<std::string>("scan_ground_topic", "/perception/lidar/scan_ground");
    scan_other_topic_ =
      declare_parameter<std::string>("scan_other_topic", "/perception/lidar/scan_other");
    scan_hazard_topic_ =
      declare_parameter<std::string>("scan_hazard_topic", "/perception/lidar/scan_hazard");
    min_range_m_ = declare_parameter<double>("min_range_m", 0.12);
    max_range_m_ = declare_parameter<double>("max_range_m", 10.0);
    forward_min_x_m_ = declare_parameter<double>("forward_min_x_m", 0.15);
    ground_height_band_m_ = declare_parameter<double>("ground_height_band_m", 0.05);
    hole_depth_m_ = declare_parameter<double>("hole_depth_m", 0.05);
    obstacle_height_m_ = declare_parameter<double>("obstacle_height_m", 0.05);
    ground_z_offset_m_ = declare_parameter<double>("ground_z_offset_m", 0.0);
    ransac_inlier_thresh_m_ = declare_parameter<double>("ransac_inlier_thresh_m", 0.06);
    ransac_max_iterations_ = declare_parameter<int>("ransac_max_iterations", 80);
    ransac_min_inliers_ = declare_parameter<int>("ransac_min_inliers", 12);
    ransac_min_inlier_ratio_ = declare_parameter<double>("ransac_min_inlier_ratio", 0.25);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    fallback_laser_pitch_rad_ = declare_parameter<double>("fallback_laser_pitch_rad", 0.436332313);
    fallback_laser_height_m_ = declare_parameter<double>("fallback_laser_height_m", 0.36);

    const auto qos = forest_sensors_cpp::best_effort_sensor_qos();
    pub_labeled_ = create_publisher<sensor_msgs::msg::PointCloud2>(labeled_cloud_topic_, qos);
    pub_ground_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_ground_topic_, qos);
    pub_other_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_other_topic_, qos);
    pub_hazard_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_hazard_topic_, qos);

    if (input_type_ == "pointcloud" || input_type_ == "cloud" || input_type_ == "3d") {
      sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, qos,
        std::bind(&LidarScanClassifyNode::on_cloud, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(),
        "LiDAR classify [%s] 3D: %s near<=%.1f m -> %s (frame %s)",
        classify_mode_.c_str(), pointcloud_topic_.c_str(), classify_near_max_range_m_,
        labeled_cloud_topic_.c_str(), classification_frame_.c_str());
    } else {
      sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, qos,
        std::bind(&LidarScanClassifyNode::on_scan, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(),
        "LiDAR classify [%s] 2D: %s -> ground/hole/obstacle (frame %s, band %.3f m)",
        classify_mode_.c_str(), scan_topic_.c_str(), classification_frame_.c_str(),
        ground_height_band_m_);
    }
  }

private:
  bool lookup_transform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & stamp, geometry_msgs::msg::TransformStamped & out) const
  {
    if (target_frame == source_frame) {
      out.header.stamp = stamp;
      out.header.frame_id = target_frame;
      out.child_frame_id = source_frame;
      out.transform.rotation.w = 1.0;
      return true;
    }
    try {
      out = tf_buffer_.lookupTransform(
        target_frame, source_frame, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      return true;
    } catch (const tf2::TransformException &) {
      try {
        out = tf_buffer_.lookupTransform(
          target_frame, source_frame, tf2::TimePointZero);
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TF %s -> %s: %s (fallback pitch/height)", source_frame.c_str(),
          target_frame.c_str(), ex.what());
        return false;
      }
    }
  }

  void transform_point(
    const geometry_msgs::msg::TransformStamped & tf, double x, double y, double z,
    double & ox, double & oy, double & oz) const
  {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = tf.child_frame_id;
    in.header.stamp = tf.header.stamp;
    in.point.x = x;
    in.point.y = y;
    in.point.z = z;

    geometry_msgs::msg::PointStamped out;
    tf2::doTransform(in, out, tf);
    ox = out.point.x;
    oy = out.point.y;
    oz = out.point.z;
  }

  void fallback_transform(
    double x_laser, double y_laser, double & x_out, double & y_out, double & z_out) const
  {
    x_out = x_laser;
    y_out = y_laser;
    z_out = fallback_laser_height_m_ - x_laser * std::tan(fallback_laser_pitch_rad_);
  }

  forest_lidar_preprocess_cpp::GroundLine2D estimate_ground_line(
    const std::vector<ClassifiedPoint> & points) const
  {
    std::vector<std::pair<double, double>> xz;
    xz.reserve(points.size());
    for (const auto & pt : points) {
      if (!pt.beam_valid) {
        continue;
      }
      if (pt.x_class < forward_min_x_m_) {
        continue;
      }
      xz.emplace_back(pt.x_class, pt.z_class);
    }

    if (classify_mode_ == "min_z" || classify_mode_ == "legacy") {
      return forest_lidar_preprocess_cpp::fit_ground_line_min_z(xz, ground_z_offset_m_);
    }

    const std::size_t min_inliers = static_cast<std::size_t>(ransac_min_inliers_);
    auto line = forest_lidar_preprocess_cpp::fit_ground_line_ransac(
      xz, ransac_inlier_thresh_m_, ransac_max_iterations_, min_inliers,
      static_cast<unsigned int>(get_clock()->now().nanoseconds()));

    if (line.valid && xz.size() > 0) {
      const double ratio = static_cast<double>(line.inliers) / static_cast<double>(xz.size());
      if (ratio >= ransac_min_inlier_ratio_) {
        return line;
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Palacín RANSAC weak (inliers=%zu/%zu) — fallback min-z",
      line.inliers, xz.size());
    return forest_lidar_preprocess_cpp::fit_ground_line_min_z(xz, ground_z_offset_m_);
  }

  void classify_points(
    std::vector<ClassifiedPoint> & points,
    const forest_lidar_preprocess_cpp::GroundLine2D & ground) const
  {
    if (!ground.valid) {
      return;
    }

    const bool palacin = (classify_mode_ != "min_z" && classify_mode_ != "legacy");
    const double band = ground_height_band_m_;
    const double hole_d = hole_depth_m_;
    const double obs_d = obstacle_height_m_;

    for (auto & pt : points) {
      if (!pt.beam_valid) {
        continue;
      }

      const double dz = forest_lidar_preprocess_cpp::residual_z(ground, pt.x_class, pt.z_class);

      if (palacin) {
        if (dz < -hole_d) {
          pt.label = kLabelHole;
        } else if (dz > obs_d) {
          pt.label = kLabelObstacle;
        } else if (std::abs(dz) <= band) {
          pt.label = kLabelGround;
        } else {
          pt.label = kLabelOther;
        }
      } else {
        if (std::abs(dz) <= band) {
          pt.label = kLabelGround;
        } else {
          pt.label = kLabelOther;
        }
      }
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    geometry_msgs::msg::TransformStamped tf;
    const bool have_tf = lookup_transform(
      classification_frame_, cloud->header.frame_id,
      rclcpp::Time(cloud->header.stamp), tf);

    std::vector<ClassifiedPoint> points;
    const std::size_t n_pts = static_cast<std::size_t>(cloud->width) * cloud->height;
    const std::size_t cap = static_cast<std::size_t>(
      std::max(1000, max_points_per_cloud_));
    points.reserve(std::min(n_pts, cap));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

    std::size_t seen = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (seen >= cap) {
        break;
      }
      ++seen;

      const float lx = *iter_x;
      const float ly = *iter_y;
      const float lz = *iter_z;
      const float r = std::sqrt(lx * lx + ly * ly + lz * lz);

      ClassifiedPoint pt;
      pt.range = r;
      pt.x_laser = lx;
      pt.y_laser = ly;

      if (!std::isfinite(r) || r < static_cast<float>(min_range_m_) ||
        r > static_cast<float>(max_range_m_))
      {
        points.push_back(pt);
        continue;
      }

      if (r > static_cast<float>(classify_near_max_range_m_)) {
        points.push_back(pt);
        continue;
      }

      pt.beam_valid = true;
      if (have_tf) {
        transform_point(tf, lx, ly, lz, pt.x_class, pt.y_class, pt.z_class);
      } else {
        const double r_xy = std::hypot(static_cast<double>(lx), static_cast<double>(ly));
        pt.x_class = lx;
        pt.y_class = ly;
        pt.z_class = fallback_laser_height_m_ - r_xy * std::tan(fallback_laser_pitch_rad_);
        (void)lz;
      }
      points.push_back(pt);
    }

    const auto ground_line = estimate_ground_line(points);
    if (!ground_line.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "3D: no ground line (%zu near valid / %zu total) — skip labeled cloud",
        std::count_if(points.begin(), points.end(),
          [](const ClassifiedPoint & p) { return p.beam_valid; }),
        points.size());
      return;
    }

    classify_points(points, ground_line);

    if (have_tf) {
      publish_labeled_cloud_from_points(cloud->header, points, true);
    }

    std::size_t n_g = 0;
    std::size_t n_near = 0;
    for (const auto & pt : points) {
      if (pt.beam_valid) {
        ++n_near;
      }
      if (pt.label == kLabelGround) {
        ++n_g;
      }
    }
    RCLCPP_DEBUG(
      get_logger(),
      "3D classified near=%zu ground=%zu line m=%.4f b=%.3f inliers=%zu",
      n_near, n_g, ground_line.m, ground_line.b, ground_line.inliers);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::TransformStamped tf;
    const bool have_tf = lookup_transform(
      classification_frame_, scan->header.frame_id,
      rclcpp::Time(scan->header.stamp), tf);

    std::vector<ClassifiedPoint> points;
    points.reserve(scan->ranges.size());

    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
      const float r = scan->ranges[i];
      ClassifiedPoint pt;
      pt.range = r;

      if (!std::isfinite(r) || r < static_cast<float>(min_range_m_) ||
        r > static_cast<float>(max_range_m_))
      {
        points.push_back(pt);
        continue;
      }

      pt.beam_valid = true;
      const float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
      pt.x_laser = r * std::cos(angle);
      pt.y_laser = r * std::sin(angle);

      if (have_tf) {
        transform_point(tf, pt.x_laser, pt.y_laser, 0.0, pt.x_class, pt.y_class, pt.z_class);
      } else {
        fallback_transform(pt.x_laser, pt.y_laser, pt.x_class, pt.y_class, pt.z_class);
      }
      points.push_back(pt);
    }

    const auto ground_line = estimate_ground_line(points);
    if (!ground_line.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "No ground line fit (%zu beams valid) — publishing empty classification",
        std::count_if(points.begin(), points.end(),
          [](const ClassifiedPoint & p) { return p.beam_valid; }));
      publish_empty_outputs(*scan);
      return;
    }

    classify_points(points, ground_line);

    // Do not publish PointCloud2 until TF is valid — RViz Ogre crashes on transform
    // failure when use_sim_time is active but /clock/TF are not ready yet.
    if (have_tf) {
      publish_labeled_cloud(*scan, points, true);
    }
    publish_split_scans(*scan, points);

    std::size_t n_g = 0;
    std::size_t n_hole = 0;
    std::size_t n_obs = 0;
    std::size_t n_other = 0;
    for (const auto & pt : points) {
      switch (pt.label) {
        case kLabelGround: ++n_g; break;
        case kLabelHole: ++n_hole; break;
        case kLabelObstacle: ++n_obs; break;
        case kLabelOther: ++n_other; break;
        default: break;
      }
    }
    RCLCPP_DEBUG(
      get_logger(),
      "classified ground=%zu hole=%zu obstacle=%zu other=%zu line m=%.4f b=%.3f inliers=%zu",
      n_g, n_hole, n_obs, n_other, ground_line.m, ground_line.b, ground_line.inliers);
  }

  void publish_labeled_cloud(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<ClassifiedPoint> & points, bool use_class_frame) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = scan.header.stamp;
    cloud.header.frame_id =
      use_class_frame ? classification_frame_ : scan.header.frame_id;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "label", 1, sensor_msgs::msg::PointField::UINT8);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(cloud, "label");

    for (const auto & pt : points) {
      if (use_class_frame) {
        *iter_x = static_cast<float>(pt.x_class);
        *iter_y = static_cast<float>(pt.y_class);
        *iter_z = static_cast<float>(pt.z_class);
      } else {
        *iter_x = pt.x_laser;
        *iter_y = pt.y_laser;
        *iter_z = 0.0f;
      }
      *iter_label = pt.label;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_label;
    }

    cloud.width = static_cast<uint32_t>(points.size());
    cloud.row_step = cloud.point_step * cloud.width;
    pub_labeled_->publish(cloud);
  }

  void publish_labeled_cloud_from_points(
    const std_msgs::msg::Header & hdr,
    const std::vector<ClassifiedPoint> & points, bool use_class_frame) const
  {
    const std::size_t n_valid = static_cast<std::size_t>(std::count_if(
      points.begin(), points.end(),
      [](const ClassifiedPoint & p) { return p.beam_valid; }));

    std::string out_frame =
      use_class_frame ? classification_frame_ : hdr.frame_id;
    geometry_msgs::msg::TransformStamped tf_to_map;
    const bool publish_in_map =
      use_class_frame && labeled_cloud_output_frame_ == "map" &&
      lookup_transform(
        "map", classification_frame_, rclcpp::Time(hdr.stamp), tf_to_map);
    if (publish_in_map) {
      out_frame = "map";
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = hdr;
    cloud.header.frame_id = out_frame;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "label", 1, sensor_msgs::msg::PointField::UINT8);
    modifier.resize(n_valid);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(cloud, "label");

    for (const auto & pt : points) {
      if (!pt.beam_valid) {
        continue;
      }
      double ox = pt.x_class;
      double oy = pt.y_class;
      double oz = pt.z_class;
      if (publish_in_map) {
        transform_point(tf_to_map, ox, oy, oz, ox, oy, oz);
      }
      *iter_x = static_cast<float>(ox);
      *iter_y = static_cast<float>(oy);
      *iter_z = static_cast<float>(oz);
      *iter_label = pt.label;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_label;
    }

    cloud.width = static_cast<uint32_t>(n_valid);
    cloud.row_step = cloud.point_step * cloud.width;
    pub_labeled_->publish(cloud);
  }

  void publish_split_scans(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<ClassifiedPoint> & points) const
  {
    auto make_scan = [&scan]() {
      sensor_msgs::msg::LaserScan out = scan;
      out.ranges.assign(scan.ranges.size(), std::numeric_limits<float>::quiet_NaN());
      if (scan.intensities.size() == scan.ranges.size()) {
        out.intensities.assign(scan.intensities.size(), 0.0f);
      }
      return out;
    };

    sensor_msgs::msg::LaserScan ground = make_scan();
    sensor_msgs::msg::LaserScan other = make_scan();
    sensor_msgs::msg::LaserScan hazard = make_scan();

    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto & pt = points[i];
      switch (pt.label) {
        case kLabelGround:
          ground.ranges[i] = pt.range;
          if (!ground.intensities.empty()) {
            ground.intensities[i] = static_cast<float>(kLabelGround);
          }
          break;
        case kLabelHole:
        case kLabelObstacle:
          hazard.ranges[i] = pt.range;
          if (!hazard.intensities.empty()) {
            hazard.intensities[i] = static_cast<float>(pt.label);
          }
          break;
        case kLabelOther:
          other.ranges[i] = pt.range;
          if (!other.intensities.empty()) {
            other.intensities[i] = static_cast<float>(kLabelOther);
          }
          break;
        default:
          break;
      }
    }

    pub_ground_->publish(ground);
    pub_other_->publish(other);
    pub_hazard_->publish(hazard);
  }

  void publish_empty_outputs(const sensor_msgs::msg::LaserScan & scan) const
  {
    std::vector<ClassifiedPoint> empty(scan.ranges.size());
    for (auto & pt : empty) {
      pt.label = kLabelInvalid;
    }
    // Skip points_labeled cloud — empty 720-point clouds still stress RViz drawables.
    publish_split_scans(scan, empty);
  }

  std::string input_type_;
  std::string scan_topic_;
  std::string pointcloud_topic_;
  double classify_near_max_range_m_{};
  int max_points_per_cloud_{};
  std::string classification_frame_;
  std::string classify_mode_;
  std::string labeled_cloud_topic_;
  std::string labeled_cloud_output_frame_;
  std::string scan_ground_topic_;
  std::string scan_other_topic_;
  std::string scan_hazard_topic_;
  double min_range_m_{};
  double max_range_m_{};
  double forward_min_x_m_{};
  double ground_height_band_m_{};
  double hole_depth_m_{};
  double obstacle_height_m_{};
  double ground_z_offset_m_{};
  double ransac_inlier_thresh_m_{};
  int ransac_max_iterations_{};
  int ransac_min_inliers_{};
  double ransac_min_inlier_ratio_{};
  double tf_timeout_sec_{};
  double fallback_laser_pitch_rad_{};
  double fallback_laser_height_m_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_labeled_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_other_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_hazard_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScanClassifyNode>());
  rclcpp::shutdown();
  return 0;
}

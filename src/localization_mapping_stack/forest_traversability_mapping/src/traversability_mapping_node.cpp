// Camada de mapa de custo / traversabilidade 2.5D.
//
// Mantem um grid_map local rolante centrado na pose do robo e publica:
//   /mapping/traversability_map        (grid_map_msgs/GridMap)
//   /mapping/traversability_costmap    (nav_msgs/OccupancyGrid)  — camada `cost`
//
// Sprint 2 + robustez de terreno: a camada `elevation` e a malha de solo 2.5D que
// o robo usa, fundida do ground CSF (/perception/lidar3d/experimental/ground).
// Robustez contra os picos que aparecem em movimento:
//   (1) Compensacao de atitude via IMU — a EKF corre em two_d_mode (pitch/roll/z=0
//       na TF); usamos o pitch/roll reais do IMU para projetar a nuvem direita.
//   (2) Range gating — so funde pontos perto (erro de altura ~ range*sin(dpitch)).
//   (3) Kalman 1D por celula + gate de outliers (var de medicao cresce com o range).
//   (4) Mediana 3x3 espacial -> camada `elevation_smooth` (publicada/visualizada).
// Sprint 1 (a seguir): camada `cost` a partir dos clusters de obstaculos.
// Ver docs/TRAVERSABILITY_COSTMAP_PLAN.md.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class TraversabilityMappingNode : public rclcpp::Node
{
public:
  TraversabilityMappingNode()
  : Node("traversability_mapping_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "marble_hd2/base_link");
    cost_layer_ = declare_parameter<std::string>("cost_layer", "cost");
    ground_topic_ = declare_parameter<std::string>(
      "ground_topic", "/perception/lidar3d/experimental/ground");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensors/imu/data");
    use_imu_attitude_ = declare_parameter<bool>("use_imu_attitude", true);
    const double size_x = declare_parameter<double>("size_x_m", 20.0);
    const double size_y = declare_parameter<double>("size_y_m", 20.0);
    resolution_ = declare_parameter<double>("resolution_m", 0.25);
    publish_hz_ = declare_parameter<double>("publish_hz", 5.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);
    cost_max_ = declare_parameter<double>("cost_max", 1.0);
    // Robustez de terreno
    max_insert_range_ = declare_parameter<double>("max_insert_range_m", 6.0);
    meas_var_base_ = declare_parameter<double>("meas_var_base", 0.0009);      // (3 cm)^2
    meas_var_range_k_ = declare_parameter<double>("meas_var_range_k", 0.025); // sigma ~ k*range
    outlier_gate_sigma_ = declare_parameter<double>("outlier_gate_sigma", 3.0);
    var_min_ = declare_parameter<double>("var_min", 0.0009);
    smooth_enable_ = declare_parameter<bool>("smooth_enable", true);

    map_ = grid_map::GridMap({"elevation", "elevation_smooth", "variance", cost_layer_});
    map_.setFrameId(map_frame_);
    map_.setGeometry(
      grid_map::Length(size_x, size_y), resolution_, grid_map::Position(0.0, 0.0));
    reset_layers();

    pub_grid_ = create_publisher<grid_map_msgs::msg::GridMap>(
      "/mapping/traversability_map", rclcpp::QoS(1).transient_local());
    pub_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/mapping/traversability_costmap", rclcpp::QoS(1).transient_local());

    sub_ground_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityMappingNode::on_ground, this, std::placeholders::_1));
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityMappingNode::on_imu, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TraversabilityMappingNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "traversability_mapping: %.1fx%.1f m @ %.2f m, frame=%s, ground=%s, imu_attitude=%s",
      size_x, size_y, resolution_, map_frame_.c_str(), ground_topic_.c_str(),
      use_imu_attitude_ ? "on" : "off");
  }

private:
  void reset_layers()
  {
    map_["elevation"].setConstant(NAN);
    map_["elevation_smooth"].setConstant(NAN);
    map_["variance"].setConstant(NAN);
    map_[cost_layer_].setZero();
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    if (q.length2() < 1e-6) {
      return;  // orientacao nao fornecida
    }
    double yaw;
    tf2::Matrix3x3(q).getRPY(imu_roll_, imu_pitch_, yaw);
    have_imu_ = true;
  }

  // Kalman 1D por celula: funde uma medicao de altura z com variancia R.
  void update_cell(const grid_map::Index & idx, float z, float meas_var)
  {
    float & h = map_.at("elevation", idx);
    float & v = map_.at("variance", idx);
    if (std::isnan(h)) {  // primeira observacao (ou celula nova pos-move)
      h = z;
      v = meas_var;
      return;
    }
    const float innovation = z - h;
    const float s = v + meas_var;
    // Gate de outliers: rejeita picos espurios a > N sigma.
    if (innovation * innovation > outlier_gate_sigma_ * outlier_gate_sigma_ * s) {
      return;
    }
    const float k = v / s;
    h += k * innovation;
    v = std::max(static_cast<float>((1.0f - k) * v), static_cast<float>(var_min_));
  }

  // Transformada odom<-sensor com pitch/roll reais do IMU (a TF da EKF e 2D).
  bool corrected_transform(
    const std_msgs::msg::Header & cloud_header, geometry_msgs::msg::TransformStamped & out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, cloud_header.frame_id, cloud_header.stamp,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException &) {
      try {
        tf = tf_buffer_.lookupTransform(
          map_frame_, cloud_header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_sec_));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "ground TF %s -> %s: %s", map_frame_.c_str(), cloud_header.frame_id.c_str(),
          ex.what());
        return false;
      }
    }
    out = tf;
    if (use_imu_attitude_ && have_imu_) {
      // Mantem xy/z e yaw da odometria; substitui roll/pitch pelos do IMU.
      tf2::Quaternion q_tf(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      double r, p, yaw;
      tf2::Matrix3x3(q_tf).getRPY(r, p, yaw);
      tf2::Quaternion q_corr;
      q_corr.setRPY(imu_roll_, imu_pitch_, yaw);
      out.transform.rotation.x = q_corr.x();
      out.transform.rotation.y = q_corr.y();
      out.transform.rotation.z = q_corr.z();
      out.transform.rotation.w = q_corr.w();
    }
    robot_x_ = out.transform.translation.x;
    robot_y_ = out.transform.translation.y;
    return true;
  }

  void on_ground(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    if (!corrected_transform(msg->header, tf)) {
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    tf2::doTransform(*msg, cloud, tf);

    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");
    const double r2_max = max_insert_range_ * max_insert_range_;
    size_t used = 0;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
        continue;
      }
      // Range gating (distancia horizontal ao robo): erro de altura cresce com o range.
      const double dx = *ix - robot_x_;
      const double dy = *iy - robot_y_;
      const double r2 = dx * dx + dy * dy;
      if (r2 > r2_max) {
        continue;
      }
      const grid_map::Position pos(*ix, *iy);
      if (!map_.isInside(pos)) {
        continue;
      }
      grid_map::Index idx;
      map_.getIndex(pos, idx);
      const double range = std::sqrt(r2);
      const float meas_var =
        static_cast<float>(meas_var_base_ + std::pow(meas_var_range_k_ * range, 2.0));
      update_cell(idx, *iz, meas_var);
      ++used;
    }
    last_ground_used_ = used;
  }

  // Mediana 3x3 sobre `elevation` -> `elevation_smooth` (remove picos isolados).
  void smooth_elevation()
  {
    const grid_map::Matrix & elev = map_["elevation"];
    grid_map::Matrix & out = map_["elevation_smooth"];
    const int rows = elev.rows();
    const int cols = elev.cols();
    std::vector<float> win;
    win.reserve(9);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        if (std::isnan(elev(i, j))) {
          out(i, j) = NAN;
          continue;
        }
        win.clear();
        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            const int ni = i + di;
            const int nj = j + dj;
            if (ni < 0 || nj < 0 || ni >= rows || nj >= cols) {
              continue;
            }
            const float val = elev(ni, nj);
            if (!std::isnan(val)) {
              win.push_back(val);
            }
          }
        }
        std::nth_element(win.begin(), win.begin() + win.size() / 2, win.end());
        out(i, j) = win[win.size() / 2];
      }
    }
  }

  void on_timer()
  {
    geometry_msgs::msg::TransformStamped tf;
    bool have_pose = false;
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      have_pose = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "traversability TF %s -> %s: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
    }

    if (have_pose) {
      map_.move(grid_map::Position(
        tf.transform.translation.x, tf.transform.translation.y));
    }

    if (smooth_enable_) {
      smooth_elevation();
    } else {
      map_["elevation_smooth"] = map_["elevation"];
    }

    const rclcpp::Time stamp = have_pose ? rclcpp::Time(tf.header.stamp) : now();
    map_.setTimestamp(stamp.nanoseconds());

    std::unique_ptr<grid_map_msgs::msg::GridMap> grid_msg =
      grid_map::GridMapRosConverter::toMessage(map_);
    pub_grid_->publish(*grid_msg);

    nav_msgs::msg::OccupancyGrid occ;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_, cost_layer_, 0.0, cost_max_, occ);
    occ.header.stamp = stamp;
    pub_costmap_->publish(occ);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "elevation: %zu pontos fundidos/scan, imu_attitude=%s",
      last_ground_used_, (use_imu_attitude_ && have_imu_) ? "on" : "off/sem-imu");
  }

  std::string map_frame_, base_frame_, cost_layer_, ground_topic_, imu_topic_;
  bool use_imu_attitude_{true};
  double resolution_{}, publish_hz_{}, tf_timeout_sec_{}, cost_max_{};
  double max_insert_range_{}, meas_var_base_{}, meas_var_range_k_{};
  double outlier_gate_sigma_{}, var_min_{};
  bool smooth_enable_{true};

  bool have_imu_{false};
  double imu_roll_{0.0}, imu_pitch_{0.0};
  double robot_x_{0.0}, robot_y_{0.0};
  size_t last_ground_used_{0};

  grid_map::GridMap map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityMappingNode>());
  rclcpp::shutdown();
  return 0;
}

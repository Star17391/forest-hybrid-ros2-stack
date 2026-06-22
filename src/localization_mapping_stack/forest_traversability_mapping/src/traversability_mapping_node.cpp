// Camada de mapa de custo / traversabilidade 2.5D.
//
// Mantem um grid_map local rolante centrado na pose do robo e publica:
//   /mapping/traversability_map        (grid_map_msgs/GridMap)
//   /mapping/traversability_costmap    (nav_msgs/OccupancyGrid)  — camada `cost`
//
// Sprint 2 + robustez de terreno: a camada `elevation` e a malha de solo 2.5D que
// o robo usa, fundida do ground CSF (/perception/lidar3d/experimental/ground).
// Robustez contra os picos que aparecem em movimento:
//   (1) Compensacao de atitude via IMU (use_imu_attitude) — LEGADO do tempo em que
//       a EKF corria em two_d_mode (pitch/roll/z=0 na TF). Com o EKF SE3 actual a
//       TF odom→base JA traz roll/pitch correctos, logo re-substituir pelo IMU seria
//       DUPLA correcao. Default agora FALSE: confia-se na TF SE3. Manter so como
//       fallback de diagnostico se a TF voltar a ser 2D.
//   (2) Range gating — so funde pontos perto (erro de altura ~ range*sin(dpitch)).
//   (3) Kalman 1D por celula + gate de outliers (var de medicao cresce com o range).
//   (4) Mediana 3x3 espacial -> camada `elevation_smooth` (publicada/visualizada).
//
// Sprint 1 (este): camada `cost` = max(custo de terreno, custo de obstaculos).
//   1A. Custo de terreno: declive (gradiente da elevation_smooth) + degrau/buraco
//       (salto de altura na vizinhanca) -> layer `cost_terrain`.
//   1B. Troncos como cilindros: subscreve /perception/lidar/tree_landmarks
//       (TreeLandmarkArray, ja parametrizado em base_link, per-frame stateless),
//       transforma a base para odom, mantem primitivas CYLINDER com margem larga
//       fixa + TTL + dedup por proximidade, e renderiza no `cost_obstacle` com
//       inflacao. Publica os cilindros em /mapping/world_model (markers castanhos).
//   Rochas/obstaculos genericos como BOX (clustering dos semantic_points) ficam
//   para o Sprint 3 (tracking/confianca). Ver docs/TRAVERSABILITY_COSTMAP_PLAN.md.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "forest_hybrid_msgs/msg/tree_landmark_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    // Default FALSE: o EKF SE3 já põe roll/pitch na TF odom→base; reativar isto
    // dupla-corrige a atitude. Só usar se a TF voltar a ser 2D (two_d_mode).
    use_imu_attitude_ = declare_parameter<bool>("use_imu_attitude", false);
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
    defragment_buffer_ = declare_parameter<bool>("defragment_buffer", true);
    // Sprint 1A — custo de terreno
    enable_terrain_cost_ = declare_parameter<bool>("enable_terrain_cost", true);
    slope_cost_max_deg_ = declare_parameter<double>("slope_cost_max_deg", 25.0);
    step_obstacle_m_ = declare_parameter<double>("step_obstacle_m", 0.35);
    // Sprint 1B — troncos como cilindros
    enable_obstacle_cost_ = declare_parameter<bool>("enable_obstacle_cost", true);
    tree_landmarks_topic_ = declare_parameter<std::string>(
      "tree_landmarks_topic", "/perception/lidar/tree_landmarks");
    trunk_margin_m_ = declare_parameter<double>("trunk_margin_m", 0.30);
    obstacle_inflation_m_ = declare_parameter<double>("obstacle_inflation_m", 0.40);
    obstacle_ttl_sec_ = declare_parameter<double>("obstacle_ttl_sec", 2.0);
    dedup_dist_m_ = declare_parameter<double>("dedup_dist_m", 0.30);
    min_trunk_confidence_ = declare_parameter<double>("min_trunk_confidence", 0.0);

    map_ = grid_map::GridMap(
      {"elevation", "elevation_smooth", "variance",
       "cost_terrain", "cost_obstacle", cost_layer_});
    map_.setFrameId(map_frame_);
    map_.setGeometry(
      grid_map::Length(size_x, size_y), resolution_, grid_map::Position(0.0, 0.0));
    reset_layers();

    pub_grid_ = create_publisher<grid_map_msgs::msg::GridMap>(
      "/mapping/traversability_map", rclcpp::QoS(1).transient_local());
    pub_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/mapping/traversability_costmap", rclcpp::QoS(1).transient_local());
    pub_world_model_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/mapping/world_model", rclcpp::QoS(1).transient_local());

    sub_ground_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityMappingNode::on_ground, this, std::placeholders::_1));
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityMappingNode::on_imu, this, std::placeholders::_1));
    sub_trees_ = create_subscription<forest_hybrid_msgs::msg::TreeLandmarkArray>(
      tree_landmarks_topic_, rclcpp::QoS(10),
      std::bind(&TraversabilityMappingNode::on_tree_landmarks, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TraversabilityMappingNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "traversability_mapping: %.1fx%.1f m @ %.2f m, frame=%s, ground=%s, imu_attitude=%s, "
      "terrain_cost=%s, obstacle_cost=%s (trees=%s)",
      size_x, size_y, resolution_, map_frame_.c_str(), ground_topic_.c_str(),
      use_imu_attitude_ ? "on" : "off", enable_terrain_cost_ ? "on" : "off",
      enable_obstacle_cost_ ? "on" : "off", tree_landmarks_topic_.c_str());
  }

private:
  // Primitiva de obstaculo (Sprint 1B: so cilindro/tronco; caixa/rocha no Sprint 3).
  struct Obstacle
  {
    double x{0.0};            // centro em odom [m]
    double y{0.0};
    double radius{0.0};       // raio fisico (tronco: DBH/2) [m]
    double confidence{0.0};
    rclcpp::Time last_seen;
  };

  void reset_layers()
  {
    map_["elevation"].setConstant(NAN);
    map_["elevation_smooth"].setConstant(NAN);
    map_["variance"].setConstant(NAN);
    map_["cost_terrain"].setZero();
    map_["cost_obstacle"].setZero();
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

  // Sprint 1B — troncos: cada TreeLandmark (base_link, per-frame) -> primitiva
  // CYLINDER em odom, com dedup por proximidade (atualiza o mais perto em vez de
  // duplicar) e TTL (purgado no timer). Margem larga fixa; estreitar = Sprint 3.
  void on_tree_landmarks(const forest_hybrid_msgs::msg::TreeLandmarkArray::SharedPtr msg)
  {
    if (!enable_obstacle_cost_ || msg->trees.empty()) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, msg->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "tree_landmarks TF %s -> %s: %s", map_frame_.c_str(),
        msg->header.frame_id.c_str(), ex.what());
      return;
    }
    // TTL/idade usam o relogio do no (consistente com sim_time), nao o stamp da
    // mensagem nem da TF — evita idades negativas por desalinhamento de stamps.
    const rclcpp::Time stamp = now();
    for (const auto & tree : msg->trees) {
      if (tree.confidence < min_trunk_confidence_) {
        continue;
      }
      geometry_msgs::msg::PointStamped in, out;
      in.header = msg->header;
      in.point = tree.base;
      tf2::doTransform(in, out, tf);

      const double radius = std::max(0.05, 0.5 * static_cast<double>(tree.diameter));
      // Dedup: se ja existe primitiva perto, refina (media) em vez de duplicar.
      Obstacle * match = nullptr;
      double best = dedup_dist_m_ * dedup_dist_m_;
      for (auto & o : obstacles_) {
        const double dx = o.x - out.point.x;
        const double dy = o.y - out.point.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) {
          best = d2;
          match = &o;
        }
      }
      if (match) {
        const double a = 0.5;  // suavizacao simples (nao e tracking completo)
        match->x = (1 - a) * match->x + a * out.point.x;
        match->y = (1 - a) * match->y + a * out.point.y;
        match->radius = (1 - a) * match->radius + a * radius;
        match->confidence = std::max<double>(match->confidence, tree.confidence);
        match->last_seen = stamp;
      } else {
        Obstacle o;
        o.x = out.point.x;
        o.y = out.point.y;
        o.radius = radius;
        o.confidence = tree.confidence;
        o.last_seen = stamp;
        obstacles_.push_back(o);
      }
    }
  }

  // Sprint 1A — custo de terreno a partir da elevation_smooth:
  //   declive (gradiente central) normalizado por slope_cost_max_deg, e
  //   degrau/buraco (salto de altura na vizinhanca > step_obstacle_m -> custo 1).
  void compute_terrain_cost()
  {
    map_["cost_terrain"].setZero();
    if (!enable_terrain_cost_) {
      return;
    }
    const grid_map::Matrix & elev = map_["elevation_smooth"];
    grid_map::Matrix & cost = map_["cost_terrain"];
    const int rows = elev.rows();
    const int cols = elev.cols();
    const double res = resolution_;
    const double slope_norm =
      std::tan(slope_cost_max_deg_ * M_PI / 180.0);  // tan(theta) maximo = custo 1
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        const float c = elev(i, j);
        if (std::isnan(c)) {
          cost(i, j) = 0.0f;
          continue;
        }
        double max_abs_step = 0.0;
        double gx = 0.0;
        double gy = 0.0;
        bool have_x = false;
        bool have_y = false;
        // Gradiente central + maior salto na vizinhanca 4-conexa.
        if (i > 0 && i < rows - 1 && !std::isnan(elev(i - 1, j)) &&
          !std::isnan(elev(i + 1, j)))
        {
          gx = (elev(i + 1, j) - elev(i - 1, j)) / (2.0 * res);
          have_x = true;
        }
        if (j > 0 && j < cols - 1 && !std::isnan(elev(i, j - 1)) &&
          !std::isnan(elev(i, j + 1)))
        {
          gy = (elev(i, j + 1) - elev(i, j - 1)) / (2.0 * res);
          have_y = true;
        }
        for (int d = 0; d < 4; ++d) {
          static const int di[4] = {-1, 1, 0, 0};
          static const int dj[4] = {0, 0, -1, 1};
          const int ni = i + di[d];
          const int nj = j + dj[d];
          if (ni < 0 || nj < 0 || ni >= rows || nj >= cols) {
            continue;
          }
          const float nv = elev(ni, nj);
          if (!std::isnan(nv)) {
            max_abs_step = std::max(max_abs_step, std::abs(static_cast<double>(nv - c)));
          }
        }
        double slope_cost = 0.0;
        if (have_x || have_y) {
          const double grad = std::sqrt(gx * gx + gy * gy);  // tan(theta)
          slope_cost = std::clamp(grad / slope_norm, 0.0, 1.0);
        }
        const double step_cost = (max_abs_step > step_obstacle_m_) ? 1.0 : 0.0;
        cost(i, j) = static_cast<float>(std::max(slope_cost, step_cost));
      }
    }
  }

  // Sprint 1B — render das primitivas de tronco no cost_obstacle: disco de
  // custo 1 ate ao raio, decrescendo linearmente na inflacao (raio..raio+margem+infl).
  void render_obstacles(const rclcpp::Time & now_stamp)
  {
    map_["cost_obstacle"].setZero();
    // Purga primitivas velhas (TTL) ou fora da janela do mapa.
    const double ttl = obstacle_ttl_sec_;
    obstacles_.erase(
      std::remove_if(
        obstacles_.begin(), obstacles_.end(),
        [&](const Obstacle & o) {
          const double age = (now_stamp - o.last_seen).seconds();
          if (age > ttl || age < -1.0) {
            return true;
          }
          return !map_.isInside(grid_map::Position(o.x, o.y));
        }),
      obstacles_.end());

    if (!enable_obstacle_cost_) {
      return;
    }
    grid_map::Matrix & cost = map_["cost_obstacle"];
    for (const auto & o : obstacles_) {
      const double r_hard = o.radius;
      const double r_soft = o.radius + trunk_margin_m_ + obstacle_inflation_m_;
      grid_map::Position center(o.x, o.y);
      for (grid_map::CircleIterator it(map_, center, r_soft); !it.isPastEnd(); ++it) {
        grid_map::Position p;
        map_.getPosition(*it, p);
        const double d = (p - center).norm();
        double v;
        if (d <= r_hard + trunk_margin_m_) {
          v = 1.0;
        } else {
          const double t = (d - (r_hard + trunk_margin_m_)) / std::max(1e-3, obstacle_inflation_m_);
          v = std::clamp(1.0 - t, 0.0, 1.0);
        }
        float & cell = cost(getRow(*it), getCol(*it));
        cell = std::max(cell, static_cast<float>(v));
      }
    }
  }

  static int getRow(const grid_map::Index & idx) { return idx(0); }
  static int getCol(const grid_map::Index & idx) { return idx(1); }

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

    // CRITICO: o grid_map e um buffer circular; move() so desloca o startIndex.
    // As operacoes de vizinhanca abaixo (mediana 3x3, gradiente do custo) acedem a
    // matriz com indices fisicos (i+-1, j+-1) e assumem que sao vizinhos espaciais.
    // Sem isto, a "costura" do buffer cria linhas de defeito no mapa em movimento.
    // convertToDefaultStartIndex() desenrola o buffer (startIndex -> 0,0).
    if (defragment_buffer_) {
      map_.convertToDefaultStartIndex();
    }

    if (smooth_enable_) {
      smooth_elevation();
    } else {
      map_["elevation_smooth"] = map_["elevation"];
    }

    const rclcpp::Time stamp = have_pose ? rclcpp::Time(tf.header.stamp) : now();

    // Sprint 1 — custo: terreno + obstaculos -> cost = max(...).
    compute_terrain_cost();
    render_obstacles(now());  // TTL no relogio do no (ver on_tree_landmarks)
    map_[cost_layer_] = map_["cost_terrain"].cwiseMax(map_["cost_obstacle"]);

    map_.setTimestamp(stamp.nanoseconds());

    std::unique_ptr<grid_map_msgs::msg::GridMap> grid_msg =
      grid_map::GridMapRosConverter::toMessage(map_);
    pub_grid_->publish(*grid_msg);

    nav_msgs::msg::OccupancyGrid occ;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_, cost_layer_, 0.0, cost_max_, occ);
    occ.header.stamp = stamp;
    pub_costmap_->publish(occ);

    publish_world_model(stamp);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "elevation: %zu pontos/scan, imu=%s | troncos=%zu (cost terrain+obst)",
      last_ground_used_, (use_imu_attitude_ && have_imu_) ? "on" : "off/sem-imu",
      obstacles_.size());
  }

  // Cilindros castanhos por primitiva de tronco; altura/alpha ∝ confianca.
  void publish_world_model(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto & o : obstacles_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = stamp;
      m.ns = "world_model_trunks";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = o.x;
      m.pose.position.y = o.y;
      m.pose.position.z = 0.75;  // meia altura nominal (sem elevacao do solo no S1)
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = std::max(0.05, 2.0 * o.radius);
      m.scale.z = 1.5;
      m.color.r = 0.40f;  // castanho
      m.color.g = 0.26f;
      m.color.b = 0.13f;
      m.color.a = static_cast<float>(std::clamp(0.35 + 0.6 * o.confidence, 0.35, 1.0));
      arr.markers.push_back(m);
    }
    pub_world_model_->publish(arr);
  }

  std::string map_frame_, base_frame_, cost_layer_, ground_topic_, imu_topic_;
  std::string tree_landmarks_topic_;
  bool use_imu_attitude_{true};
  double resolution_{}, publish_hz_{}, tf_timeout_sec_{}, cost_max_{};
  double max_insert_range_{}, meas_var_base_{}, meas_var_range_k_{};
  double outlier_gate_sigma_{}, var_min_{};
  bool smooth_enable_{true};
  bool defragment_buffer_{true};
  // Sprint 1
  bool enable_terrain_cost_{true}, enable_obstacle_cost_{true};
  double slope_cost_max_deg_{}, step_obstacle_m_{};
  double trunk_margin_m_{}, obstacle_inflation_m_{}, obstacle_ttl_sec_{};
  double dedup_dist_m_{}, min_trunk_confidence_{};

  bool have_imu_{false};
  double imu_roll_{0.0}, imu_pitch_{0.0};
  double robot_x_{0.0}, robot_y_{0.0};
  size_t last_ground_used_{0};
  std::vector<Obstacle> obstacles_;

  grid_map::GridMap map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::TreeLandmarkArray>::SharedPtr sub_trees_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_world_model_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityMappingNode>());
  rclcpp::shutdown();
  return 0;
}

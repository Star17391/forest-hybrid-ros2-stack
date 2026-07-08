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
// camada `cost` = max(custo de terreno, custo de obstaculos).
//   Custo de terreno: declive (gradiente da elevation_smooth) + degrau/buraco
//       (salto de altura na vizinhanca) -> layer `cost_terrain`.
//   Custo de obstaculo: dos landmarks COMPROMETIDOS do SLAM (/slam/tree_map, uids
//       estaveis, em frame `map`) -> render de disco letal UNIFORME + inflacao no
//       `cost_obstacle`. Fonte unica = SLAM (persistente, corrigido). A antiga via
//       per-frame stateless (/perception/lidar/tree_landmarks, em odom) foi REMOVIDA.
//   Ver docs/MAPPING_LAYER_REBUILD_PLAN.md.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "forest_hybrid_msgs/msg/tracked_tree_landmark_array.hpp"
#include "forest_tree_slam/tile_map.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
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
    // `map` (SLAM corrigido) — a camada vive no frame do Tree-SLAM via TF map->base
    // para nao derivar com a odometria. Era "odom" (provisorio, pre-SLAM).
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
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
    // Obstaculos: dos landmarks COMPROMETIDOS do SLAM (custo uniforme letal).
    enable_obstacle_cost_ = declare_parameter<bool>("enable_obstacle_cost", true);
    trunk_margin_m_ = declare_parameter<double>("trunk_margin_m", 0.30);
    obstacle_inflation_m_ = declare_parameter<double>("obstacle_inflation_m", 0.40);
    // Parte 3 — MODELO DO MUNDO COMPROMETIDO (cilindros) a partir do mapa SLAM
    // (/slam/tree_map, uids estáveis, só promovidos). Cadência LENTA: um landmark
    // só vira cilindro comprometido quando tem certeza (n_obs + DBH estável + pose
    // pinada). Depois, a posição segue o uid (loop closure) mas o raio é suavizado
    // (histerese) para não tremer. Publicado como PointCloud2 p/ o costmap GLOBAL.
    tree_map_topic_ = declare_parameter<std::string>("tree_map_topic", "/slam/tree_map");
    commit_min_obs_ = declare_parameter<int>("commit_min_obs", 8);
    commit_max_dbh_var_ = declare_parameter<double>("commit_max_dbh_var", 0.01);   // m² (σ≈0.1m)
    commit_max_pos_var_ = declare_parameter<double>("commit_max_pos_var", 0.05);   // m² (traço xy)
    commit_radius_ema_ = declare_parameter<double>("commit_radius_ema", 0.2);      // histerese do raio
    committed_cloud_perim_pts_ = declare_parameter<int>("committed_cloud_perim_pts", 12);

    // Tiles de terreno persistentes (FUTURE_TILED_MAPS.md): mesma particao fixa
    // do forest_tree_slam (tile (0,0) centrado na origem do mundo). O terreno
    // NUNCA e esquecido; a janela de saida e reconstruida dos tiles. Cada tile
    // ancora a keyframe do grafo em vigor na criacao e segue-a no loop closure.
    const double tile_size_m = declare_parameter<double>("tile_size_m", 20.0);
    tile_grid_ = std::make_unique<forest_tree_slam::TileGrid>(tile_size_m);

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
    sub_tree_map_ = create_subscription<forest_hybrid_msgs::msg::TrackedTreeLandmarkArray>(
      tree_map_topic_, rclcpp::QoS(10),
      std::bind(&TraversabilityMappingNode::on_tree_map, this, std::placeholders::_1));
    // Poses otimizadas das keyframes do grafo (indice = id): ancoras dos tiles.
    sub_keyframes_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/slam/keyframes", rclcpp::QoS(10),
      std::bind(&TraversabilityMappingNode::on_keyframes, this, std::placeholders::_1));
    pub_committed_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mapping/committed_obstacles", rclcpp::QoS(1).transient_local());

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TraversabilityMappingNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "traversability_mapping: %.1fx%.1f m @ %.2f m, frame=%s, ground=%s, imu_attitude=%s, "
      "terrain_cost=%s, obstacle_cost=%s (committed do %s)",
      size_x, size_y, resolution_, map_frame_.c_str(), ground_topic_.c_str(),
      use_imu_attitude_ ? "on" : "off", enable_terrain_cost_ ? "on" : "off",
      enable_obstacle_cost_ ? "on" : "off", tree_map_topic_.c_str());
  }

private:
  // Primitiva COMPROMETIDA indexada por uid do SLAM (fonte unica de obstaculos).
  struct Committed
  {
    double x{0.0};
    double y{0.0};
    double radius{0.0};
    std::uint8_t semantic_class{0};
  };

  // Pose SE(2) minima para as ancoras (evita depender do GTSAM aqui).
  struct Se2
  {
    double x{0.0}, y{0.0}, theta{0.0};
  };
  static Se2 se2_inverse(const Se2 & p)
  {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    return {-c * p.x - s * p.y, s * p.x - c * p.y, -p.theta};
  }
  static Se2 se2_compose(const Se2 & a, const Se2 & b)
  {
    const double c = std::cos(a.theta), s = std::sin(a.theta);
    return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.theta + b.theta};
  }
  static Eigen::Vector2d se2_apply(const Se2 & t, const Eigen::Vector2d & p)
  {
    const double c = std::cos(t.theta), s = std::sin(t.theta);
    return {t.x + c * p.x() - s * p.y(), t.y + s * p.x() + c * p.y()};
  }

  // TILE de terreno persistente (FUTURE_TILED_MAPS.md + decisao 2026-07):
  // cobre os limites FIXOS do tile em coordenadas do mundo e NUNCA e esquecido.
  // O conteudo fica ANCORADO a keyframe do grafo (iSAM2) mais recente no
  // momento da criacao: as celulas foram escritas com a estimativa dessa era
  // (pose A0). Quando o loop closure move a ancora para A_now, o tile e lido
  // atraves da correcao rigida delta = A_now ∘ A0⁻¹ — o terreno acompanha a
  // otimizacao do grafo sem re-integrar medicoes. Dentro de um tile o drift e
  // pequeno (troco curto), por isso a correcao rigida e uma boa aproximacao.
  struct TerrainTile
  {
    grid_map::GridMap grid;     // layers: elevation, variance (frame da era A0)
    int anchor_kf{-1};          // id da keyframe-ancora (-1 = pre-SLAM, delta=I)
    Se2 anchor_at_creation{};   // A0: pose da ancora quando o tile nasceu
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
  // Opera sobre o grid do TILE (persistente), nao sobre a janela de saida.
  void update_cell(grid_map::GridMap & g, const grid_map::Index & idx, float z, float meas_var)
  {
    float & h = g.at("elevation", idx);
    float & v = g.at("variance", idx);
    if (std::isnan(h)) {  // primeira observacao
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

  // Tile que contem a posicao (cria-o na 1.ª visita, ancorado a keyframe do
  // SLAM mais recente). A grelha de tiles e FIXA: tile (0,0) centrado na
  // origem do mundo — mesma particao do forest_tree_slam (tile_map.hpp).
  TerrainTile & tile_at(const forest_tree_slam::TileIndex & t)
  {
    auto it = terrain_tiles_.find(t);
    if (it != terrain_tiles_.end()) {
      return it->second;
    }
    TerrainTile tile;
    tile.grid = grid_map::GridMap({"elevation", "variance"});
    tile.grid.setFrameId(map_frame_);
    const Eigen::Vector2d c = tile_grid_->center_of(t);
    tile.grid.setGeometry(
      grid_map::Length(tile_grid_->tile_size(), tile_grid_->tile_size()),
      resolution_, grid_map::Position(c.x(), c.y()));
    tile.grid["elevation"].setConstant(NAN);
    tile.grid["variance"].setConstant(NAN);
    if (!keyframes_.empty()) {
      tile.anchor_kf = static_cast<int>(keyframes_.size()) - 1;
      tile.anchor_at_creation = keyframes_.back();
    }
    auto res = terrain_tiles_.emplace(t, std::move(tile));
    RCLCPP_INFO(
      get_logger(), "terreno: tile novo (%d,%d) ancora=kf%d (total %zu tiles)",
      t.r, t.c, res.first->second.anchor_kf, terrain_tiles_.size());
    return res.first->second;
  }

  // Correcao rigida do tile: delta = A_now ∘ A0⁻¹ (I se a ancora nao mexeu ou
  // o tile e pre-SLAM). A_now vem do /slam/keyframes (poses otimizadas iSAM2).
  Se2 tile_delta(const TerrainTile & tile) const
  {
    if (tile.anchor_kf < 0 ||
      static_cast<std::size_t>(tile.anchor_kf) >= keyframes_.size())
    {
      return {};
    }
    return se2_compose(
      keyframes_[static_cast<std::size_t>(tile.anchor_kf)],
      se2_inverse(tile.anchor_at_creation));
  }

  void on_keyframes(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    keyframes_.clear();
    keyframes_.reserve(msg->poses.size());
    for (const auto & p : msg->poses) {
      const double yaw = 2.0 * std::atan2(p.orientation.z, p.orientation.w);
      keyframes_.push_back({p.position.x, p.position.y, yaw});
    }
  }

  // Reconstrói elevation/variance da janela de saida a partir dos tiles que
  // intersectam a vizinhanca do robo, aplicando a correcao rigida de cada tile.
  // Empate em overlaps: ganha a celula com MENOR variancia (mais confiante).
  void stitch_tiles_into_window(const Eigen::Vector2d & robot)
  {
    map_["elevation"].setConstant(NAN);
    map_["variance"].setConstant(NAN);
    // Raio que garante cobrir a janela inteira a partir do centro (meia
    // diagonal) + margem de um tile para as correcoes rigidas nas bordas.
    const double half_diag = 0.5 * std::hypot(
      map_.getLength().x(), map_.getLength().y());
    const double radius = half_diag + tile_grid_->tile_size();
    for (const auto & t : tile_grid_->tiles_in_radius(robot, radius)) {
      const auto it = terrain_tiles_.find(t);
      if (it == terrain_tiles_.end()) {
        continue;
      }
      const TerrainTile & tile = it->second;
      const Se2 delta = tile_delta(tile);
      const bool identity =
        std::abs(delta.x) < 1e-9 && std::abs(delta.y) < 1e-9 &&
        std::abs(delta.theta) < 1e-9;
      for (grid_map::GridMapIterator gi(tile.grid); !gi.isPastEnd(); ++gi) {
        const float h = tile.grid.at("elevation", *gi);
        if (std::isnan(h)) {
          continue;
        }
        grid_map::Position p;
        tile.grid.getPosition(*gi, p);
        const Eigen::Vector2d q = identity ?
          Eigen::Vector2d(p.x(), p.y()) :
          se2_apply(delta, Eigen::Vector2d(p.x(), p.y()));
        const grid_map::Position out_pos(q.x(), q.y());
        if (!map_.isInside(out_pos)) {
          continue;
        }
        grid_map::Index oi;
        map_.getIndex(out_pos, oi);
        const float v = tile.grid.at("variance", *gi);
        float & oh = map_.at("elevation", oi);
        float & ov = map_.at("variance", oi);
        if (std::isnan(oh) || v < ov) {
          oh = h;
          ov = v;
        }
      }
    }
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
      // Insercao no TILE persistente (grelha fixa), nao na janela de saida:
      // o terreno nunca e esquecido; a janela e reconstruida dos tiles no timer.
      const Eigen::Vector2d pw(*ix, *iy);
      TerrainTile & tile = tile_at(tile_grid_->index_of(pw));
      const grid_map::Position pos(pw.x(), pw.y());
      if (!tile.grid.isInside(pos)) {
        continue;  // borda numerica (ponto exatamente na fronteira)
      }
      grid_map::Index idx;
      tile.grid.getIndex(pos, idx);
      const double range = std::sqrt(r2);
      const float meas_var =
        static_cast<float>(meas_var_base_ + std::pow(meas_var_range_k_ * range, 2.0));
      update_cell(tile.grid, idx, *iz, meas_var);
      ++used;
    }
    last_ground_used_ = used;
  }

  // Parte 3 — MODELO DO MUNDO COMPROMETIDO: cada landmark do /slam/tree_map (uid
  // estável, já promovido) só entra/atualiza quando passa o gate de COMPROMISSO
  // (n_obs + DBH estável + pose pinada). Indexado por uid: a posição segue o SLAM
  // (loop closure move o cilindro); o raio é suavizado por EMA (histerese, sem
  // flicker). committed_class: 3=tronco(CYLINDER), 6=rocha/4=obstáculo(BOX).
  void on_tree_map(const forest_hybrid_msgs::msg::TrackedTreeLandmarkArray::SharedPtr msg)
  {
    // As posições do /slam/tree_map estão no frame do SLAM (map), NÃO no frame da
    // grelha do nó (odom). Guarda o frame correto para publicar a nuvem comprometida.
    if (!msg->header.frame_id.empty()) {
      tree_map_frame_ = msg->header.frame_id;
    }
    for (const auto & t : msg->trees) {
      // covariance é row-major 3x3 sobre (x, y, DBH) em map.
      const double pos_var = t.covariance[0] + t.covariance[4];  // traço xy
      const double dbh_var = t.covariance[8];
      const bool gate_ok =
        static_cast<int>(t.n_observations) >= commit_min_obs_ &&
        dbh_var <= commit_max_dbh_var_ && pos_var <= commit_max_pos_var_;
      auto it = committed_.find(t.uid);
      if (it == committed_.end()) {
        if (!gate_ok) {continue;}  // só nasce comprometido quando há certeza
        Committed c;
        c.x = t.position.x;
        c.y = t.position.y;
        c.radius = std::max(0.05, 0.5 * static_cast<double>(t.diameter));
        c.semantic_class = t.semantic_class;
        committed_[t.uid] = c;
      } else {
        // Já comprometido: a posição SEGUE o SLAM (loop closure); o raio é
        // suavizado (histerese) para não tremer. Não se descompromete.
        it->second.x = t.position.x;
        it->second.y = t.position.y;
        const double a = commit_radius_ema_;
        it->second.radius =
          (1.0 - a) * it->second.radius + a * std::max(0.05, 0.5 * static_cast<double>(t.diameter));
        it->second.semantic_class = t.semantic_class;
      }
    }
  }

  // Publica os obstáculos comprometidos como PointCloud2 (perímetro de cada
  // primitiva amostrado em pontos) p/ o ObstacleLayer do costmap GLOBAL do nav2 os
  // consumir — mesma mecânica do local (ObstacleLayer+PointCloud2), sem tradutor.
  void publish_committed_cloud(const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    // Frame = frame do /slam/tree_map (map), não a grelha do nó (odom); stamp atual
    // para o tf2 message filter do costmap global não dropar por timestamp velho.
    cloud.header.stamp = now();
    cloud.header.frame_id = tree_map_frame_;
    cloud.height = 1;
    (void)stamp;
    const int np = std::max(4, committed_cloud_perim_pts_);
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(committed_.size() * static_cast<std::size_t>(np));
    sensor_msgs::PointCloud2Iterator<float> ox(cloud, "x"), oy(cloud, "y"), oz(cloud, "z");
    for (const auto & kv : committed_) {
      const auto & c = kv.second;
      for (int k = 0; k < np; ++k) {
        const double ang = 2.0 * M_PI * k / np;
        *ox = static_cast<float>(c.x + c.radius * std::cos(ang));
        *oy = static_cast<float>(c.y + c.radius * std::sin(ang));
        *oz = 0.5f;  // altura no plano de varrimento do costmap
        ++ox; ++oy; ++oz;
      }
    }
    pub_committed_->publish(cloud);
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

  // Render dos obstaculos COMPROMETIDOS (do /slam/tree_map) no cost_obstacle:
  // disco de custo 1.0 (letal, UNIFORME para qualquer classe) ate r_hard, decrescendo
  // linearmente na inflacao. Fonte unica = SLAM (persistente, corrigido, em `map`);
  // sem TTL nem dedup — isso e responsabilidade do SLAM, nao desta camada.
  void render_obstacles()
  {
    map_["cost_obstacle"].setZero();
    if (!enable_obstacle_cost_) {
      return;
    }
    grid_map::Matrix & cost = map_["cost_obstacle"];
    for (const auto & kv : committed_) {
      const auto & o = kv.second;
      const double r_hard = o.radius + trunk_margin_m_;
      const double r_soft = r_hard + obstacle_inflation_m_;
      const grid_map::Position center(o.x, o.y);
      for (grid_map::CircleIterator it(map_, center, r_soft); !it.isPastEnd(); ++it) {
        grid_map::Position p;
        map_.getPosition(*it, p);
        const double d = (p - center).norm();
        double v;
        if (d <= r_hard) {
          v = 1.0;  // custo letal uniforme
        } else {
          const double t = (d - r_hard) / std::max(1e-3, obstacle_inflation_m_);
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

    // A janela de saida e RECONSTRUIDA dos tiles persistentes a cada tick
    // (o terreno vive nos tiles, nunca e esquecido; a janela e so a vista
    // local para o custo/Nav2). Cada tile entra pela sua correcao rigida
    // delta = A_now ∘ A0⁻¹ — o loop closure do iSAM2 move a ancora e o
    // terreno acompanha sem re-integrar medicoes.
    if (have_pose) {
      stitch_tiles_into_window(
        Eigen::Vector2d(tf.transform.translation.x, tf.transform.translation.y));
    }

    if (smooth_enable_) {
      smooth_elevation();
    } else {
      map_["elevation_smooth"] = map_["elevation"];
    }

    const rclcpp::Time stamp = have_pose ? rclcpp::Time(tf.header.stamp) : now();

    // Sprint 1 — custo: terreno + obstaculos -> cost = max(...).
    compute_terrain_cost();
    render_obstacles();  // obstaculos committed do SLAM (uniforme letal)
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
    publish_committed_cloud(stamp);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "elevation: %zu pontos/scan | tiles de terreno=%zu (persistentes) | "
      "keyframes=%zu | obstaculos comprometidos (SLAM)=%zu",
      last_ground_used_, terrain_tiles_.size(), keyframes_.size(), committed_.size());
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
    // Renderização REATIVA por-frame (world_model_trunks, cilindro castanho) REMOVIDA:
    // tremeluzia e desenhava TUDO como cilindro, incl. rochas (a rocha ficava com
    // cubo comprometido E cilindro reativo por cima). Os obstáculos reativos
    // continuam a alimentar o cost layer (funcional, não-viz). O INVENTÁRIO visual é
    // só o modelo COMPROMETIDO abaixo (consistente/gated; tronco=cilindro, rocha=cubo).
    // Modelo do mundo COMPROMETIDO (do /slam/tree_map): tronco=CYLINDER verde,
    // rocha/obstáculo=CUBE cinza. Estável (gate + histerese), segue o uid.
    for (const auto & kv : committed_) {
      const auto & c = kv.second;
      const bool trunk = (c.semantic_class == 3);
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = stamp;
      m.ns = trunk ? "committed_trunks" : "committed_obstacles";
      m.id = id++;
      m.type = trunk ? visualization_msgs::msg::Marker::CYLINDER
                     : visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = c.x;
      m.pose.position.y = c.y;
      m.pose.position.z = 0.75;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = std::max(0.05, 2.0 * c.radius);
      m.scale.z = 1.5;
      m.color.r = trunk ? 0.10f : 0.55f;
      m.color.g = trunk ? 0.70f : 0.55f;
      m.color.b = trunk ? 0.20f : 0.55f;
      m.color.a = 0.85f;
      arr.markers.push_back(m);
    }
    pub_world_model_->publish(arr);
  }

  std::string map_frame_, base_frame_, cost_layer_, ground_topic_, imu_topic_;
  bool use_imu_attitude_{true};
  double resolution_{}, publish_hz_{}, tf_timeout_sec_{}, cost_max_{};
  double max_insert_range_{}, meas_var_base_{}, meas_var_range_k_{};
  double outlier_gate_sigma_{}, var_min_{};
  bool smooth_enable_{true};
  bool defragment_buffer_{true};
  // Sprint 1
  bool enable_terrain_cost_{true}, enable_obstacle_cost_{true};
  double slope_cost_max_deg_{}, step_obstacle_m_{};
  double trunk_margin_m_{}, obstacle_inflation_m_{};
  // Parte 3 — modelo do mundo comprometido (do /slam/tree_map).
  std::string tree_map_topic_;
  int commit_min_obs_{8};
  double commit_max_dbh_var_{}, commit_max_pos_var_{}, commit_radius_ema_{};
  int committed_cloud_perim_pts_{12};
  std::string tree_map_frame_{"map"};

  bool have_imu_{false};
  double imu_roll_{0.0}, imu_pitch_{0.0};
  double robot_x_{0.0}, robot_y_{0.0};
  size_t last_ground_used_{0};
  std::map<std::uint64_t, Committed> committed_;
  // Tiles de terreno persistentes (grelha fixa; conteudo ancorado a keyframes).
  std::unique_ptr<forest_tree_slam::TileGrid> tile_grid_;
  std::map<forest_tree_slam::TileIndex, TerrainTile> terrain_tiles_;
  std::vector<Se2> keyframes_;  // poses otimizadas (indice = id da keyframe)

  grid_map::GridMap map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::TrackedTreeLandmarkArray>::SharedPtr sub_tree_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_keyframes_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_committed_;
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

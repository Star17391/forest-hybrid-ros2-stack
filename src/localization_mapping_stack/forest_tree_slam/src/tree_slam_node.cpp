// Nó ROS2 do Tree-SLAM florestal — liga tracker + backend GTSAM + relocalizer
// + gestor de modo. Ver docs/FOREST_TREE_SLAM_DESIGN.md (desenho completo) e
// docs/LAYER_CONTRACTS.md (contrato de tópicos/TF).
//
// Simplificação deliberada (âmbito de tese, ver design §11): observações de
// tronco feitas ENTRE keyframes são associadas à última keyframe usando a
// pose "morta-reconhecida" (dead-reckoned) por odom só para o cálculo de
// bearing/range — não criam keyframe própria. É a aproximação padrão de
// "low-rate keyframe, high-rate observation" em SLAM por landmarks.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "forest_hybrid_msgs/msg/hybrid_hop_status.hpp"
#include "forest_hybrid_msgs/msg/operation_mode.hpp"
#include "forest_hybrid_msgs/msg/semantic_class.hpp"
#include "forest_hybrid_msgs/msg/slam_status.hpp"
#include "forest_hybrid_msgs/msg/tracked_tree_landmark.hpp"
#include "forest_hybrid_msgs/msg/tracked_tree_landmark_array.hpp"
#include "forest_hybrid_msgs/msg/tree_landmark_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "forest_tree_slam/backend.hpp"
#include "forest_tree_slam/landmark_class.hpp"
#include "forest_tree_slam/mode_manager.hpp"
#include "forest_tree_slam/relocalizer.hpp"
#include "forest_tree_slam/se2_geometry.hpp"
#include "forest_tree_slam/tracker.hpp"

namespace forest_tree_slam
{
namespace
{
// wrap_angle / compose / between / transform_point / bearing_range_from /
// interpolate_pose vivem agora em se2_geometry.hpp (partilhados com os testes).

Pose2 pose_from_odom(const nav_msgs::msg::Odometry & msg)
{
  return Pose2{msg.pose.pose.position.x, msg.pose.pose.position.y,
    tf2::getYaw(msg.pose.pose.orientation)};
}

std::uint8_t semantic_class_from_committed(std::uint8_t committed_class)
{
  using forest_hybrid_msgs::msg::SemanticClass;
  switch (committed_class) {
    case kCommittedTrunk:
      return SemanticClass::CLASS_VEGETATION_RIGID;
    case kCommittedRock:
      return SemanticClass::CLASS_ROCK;
    case kCommittedObstacle:
      return SemanticClass::CLASS_OBSTACLE;
    default:
      return SemanticClass::CLASS_VOID;
  }
}

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

std::vector<std::vector<Eigen::Vector3d>> parse_tree_clusters_base_link(
  const sensor_msgs::msg::PointCloud2 & cloud, std::size_t n_trees)
{
  std::vector<std::vector<Eigen::Vector3d>> per_tree(n_trees);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_i(cloud, "intensity");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
    const auto idx = static_cast<std::size_t>(std::lround(*iter_i));
    if (idx >= n_trees) {
      continue;
    }
    per_tree[idx].emplace_back(static_cast<double>(*iter_x), static_cast<double>(*iter_y),
      static_cast<double>(*iter_z));
  }
  return per_tree;
}

std::vector<Eigen::Vector3d> transform_points_to_map(
  const std::vector<Eigen::Vector3d> & base_link_pts, const Pose2 & world_pose)
{
  std::vector<Eigen::Vector3d> out;
  out.reserve(base_link_pts.size());
  for (const auto & p : base_link_pts) {
    const Eigen::Vector2d w = transform_point(world_pose, p.x(), p.y());
    out.emplace_back(w.x(), w.y(), p.z());
  }
  return out;
}
}  // namespace

class TreeSlamNode : public rclcpp::Node
{
public:
  TreeSlamNode()
  : Node("tree_slam_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "marble_hd2/base_link");
    publish_hz_ = declare_parameter<double>("publish_hz", 10.0);
    gnss_good_variance_m2_ = declare_parameter<double>("gnss_good_variance_m2", 4.0);
    relocalization_max_scans_per_attempt_ =
      declare_parameter<int>("relocalization_max_scans_per_attempt", 10);
    diagnostics_ = declare_parameter<bool>("diagnostics", false);

    Eigen::Vector3d hop_sigma(
      declare_parameter<double>("aerial_hop_sigma_x", 3.0),
      declare_parameter<double>("aerial_hop_sigma_y", 3.0),
      declare_parameter<double>("aerial_hop_sigma_theta", 0.3));
    aerial_hop_sigma_ = hop_sigma;

    backend_ = std::make_unique<TreeSlamBackend>();

    TrackerParams tracker_params;
    tracker_params.promote_prob = declare_parameter<double>("tracker_promote_prob", 0.70);
    tracker_params.promote_margin = declare_parameter<double>("tracker_promote_margin", 0.20);
    tracker_params.promote_min_obs =
      static_cast<std::uint32_t>(declare_parameter<int>("tracker_promote_min_obs", 4));
    tracker_params.class_min_bearing_delta_rad =
      declare_parameter<double>("tracker_class_min_bearing_delta_rad", 0.15);
    tracker_params.class_correlated_obs_weight =
      declare_parameter<double>("tracker_class_correlated_obs_weight", 0.15);
    MultiviewDbhParams mv_params;
    mv_params.voxel_size_m = declare_parameter<double>("multiview_voxel_size_m", 0.02);
    mv_params.coverage_bins = declare_parameter<int>("multiview_coverage_bins", 36);
    mv_params.saturation_coverage = declare_parameter<double>("multiview_saturation_coverage",
        0.65);
    mv_params.min_inlier_frames =
      static_cast<std::uint32_t>(declare_parameter<int>("multiview_min_inlier_frames", 3));
    mv_params.saturation_max_diameter_var =
      declare_parameter<double>("multiview_saturation_max_diameter_var", 0.0025);
    mv_params.trim_tol_m = declare_parameter<double>("multiview_trim_tol_m", 0.05);
    tracker_params.multiview = mv_params;
    // Gate de ingestão multi-vista (A: resíduo de pose; B: qualidade por-frame).
    tracker_params.multiview_gate_max_center_residual_m =
      declare_parameter<double>("multiview_gate_max_center_residual_m", 0.15);
    tracker_params.multiview_gate_max_pos_var =
      declare_parameter<double>("multiview_gate_max_pos_var", 0.0025);
    tracker_params.multiview_gate_min_obs =
      static_cast<std::uint32_t>(declare_parameter<int>("multiview_gate_min_obs", 6));
    tracker_params.multiview_gate_max_diameter_stddev_m =
      declare_parameter<double>("multiview_gate_max_diameter_stddev_m", 0.20);
    tracker_params.multiview_gate_max_diameter_rel_dev =
      declare_parameter<double>("multiview_gate_max_diameter_rel_dev", 0.5);
    tracker_params.multiview_gate_point_consistency_tol_m =
      declare_parameter<double>("multiview_gate_point_consistency_tol_m", 0.06);
    tracker_params.multiview_gate_confident_var =
      declare_parameter<double>("multiview_gate_confident_var", 0.0025);
    tracker_ = std::make_unique<LandmarkTracker>(tracker_params);

    RelocalizerParams reloc_params;
    reloc_params.diameter_bin_max_m =
      declare_parameter<double>("relocalizer_diameter_bin_max_m", 1.5);
    relocalizer_ = std::make_unique<TreeLocRelocalizer>(reloc_params);
    mode_manager_ = std::make_unique<ModeManager>();

    rclcpp::QoS transient_local(1);
    transient_local.transient_local();

    sub_landmarks_ = create_subscription<forest_hybrid_msgs::msg::TreeLandmarkArray>(
      "/perception/lidar/tree_landmarks", 10,
      std::bind(&TreeSlamNode::on_landmarks, this, std::placeholders::_1));
    // tree_clusters é publicado pela perceção com SensorDataQoS (BEST_EFFORT). Um
    // subscriber RELIABLE (QoS=10) NÃO recebe de um publisher BEST_EFFORT — por isso
    // estas nuvens nunca chegavam e a ingestão multi-vista nunca acontecia. Usar QoS
    // compatível (sensor data) para receber de facto os inliers de tronco.
    sub_tree_clusters_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/perception/lidar/tree_clusters", rclcpp::SensorDataQoS(),
      std::bind(&TreeSlamNode::on_tree_clusters, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/state/odometry", 50, std::bind(&TreeSlamNode::on_odom, this, std::placeholders::_1));
    sub_mode_ = create_subscription<forest_hybrid_msgs::msg::OperationMode>(
      "/system/locomotion_mode", transient_local,
      std::bind(&TreeSlamNode::on_locomotion_mode, this, std::placeholders::_1));
    sub_hop_ = create_subscription<forest_hybrid_msgs::msg::HybridHopStatus>(
      "/forest_gen/hybrid/hop_status", 10,
      std::bind(&TreeSlamNode::on_hop_status, this, std::placeholders::_1));
    sub_ap_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ardupilot/local_position_odom", 10,
      std::bind(&TreeSlamNode::on_ap_odom, this, std::placeholders::_1));
    sub_gnss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/sensors/gnss/fix_adapted", 10,
      std::bind(&TreeSlamNode::on_gnss, this, std::placeholders::_1));

    pub_tree_map_ = create_publisher<forest_hybrid_msgs::msg::TrackedTreeLandmarkArray>(
      "/slam/tree_map", 10);
    // transient_local: map_odom_authority_node subscreve /slam/status com esta
    // durabilidade (sinal auxiliar de modo aéreo) — QoS tem de ser compatível.
    pub_status_ = create_publisher<forest_hybrid_msgs::msg::SlamStatus>("/slam/status",
        transient_local);
    pub_pose_graph_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/slam/pose_graph", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TreeSlamNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "tree_slam_node [build bearing-fix+odom-sync] pronto; diagnostics=%s. A aguardar 1.ª odometria.",
      diagnostics_ ? "ON" : "off");
  }

private:
  // --- Subscritores -------------------------------------------------------

  // Pose de odometria no instante `stamp_sec`, interpolada do buffer (Causa
  // nº2: a odom mais recente está adiantada face ao tempo de captura do scan).
  // Faz clamp aos extremos do buffer; se vazio, devolve a última pose conhecida.
  std::optional<Pose2> odom_at(double stamp_sec) const
  {
    if (odom_buffer_.empty()) {
      return last_odom_pose_;
    }
    // Mais antigo que tudo o que temos -> usa a amostra mais antiga.
    if (stamp_sec <= odom_buffer_.front().first) {
      return odom_buffer_.front().second;
    }
    // Mais recente que tudo (caso típico: scan chegou antes da odom seguinte)
    // -> usa a amostra mais recente sem extrapolar.
    if (stamp_sec >= odom_buffer_.back().first) {
      return odom_buffer_.back().second;
    }
    for (std::size_t i = 1; i < odom_buffer_.size(); ++i) {
      if (odom_buffer_[i].first >= stamp_sec) {
        return interpolate_pose(
          odom_buffer_[i - 1].second, odom_buffer_[i - 1].first,
          odom_buffer_[i].second, odom_buffer_[i].first, stamp_sec);
      }
    }
    return odom_buffer_.back().second;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_pose_ = pose_from_odom(*msg);
    const double odom_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9;
    odom_buffer_.emplace_back(odom_stamp, *last_odom_pose_);
    // Mantém ~2 s de histórico (chega para cobrir a latência da perceção).
    while (odom_buffer_.size() > 2 &&
      odom_stamp - odom_buffer_.front().first > odom_buffer_max_age_sec_)
    {
      odom_buffer_.pop_front();
    }
    if (!backend_->initialized()) {
      backend_->initialize(Pose2{0, 0, 0});
      // `initialize()` só enfileira o prior em new_factors_/new_values_; sem
      // este optimize() imediato, `current_estimate_` fica vazio e a 1.ª
      // chamada a `keyframe_pose(0)` (em on_landmarks/on_timer) lança
      // gtsam::ValuesKeyDoesNotExist.
      backend_->optimize();
      last_keyframe_odom_pose_ = *last_odom_pose_;
      RCLCPP_INFO(get_logger(), "Tree-SLAM inicializado (prior na origem).");
    }
  }

  void on_locomotion_mode(const forest_hybrid_msgs::msg::OperationMode::SharedPtr msg)
  {
    locomotion_aerial_ = (msg->mode == forest_hybrid_msgs::msg::OperationMode::MODE_AERIAL);
  }

  void on_hop_status(const forest_hybrid_msgs::msg::HybridHopStatus::SharedPtr msg)
  {
    hop_in_progress_ = (msg->state == forest_hybrid_msgs::msg::HybridHopStatus::STATE_IN_PROGRESS);
    hop_done_ = (msg->state == forest_hybrid_msgs::msg::HybridHopStatus::STATE_DONE);
    hop_failed_ = (msg->state == forest_hybrid_msgs::msg::HybridHopStatus::STATE_FAILED);
  }

  void on_ap_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_ap_pose_ = pose_from_odom(*msg);
  }

  void on_gnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    const double horiz_var = std::max(msg->position_covariance[0], msg->position_covariance[4]);
    gnss_good_ = horiz_var <= gnss_good_variance_m2_;
  }

  void on_tree_clusters(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double stamp_sec = stamp_to_sec(msg->header.stamp);
    clusters_by_stamp_[stamp_sec] = msg;
    while (clusters_by_stamp_.size() > 32) {
      clusters_by_stamp_.erase(clusters_by_stamp_.begin());
    }
    // O cluster deste stamp acabou de chegar (tipicamente DEPOIS dos landmarks):
    // tenta fechar o par e ingerir os pontos de tronco na referência multi-vista.
    try_flush_multiview(stamp_sec);
  }

  sensor_msgs::msg::PointCloud2::ConstSharedPtr clusters_for_stamp(double stamp_sec) const
  {
    constexpr double kTol = 5.0e-4;
    for (auto it = clusters_by_stamp_.rbegin(); it != clusters_by_stamp_.rend(); ++it) {
      if (std::abs(it->first - stamp_sec) <= kTol) {
        return it->second;
      }
    }
    return nullptr;
  }

  // Guarda o frame de landmarks (pose + uid associado por deteção) para ingestão
  // multi-vista diferida, à espera que o cluster do mesmo stamp chegue.
  void store_pending_multiview(
    double stamp_sec, const Pose2 & pose,
    const std::vector<TreeDetection> & dets, const TrackerUpdateReport & report)
  {
    PendingMultiview pm;
    pm.pose = pose;
    pm.robot_xy = Eigen::Vector2d(pose.x, pose.y);
    pm.dets = dets;
    pm.uids = report.detection_to_uid;
    pending_mv_[stamp_sec] = std::move(pm);
    while (pending_mv_.size() > 32) {
      pending_mv_.erase(pending_mv_.begin());
    }
  }

  // Fecha o par (landmarks+clusters) do mesmo stamp e ingere os pontos de tronco
  // na referência multi-vista de cada landmark. Idempotente: apaga o pending no fim.
  void try_flush_multiview(double stamp_sec)
  {
    constexpr double kTol = 5.0e-4;
    auto pit = pending_mv_.end();
    for (auto it = pending_mv_.begin(); it != pending_mv_.end(); ++it) {
      if (std::abs(it->first - stamp_sec) <= kTol) { pit = it; break; }
    }
    if (pit == pending_mv_.end()) { return; }
    const auto clusters = clusters_for_stamp(pit->first);
    if (!clusters) { return; }  // cluster ainda não chegou; tenta-se de novo quando chegar
    PendingMultiview & pm = pit->second;
    const auto per_tree_bl = parse_tree_clusters_base_link(*clusters, pm.uids.size());
    for (std::size_t i = 0; i < pm.uids.size(); ++i) {
      const LandmarkUid uid = pm.uids[i];
      if (uid == 0 || i >= per_tree_bl.size() || per_tree_bl[i].empty()) { continue; }
      TreeDetection det = pm.dets[i];
      det.has_stem_inliers = true;  // afirmado: temos os inliers de tronco deste stamp
      const auto pts = transform_points_to_map(per_tree_bl[i], pm.pose);
      tracker_->ingest_multiview_inliers(uid, pts, pm.robot_xy, det);
    }
    pending_mv_.erase(pit);
  }

  const LandmarkTrack * find_track(LandmarkUid uid) const
  {
    for (const auto & t : tracker_->tracks()) {
      if (t.uid == uid) {
        return &t;
      }
    }
    return nullptr;
  }

  bool feeds_pose_graph(LandmarkUid uid) const
  {
    const auto * track = find_track(uid);
    return track != nullptr && LandmarkTracker::is_promoted(*track) &&
           is_slam_graph_class(track->committed_class);
  }

  void on_landmarks(const forest_hybrid_msgs::msg::TreeLandmarkArray::SharedPtr msg)
  {
    if (!backend_->initialized() || !last_odom_pose_) {
      return;  // ainda sem odometria/prior
    }

    const double stamp_sec =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1.0e-9;

    if (mode_manager_->mode() == SlamMode::RELOCALIZING) {
      try_relocalize(*msg);
      return;  // não alimenta o tracker/grafo enquanto não voltar a GROUND
    }
    if (mode_manager_->mode() != SlamMode::GROUND) {
      return;  // AERIAL: sem troncos visíveis por definição (design §5.4)
    }

    // Odometria NO INSTANTE DE CAPTURA do scan (não a mais recente recebida) —
    // a perceção leva tempo a processar a nuvem, por isso `last_odom_pose_` está
    // adiantada face ao tempo do scan. Interpola do buffer (Causa nº2).
    const Pose2 scan_odom_pose = odom_at(stamp_sec).value_or(*last_odom_pose_);

    // delta desde a última keyframe, em SE(2), via odometria (frame `odom`).
    const Pose2 delta_since_keyframe = between(last_keyframe_odom_pose_, scan_odom_pose);
    const Pose2 last_kf_pose = backend_->keyframe_pose(backend_->n_keyframes() - 1);
    const Pose2 predicted_world_pose = compose(last_kf_pose, delta_since_keyframe);

    // Transforma deteções base_link -> frame de trabalho do tracker (mundo).
    std::vector<TreeDetection> detections_world;
    detections_world.reserve(msg->trees.size());
    const double c = std::cos(predicted_world_pose.theta), s = std::sin(predicted_world_pose.theta);
    for (const auto & tree : msg->trees) {
      TreeDetection d;
      const Eigen::Vector2d w = transform_point(predicted_world_pose, tree.base.x, tree.base.y);
      d.x = w.x();
      d.y = w.y();
      d.diameter = tree.diameter;
      d.confidence = tree.confidence;
      d.diameter_stddev = tree.diameter_stddev;
      Eigen::Matrix3d cov_in;
      for (int i = 0; i < 9; ++i) {
        cov_in(i / 3, i % 3) = tree.base_covariance[static_cast<std::size_t>(i)];
      }
      Eigen::Matrix2d r;
      r << c, -s, s, c;
      d.base_covariance.setZero();
      d.base_covariance.topLeftCorner<2, 2>() = r * cov_in.topLeftCorner<2, 2>() * r.transpose();
      for (std::size_t k = 0; k < kNumClassScores; ++k) {
        d.class_scores[k] = tree.class_scores[k];
      }
      detections_world.push_back(d);
    }

    // A ingestão multi-vista (pontos de tronco -> multiview_buffer) é DIFERIDA: os
    // tree_clusters do mesmo stamp chegam DEPOIS dos tree_landmarks (a perceção
    // publica landmarks antes dos clusters), por isso clusters_for_stamp() falharia
    // aqui e has_stem_inliers ficaria sempre falso. Guardamos o frame em pending_mv_
    // e fazemos a ingestão quando o par (landmarks+clusters) estiver completo —
    // ver store_pending_multiview()/try_flush_multiview(), chamado abaixo e em
    // on_tree_clusters(). has_stem_inliers é afirmado no momento da ingestão.

    // Δheading desde o scan anterior (não desde a última keyframe) — só para
    // a predição do tracker, ver process_noise_xy_var em tracker.hpp.
    const double angular_delta_since_last_scan = last_landmarks_odom_pose_ ?
      std::abs(between(*last_landmarks_odom_pose_, scan_odom_pose).theta) :
      0.0;
    last_landmarks_odom_pose_ = scan_odom_pose;

    // Frame consistente (Passo 1): antes de associar, ancora os landmarks
    // promovidos às posições OTIMIZADAS do backend. Sem isto o tracker associa
    // contra a sua cópia desatualizada (congelada ao adormecer) e os adormecidos
    // não reativam no regresso após uma correção do backend.
    {
      std::unordered_map<LandmarkUid, Eigen::Vector2d> backend_positions;
      for (const auto uid : backend_->all_landmark_uids()) {
        backend_positions.emplace(uid, backend_->landmark_position(uid));
      }
      tracker_->sync_landmark_anchors(backend_positions);
    }

    const auto report = tracker_->update(
      detections_world, stamp_sec,
      Eigen::Vector2d(predicted_world_pose.x, predicted_world_pose.y),
      angular_delta_since_last_scan);

    // Guarda este frame (pose + uid por deteção) para ingestão multi-vista diferida,
    // e tenta logo fechar o par caso o cluster deste stamp já tenha chegado.
    store_pending_multiview(stamp_sec, predicted_world_pose, detections_world, report);
    try_flush_multiview(stamp_sec);

    // Resumo de associação (sempre ligado, throttled): distingue associações
    // por POSIÇÃO (stage-A) das re-associações por GEOMETRIA (loop closure no
    // solo) e dos births; conta adormecidos vs ativos. Diagnostica diretamente
    // a queixa "os azuis não reativam": se geo_reawaken≈0 com adormecidos a
    // crescer, a re-associação por geometria não está a disparar.
    {
      std::size_t n_assoc_total = 0;
      for (auto u : report.detection_to_uid) {
        if (u != 0) {++n_assoc_total;}
      }
      std::size_t n_dormant = 0, n_active = 0;
      for (const auto & t : tracker_->tracks()) {
        if (LandmarkTracker::is_promoted(t) && is_slam_graph_class(t.committed_class)) {
          (t.dormant ? n_dormant : n_active)++;
        }
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[assoc] dets=%zu assoc=%zu (geo_reawaken=%zu) birth=%zu | mapa: ativos=%zu adormecidos=%zu",
        detections_world.size(), n_assoc_total, report.reawakened.size(), report.births.size(),
        n_active, n_dormant);
    }

    // GROUND: decide se abre keyframe nova (acumula odom) e atribui o índice
    // de keyframe ao qual as observações deste scan vão ser ligadas.
    std::size_t obs_keyframe = backend_->n_keyframes() - 1;
    bool opened_keyframe = false;
    if (backend_->should_open_keyframe(delta_since_keyframe)) {
      obs_keyframe = backend_->add_odom_keyframe(delta_since_keyframe);
      last_keyframe_odom_pose_ = scan_odom_pose;
      opened_keyframe = true;
    }

    // Pose da KEYFRAME de observação — é o frame no qual o BearingRangeFactor
    // tem de ser expresso (Causa nº1). Se a keyframe foi aberta agora, ainda
    // não existe em current_estimate_ (gtsam::ValuesKeyDoesNotExist); mas o seu
    // estimate inicial é exatamente `predicted_world_pose` (== prev_pose.compose
    // (delta), backend.cpp:81). Se NÃO foi aberta, a keyframe anterior já está
    // otimizada e a sua pose é `last_kf_pose` — e é relativamente a ESSA pose
    // (não à pose dead-reckoned predicted) que a medição tem de ser calculada,
    // senão o fator descarta o Δodom keyframe->scan (bug corrigido, ver
    // test_node_glue.cpp).
    const Pose2 obs_pose = opened_keyframe ? predicted_world_pose : last_kf_pose;

    // Fatores bearing-range: só landmarks promovidos tronco/rocha (obstáculo fora do grafo).
    for (std::size_t i = 0; i < detections_world.size(); ++i) {
      const LandmarkUid uid = report.detection_to_uid[i];
      if (uid == 0 || !feeds_pose_graph(uid)) {
        continue;
      }
      const BearingRange br =
        bearing_range_from(obs_pose, detections_world[i].x, detections_world[i].y);
      backend_->add_tree_observation(
        uid, obs_keyframe, br.bearing, br.range,
        Eigen::Vector2d(detections_world[i].x, detections_world[i].y));
    }
    // Rigidez de constelação entre troncos vistos no MESMO scan (geometria
    // local crua em base_link, mais fiável que a estimativa em mundo).
    for (std::size_t a = 0; a < msg->trees.size(); ++a) {
      for (std::size_t b = a + 1; b < msg->trees.size(); ++b) {
        const LandmarkUid ua = report.detection_to_uid[a], ub = report.detection_to_uid[b];
        if (ua == 0 || ub == 0 || !feeds_pose_graph(ua) || !feeds_pose_graph(ub)) {
          continue;
        }
        const double d = std::hypot(
          msg->trees[a].base.x - msg->trees[b].base.x, msg->trees[a].base.y - msg->trees[b].base.y);
        backend_->add_constellation_distance(ua, ub, d);
      }
    }

    if (gnss_good_) {
      // Fator GPS fraco — só quando a cov reportada é boa (acima do dossel).
      // Sem fonte de posição GNSS absoluta ligada diretamente a este nó por
      // agora; a ancoragem fica a cargo do EKF global enquanto não houver um
      // `/sensors/gnss/fix_local` em frame `map`. TODO: ligar quando existir.
    }

    backend_->optimize();
    scans_since_any_association_ = report.detection_to_uid.empty() ||
      std::all_of(report.detection_to_uid.begin(), report.detection_to_uid.end(), [](auto u) {
          return u == 0;
                                                                                                            })
      ?
      scans_since_any_association_ + 1 :
      0;

    if (diagnostics_) {
      log_diagnostics(stamp_sec, scan_odom_pose, opened_keyframe, report);
    }
  }

  // Diagnóstico de atribuição do "solavanco" (qual camada). Imprime por ciclo:
  //  - latency: now() - stamp do scan -> magnitude da dessincronização odom/scan.
  //  - kf_wobble: quanto a MESMA keyframe se mexeu entre dois optimize() sem
  //    keyframe nova nem loop closure -> instabilidade INTERNA do backend (esta
  //    camada). Suave/pequeno = backend estável; grande/errático = backend a
  //    saltar (entrada inconsistente da perceção ou grafo mal condicionado).
  //  - odom_step: passo da odom de entrada entre scans -> se der saltos grandes,
  //    o problema é a MONTANTE (EKF), não aqui.
  //  - births/deaths/assoc: churn de associação -> piscar da perceção.
  void log_diagnostics(
    double stamp_sec, const Pose2 & scan_odom_pose, bool opened_keyframe,
    const TrackerUpdateReport & report)
  {
    const double latency = now().seconds() - stamp_sec;

    std::size_t n_assoc = 0;
    for (auto u : report.detection_to_uid) {
      if (u != 0) {++n_assoc;}
    }

    const std::size_t kf_idx = backend_->n_keyframes() - 1;
    const Pose2 kf_now = backend_->keyframe_pose(kf_idx);
    double kf_wobble_xy = 0.0, kf_wobble_th = 0.0;
    if (!opened_keyframe && diag_prev_kf_valid_ && diag_prev_kf_idx_ == kf_idx) {
      kf_wobble_xy = std::hypot(kf_now.x - diag_prev_kf_.x, kf_now.y - diag_prev_kf_.y);
      kf_wobble_th = std::abs(wrap_angle(kf_now.theta - diag_prev_kf_.theta));
    }
    diag_prev_kf_ = kf_now;
    diag_prev_kf_idx_ = kf_idx;
    diag_prev_kf_valid_ = true;

    double odom_step = 0.0;
    if (diag_prev_odom_) {
      odom_step = std::hypot(
        scan_odom_pose.x - diag_prev_odom_->x, scan_odom_pose.y - diag_prev_odom_->y);
    }
    diag_prev_odom_ = scan_odom_pose;

    RCLCPP_INFO(
      get_logger(),
      "[diag] lat=%.0fms odom_step=%.3fm kf_wobble=%.3fm/%.1fdeg%s assoc=%zu birth=%zu death=%zu tracks=%zu",
      latency * 1e3, odom_step, kf_wobble_xy, kf_wobble_th * 180.0 / M_PI,
      opened_keyframe ? " [KF]" : "", n_assoc, report.births.size(), report.deaths.size(),
      tracker_->tracks().size());
  }

  void try_relocalize(const forest_hybrid_msgs::msg::TreeLandmarkArray & msg)
  {
    ++relocalization_scans_this_attempt_;
    if (msg.trees.size() < 3 &&
      relocalization_scans_this_attempt_ < relocalization_max_scans_per_attempt_)
    {
      return;  // espera por mais troncos visíveis antes de tentar
    }

    std::vector<LandmarkPoint> query;
    query.reserve(msg.trees.size());
    for (const auto & tree : msg.trees) {
      LandmarkPoint p;
      p.x = tree.base.x;
      p.y = tree.base.y;
      p.diameter = tree.diameter;
      query.push_back(p);
    }
    std::vector<LandmarkPoint> map_points;
    for (const auto uid : backend_->all_landmark_uids()) {
      const auto xy = backend_->landmark_position(uid);
      LandmarkPoint p;
      p.uid = uid;
      p.x = xy.x();
      p.y = xy.y();
      map_points.push_back(p);
    }

    const auto result = relocalizer_->relocalize(query, map_points);
    if (result.accepted) {
      const std::size_t landing_kf = backend_->n_keyframes() - 1;
      // `map_to_query_transform` transforma pontos do frame query (base_link
      // na aterragem) para `map` — é exatamente a pose da keyframe de aterragem.
      const Pose2 landing_pose = result.map_to_query_transform;
      for (const auto & c : result.correspondences) {
        const auto & q = query[c.query_index];
        const double dx = q.x, dy = q.y;  // já em base_link == frame da keyframe
        const double range = std::hypot(dx, dy);
        const double bearing = std::atan2(dy, dx);
        backend_->add_relocalization_factor(c.map_uid, landing_kf, bearing, range);
      }
      // Ancora a pose da keyframe à estimativa do TreeLoc (alta confiança).
      backend_->add_weak_gps_prior(
        landing_kf, Eigen::Vector2d(landing_pose.x, landing_pose.y), Eigen::Vector2d(0.05, 0.05));
      backend_->optimize();
      relocalization_scans_this_attempt_ = 0;
      mode_manager_->notify_relocalization_result(true);
      RCLCPP_INFO(
        get_logger(), "Relocalização aceite: overlap=%.2f, residual=%.3fm, %zu correspondências",
        result.overlap_ratio, result.mean_residual_m, result.correspondences.size());
      return;
    }

    if (relocalization_scans_this_attempt_ >= relocalization_max_scans_per_attempt_) {
      relocalization_scans_this_attempt_ = 0;
      mode_manager_->notify_relocalization_result(false);
      RCLCPP_WARN(get_logger(), "Relocalização rejeitada após esgotar tentativa.");
    }
  }

  // --- Timer: gestor de modo, TF, publicações ------------------------------

  void on_timer()
  {
    if (!backend_->initialized()) {
      return;
    }

    ModeManagerInputs in;
    in.locomotion_aerial = locomotion_aerial_;
    in.hop_in_progress = hop_in_progress_;
    in.hop_done = hop_done_;
    in.hop_failed = hop_failed_;
    in.gnss_good = gnss_good_;
    in.scans_since_any_association = scans_since_any_association_;
    const auto events = mode_manager_->update(in);

    if (events.request_takeoff_snapshot) {
      RCLCPP_INFO(get_logger(), "AERIAL: map->odom congela (autoridade passa ao EKF global).");
    }
    if (events.request_relocalization) {
      const std::size_t landing_kf = backend_->add_aerial_hop_edge(
        last_ap_pose_.value_or(Pose2{0, 0, 0}), aerial_hop_sigma_);
      last_keyframe_odom_pose_ = last_odom_pose_.value_or(last_keyframe_odom_pose_);
      backend_->optimize();
      relocalization_scans_this_attempt_ = 0;
      RCLCPP_INFO(
        get_logger(), "Aterragem: keyframe %zu criada via aresta SE2 do salto; relocalização %s.",
        landing_kf, events.relocalization_mandatory ? "OBRIGATÓRIA" : "opcional");
    }

    publish_status();
    publish_tree_map();
    publish_pose_graph();
    publish_tf();
  }

  void publish_status()
  {
    forest_hybrid_msgs::msg::SlamStatus status;
    status.header.stamp = now();
    status.mode = static_cast<std::uint8_t>(mode_manager_->mode());
    status.n_landmarks_tracked = static_cast<std::uint32_t>(std::count_if(
      tracker_->tracks().begin(), tracker_->tracks().end(),
        [](const LandmarkTrack & t) {
          return LandmarkTracker::is_promoted(t) && is_map_output_class(t.committed_class);
      }));
    status.owns_map_to_odom = mode_manager_->owns_map_to_odom();
    pub_status_->publish(status);
  }

  void publish_tree_map()
  {
    forest_hybrid_msgs::msg::TrackedTreeLandmarkArray out;
    out.header.stamp = now();
    out.header.frame_id = map_frame_;
    for (const auto & t : tracker_->tracks()) {
      if (!LandmarkTracker::is_promoted(t) || !is_map_output_class(t.committed_class)) {
        continue;
      }
      forest_hybrid_msgs::msg::TrackedTreeLandmark m;
      m.uid = t.uid;
      m.semantic_class = semantic_class_from_committed(t.committed_class);
      if (backend_->has_landmark(t.uid)) {
        const auto xy = backend_->landmark_position(t.uid);
        m.position.x = xy.x();
        m.position.y = xy.y();
      } else {
        m.position.x = t.xy.x();
        m.position.y = t.xy.y();
      }
      m.diameter = static_cast<float>(t.diameter);
      // Confiança = posterior de CLASSIFICAÇÃO da classe comprometida (acumula
      // com vistas diversas e PERSISTE), não o contador de recência t.confidence
      // (que decaía a 0 fora de vista). O volume de evidência vai em n_observations.
      const Eigen::Vector3d post = LandmarkTracker::class_posterior(t);
      const int ci = score_index_from_committed(t.committed_class);
      m.confidence = (ci >= 0 && ci < 3) ? static_cast<float>(post[ci]) : 0.0F;
      m.n_observations = t.n_observations;
      // Covariância (x,y,dbh) para diagnóstico downstream.
      Eigen::Matrix3d cov3 = Eigen::Matrix3d::Zero();
      cov3.topLeftCorner<2, 2>() = t.cov;
      cov3(2, 2) = t.diameter_var;
      for (int i = 0; i < 9; ++i) {
        m.covariance[static_cast<std::size_t>(i)] = cov3(i / 3, i % 3);
      }
      out.trees.push_back(m);
    }
    pub_tree_map_->publish(out);
  }

  // Visualização de diagnóstico do pose-graph: uma esfera por landmark
  // (tamanho ~ DBH, cor por estado) com etiqueta de texto (uid + n_obs), uma
  // linha ligando as keyframes otimizadas, e um contador `next_uid` (total de
  // tracks alguma vez criados — Passo 0 do plano de robustez de associação:
  // se este número não pára de crescer numa 2.ª volta, NÃO há associação/loop
  // closure; os landmarks adormecidos aparecem a AZUL e devem voltar à cor
  // normal com o MESMO número ao serem re-detetados).
  void publish_pose_graph()
  {
    visualization_msgs::msg::MarkerArray markers;

    // DELETEALL no início: limpa marcadores antigos (uids que morreram) para
    // que cada publicação reflita exatamente o estado atual do mapa.
    {
      visualization_msgs::msg::Marker clear;
      clear.header.stamp = now();
      clear.header.frame_id = map_frame_;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
    }

    // Cores por estado (a do solavanco/azul = adormecido).
    constexpr float kTrunk[3] = {0.55F, 0.27F, 0.07F};   // castanho
    constexpr float kRock[3] = {0.5F, 0.5F, 0.5F};        // cinza
    constexpr float kDormant[3] = {0.1F, 0.35F, 0.95F};   // azul = fora de vista

    // Cor distinta por uid (hue por razão áurea) — para colorir os pontos
    // ACUMULADOS de cada landmark e ver, no RViz, que pontos alimentam cada
    // cilindro de DBH (revela contaminação por copa: pontos altos/largos).
    auto uid_color = [](std::uint64_t uid, float & r, float & g, float & b) {
        const double hh = std::fmod(static_cast<double>(uid) * 0.61803398875, 1.0) * 6.0;
        const int i = static_cast<int>(hh);
        const auto f = static_cast<float>(hh - i);
        const float q = 1.0F - f;
        switch (i % 6) {
          case 0: r = 1; g = f; b = 0; break;
          case 1: r = q; g = 1; b = 0; break;
          case 2: r = 0; g = 1; b = f; break;
          case 3: r = 0; g = q; b = 1; break;
          case 4: r = f; g = 0; b = 1; break;
          default: r = 1; g = 0; b = q; break;
        }
      };

    for (const auto & t : tracker_->tracks()) {
      if (!LandmarkTracker::is_promoted(t) || !is_map_output_class(t.committed_class)) {
        continue;
      }
      if (is_slam_graph_class(t.committed_class) && !backend_->has_landmark(t.uid)) {
        continue;
      }
      Eigen::Vector2d xy = t.xy;
      if (backend_->has_landmark(t.uid)) {
        xy = backend_->landmark_position(t.uid);
      }

      // Adormecido = não associado neste scan (scans_since_seen > 0). Volta à
      // cor da classe assim que for re-detetado (scans_since_seen == 0).
      const bool dormant = t.scans_since_seen > 0;
      const float * c = dormant ? kDormant :
        (t.committed_class == kCommittedRock ? kRock : kTrunk);

      visualization_msgs::msg::Marker sphere;
      sphere.header.stamp = now();
      sphere.header.frame_id = map_frame_;
      sphere.ns = "tree_slam_landmarks";
      sphere.id = static_cast<int>(t.uid);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = xy.x();
      sphere.pose.position.y = xy.y();
      sphere.pose.position.z = 0.0;
      sphere.pose.orientation.w = 1.0;
      const double d = std::max(0.15, t.diameter);
      sphere.scale.x = d;
      sphere.scale.y = d;
      sphere.scale.z = d;
      sphere.color.r = c[0];
      sphere.color.g = c[1];
      sphere.color.b = c[2];
      sphere.color.a = dormant ? 0.55F : 0.95F;
      markers.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header.stamp = now();
      label.header.frame_id = map_frame_;
      label.ns = "tree_slam_uid";
      label.id = static_cast<int>(t.uid);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = xy.x();
      label.pose.position.y = xy.y();
      label.pose.position.z = 1.2;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.45;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      // Label: uid, nº de observações e posterior de classificação (confiança
      // real, 0..1) — para se VER a confiança a acumular/persistir no RViz.
      const Eigen::Vector3d post = LandmarkTracker::class_posterior(t);
      const int ci = score_index_from_committed(t.committed_class);
      char pbuf[8];
      std::snprintf(pbuf, sizeof(pbuf), "%.2f", (ci >= 0 && ci < 3) ? post[ci] : 0.0);
      label.text =
        "#" + std::to_string(t.uid) + " n" + std::to_string(t.n_observations) + " p" + pbuf;
      markers.markers.push_back(label);

      // Pontos ACUMULADOS deste landmark (multi-view, frame map, z REAL) — é o
      // conjunto a que o cilindro de DBH é ajustado. Cor única por uid. Permite
      // ver no RViz a junção dos pontos por landmark e a contaminação por copa
      // (pontos muito acima da base ou muito espalhados => DBH inflado).
      const auto lm_points = t.multiview_buffer.points();
      if (!lm_points.empty()) {
        visualization_msgs::msg::Marker pts;
        pts.header.stamp = now();
        pts.header.frame_id = map_frame_;
        pts.ns = "tree_slam_landmark_points";
        pts.id = static_cast<int>(t.uid);
        pts.type = visualization_msgs::msg::Marker::POINTS;
        pts.action = visualization_msgs::msg::Marker::ADD;
        pts.scale.x = 0.03;
        pts.scale.y = 0.03;
        pts.pose.orientation.w = 1.0;
        float r, g, b;
        uid_color(t.uid, r, g, b);
        pts.color.r = r;
        pts.color.g = g;
        pts.color.b = b;
        pts.color.a = 1.0F;
        for (const auto & p : lm_points) {
          geometry_msgs::msg::Point gp;
          gp.x = p.x();
          gp.y = p.y();
          gp.z = p.z();
          pts.points.push_back(gp);
        }
        markers.markers.push_back(pts);
      }
    }

    // Contador global: total de uids alguma vez criados (baseline do churn).
    {
      visualization_msgs::msg::Marker counter;
      counter.header.stamp = now();
      counter.header.frame_id = map_frame_;
      counter.ns = "tree_slam_next_uid";
      counter.id = 0;
      counter.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      counter.action = visualization_msgs::msg::Marker::ADD;
      counter.pose.position.z = 3.0;
      counter.pose.orientation.w = 1.0;
      counter.scale.z = 0.7;
      counter.color.r = 1.0F;
      counter.color.g = 0.9F;
      counter.color.b = 0.2F;
      counter.color.a = 1.0F;
      counter.text = "uids criados: " + std::to_string(tracker_->next_uid() - 1);
      markers.markers.push_back(counter);
    }

    visualization_msgs::msg::Marker trajectory;
    trajectory.header.stamp = now();
    trajectory.header.frame_id = map_frame_;
    trajectory.ns = "tree_slam_trajectory";
    trajectory.id = 3;
    trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::msg::Marker::ADD;
    trajectory.scale.x = 0.05;
    trajectory.color.r = 0.1F;
    trajectory.color.g = 0.6F;
    trajectory.color.b = 1.0F;
    trajectory.color.a = 1.0F;
    trajectory.pose.orientation.w = 1.0;
    for (std::size_t i = 0; i < backend_->n_keyframes(); ++i) {
      const Pose2 p = backend_->keyframe_pose(i);
      geometry_msgs::msg::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.0;
      trajectory.points.push_back(pt);
    }
    markers.markers.push_back(trajectory);

    pub_pose_graph_->publish(markers);
  }

  void publish_tf()
  {
    if (!mode_manager_->owns_map_to_odom()) {
      return;  // autoridade é do EKF global (AERIAL) — regra de ouro, §LAYER_CONTRACTS
    }
    if (!mode_manager_->pose_frozen()) {
      const Pose2 last_kf = backend_->keyframe_pose(backend_->n_keyframes() - 1);
      const Pose2 delta = last_odom_pose_ ? between(last_keyframe_odom_pose_,
          *last_odom_pose_) : Pose2{};
      const Pose2 map_base = compose(last_kf, delta);

      tf2::Transform t_map_base;
      t_map_base.setOrigin(tf2::Vector3(map_base.x, map_base.y, 0.0));
      tf2::Quaternion q;
      q.setRPY(0, 0, map_base.theta);
      t_map_base.setRotation(q);

      geometry_msgs::msg::TransformStamped t_odom_base;
      try {
        t_odom_base = tf_buffer_.lookupTransform(
          odom_frame_, base_link_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "TF %s: %s", odom_frame_.c_str(),
            ex.what());
        return;
      }
      tf2::Transform t_odom_base_tf;
      tf2::fromMsg(t_odom_base.transform, t_odom_base_tf);
      last_map_odom_ = t_map_base * t_odom_base_tf.inverse();
    }

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = now();
    out.header.frame_id = map_frame_;
    out.child_frame_id = odom_frame_;
    tf2::toMsg(last_map_odom_, out.transform);
    tf_broadcaster_.sendTransform(out);
  }

  // --- Parâmetros -----------------------------------------------------
  std::string map_frame_, odom_frame_, base_link_frame_;
  double publish_hz_{10.0};
  double gnss_good_variance_m2_{4.0};
  int relocalization_max_scans_per_attempt_{10};
  Eigen::Vector3d aerial_hop_sigma_{3.0, 3.0, 0.3};

  // --- Estado -----------------------------------------------------------
  std::unique_ptr<TreeSlamBackend> backend_;
  std::unique_ptr<LandmarkTracker> tracker_;
  std::unique_ptr<TreeLocRelocalizer> relocalizer_;
  std::unique_ptr<ModeManager> mode_manager_;

  std::optional<Pose2> last_odom_pose_;
  // Histórico de odometria (stamp_sec, pose) para sincronizar com o tempo de
  // captura de cada scan (Causa nº2). Mantém ~odom_buffer_max_age_sec_ de dados.
  std::deque<std::pair<double, Pose2>> odom_buffer_;
  double odom_buffer_max_age_sec_{2.0};
  std::optional<Pose2> last_ap_pose_;
  Pose2 last_keyframe_odom_pose_{};
  // Pose de odometria no scan ANTERIOR (distinto de last_keyframe_odom_pose_,
  // que só muda quando se abre keyframe nova) — usada só para o passo de
  // predição do tracker (Δheading entre scans consecutivos).
  std::optional<Pose2> last_landmarks_odom_pose_;
  tf2::Transform last_map_odom_{tf2::Transform::getIdentity()};

  bool locomotion_aerial_{false};
  bool hop_in_progress_{false};
  bool hop_done_{false};
  bool hop_failed_{false};
  bool gnss_good_{false};
  int scans_since_any_association_{0};
  int relocalization_scans_this_attempt_{0};

  std::map<double, sensor_msgs::msg::PointCloud2::SharedPtr> clusters_by_stamp_;

  // Frame de landmarks à espera do cluster do mesmo stamp, para ingestão multi-vista
  // diferida (os tree_clusters chegam depois dos tree_landmarks). Ver try_flush_multiview.
  struct PendingMultiview
  {
    Pose2 pose{};
    Eigen::Vector2d robot_xy{Eigen::Vector2d::Zero()};
    std::vector<TreeDetection> dets;
    std::vector<LandmarkUid> uids;
  };
  std::map<double, PendingMultiview> pending_mv_;

  // Diagnóstico (gated por `diagnostics_`): estado anterior para medir deltas.
  bool diagnostics_{false};
  Pose2 diag_prev_kf_{};
  std::size_t diag_prev_kf_idx_{0};
  bool diag_prev_kf_valid_{false};
  std::optional<Pose2> diag_prev_odom_;

  // --- ROS --------------------------------------------------------------
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Subscription<forest_hybrid_msgs::msg::TreeLandmarkArray>::SharedPtr sub_landmarks_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_tree_clusters_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::OperationMode>::SharedPtr sub_mode_;
  rclcpp::Subscription<forest_hybrid_msgs::msg::HybridHopStatus>::SharedPtr sub_hop_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ap_odom_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_;

  rclcpp::Publisher<forest_hybrid_msgs::msg::TrackedTreeLandmarkArray>::SharedPtr pub_tree_map_;
  rclcpp::Publisher<forest_hybrid_msgs::msg::SlamStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_graph_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace forest_tree_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_tree_slam::TreeSlamNode>());
  rclcpp::shutdown();
  return 0;
}

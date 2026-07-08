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
#include <set>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "forest_tree_slam/backend.hpp"
#include "forest_tree_slam/camera_projection.hpp"
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
    // Pós-data do map->odom (como o AMCL): carimbar para o FUTURO por esta margem,
    // para o TF ser válido até à próxima publicação. Sem isto, consumidores (Nav2/RPP)
    // que pedem a transformação "agora" caem à frente do último carimbo →
    // "extrapolation into the future" → o controlador não transforma a pose e ABORTA.
    tf_transform_tolerance_ = declare_parameter<double>("tf_transform_tolerance", 0.2);
    gnss_good_variance_m2_ = declare_parameter<double>("gnss_good_variance_m2", 4.0);
    relocalization_max_scans_per_attempt_ =
      declare_parameter<int>("relocalization_max_scans_per_attempt", 10);
    diagnostics_ = declare_parameter<bool>("diagnostics", false);

    Eigen::Vector3d hop_sigma(
      declare_parameter<double>("aerial_hop_sigma_x", 3.0),
      declare_parameter<double>("aerial_hop_sigma_y", 3.0),
      declare_parameter<double>("aerial_hop_sigma_theta", 0.3));
    aerial_hop_sigma_ = hop_sigma;

    // --- BackendParams afináveis por YAML (antes eram defaults hardcoded e o
    //     backend era construído sem params → impossível afinar o ruído do
    //     GTSAM sem recompilar). Os defaults aqui replicam os de backend.hpp. ---
    BackendParams backend_params;
    backend_params.keyframe_distance_m =
      declare_parameter<double>("backend_keyframe_distance_m", 0.75);
    backend_params.keyframe_angle_rad =
      declare_parameter<double>("backend_keyframe_angle_rad", 0.35);
    const double prior_sigma_xy = declare_parameter<double>("backend_prior_sigma_xy", 0.1);
    backend_params.prior_pose_sigma = Eigen::Vector3d(
      prior_sigma_xy, prior_sigma_xy,
      declare_parameter<double>("backend_prior_sigma_theta", 0.05));
    const double odom_sigma_xy = declare_parameter<double>("backend_odom_sigma_xy", 0.05);
    backend_params.default_odom_sigma = Eigen::Vector3d(
      odom_sigma_xy, odom_sigma_xy,
      declare_parameter<double>("backend_odom_sigma_theta", 0.03));
    backend_params.default_bearing_sigma_rad =
      declare_parameter<double>("backend_bearing_sigma_rad", 0.05);
    backend_params.default_range_sigma_m =
      declare_parameter<double>("backend_range_sigma_m", 0.15);
    backend_params.constellation_distance_sigma_m =
      declare_parameter<double>("backend_constellation_sigma_m", 0.1);
    backend_params.use_robust_kernels =
      declare_parameter<bool>("backend_use_robust_kernels", true);
    backend_params.robust_huber_k =
      declare_parameter<double>("backend_robust_huber_k", 1.345);
    backend_ = std::make_unique<TreeSlamBackend>(backend_params);
    backend_default_odom_sigma_ = backend_params.default_odom_sigma;

    // --- Modelo de ruído de odom PROPORCIONAL AO MOVIMENTO (substitui o sigma
    //     fixo de 5 cm que o nó nunca preenchia). Entre keyframes, a incerteza
    //     do delta cresce com a translação e a rotação percorridas — em terreno
    //     irregular a odom derrapa muito mais que 5 cm, e um sigma fixo apertado
    //     fazia o backend confiar cego na odom e ignorar a correção dos troncos.
    odom_sigma_base_xy_ = declare_parameter<double>("odom_sigma_base_xy", 0.05);
    odom_sigma_per_trans_ = declare_parameter<double>("odom_sigma_per_trans", 0.15);
    odom_sigma_per_rot_xy_ = declare_parameter<double>("odom_sigma_per_rot_xy", 0.10);
    odom_sigma_base_theta_ = declare_parameter<double>("odom_sigma_base_theta", 0.02);
    odom_sigma_per_rot_ = declare_parameter<double>("odom_sigma_per_rot", 0.10);
    use_motion_odom_sigma_ = declare_parameter<bool>("use_motion_odom_sigma", true);

    // --- Ruído da observação bearing-range DEPENDENTE DO ALCANCE. O DBH/centro
    //     de um tronco visto a 8-12 m com arco parcial é mal-condicionado (ver
    //     dbh_stability_arc_illposed); injetá-lo com o mesmo sigma de 15 cm de um
    //     tronco a 2 m faz observações más puxar a pose. range_sigma cresce com r.
    obs_bearing_sigma_rad_ = declare_parameter<double>("obs_bearing_sigma_rad", 0.05);
    obs_range_sigma_base_m_ = declare_parameter<double>("obs_range_sigma_base_m", 0.10);
    obs_range_sigma_per_m_ = declare_parameter<double>("obs_range_sigma_per_m", 0.03);
    use_range_dependent_obs_sigma_ =
      declare_parameter<bool>("use_range_dependent_obs_sigma", true);

    TrackerParams tracker_params;
    tracker_params.promote_prob = declare_parameter<double>("tracker_promote_prob", 0.70);
    tracker_params.promote_margin = declare_parameter<double>("tracker_promote_margin", 0.20);
    tracker_params.promote_min_obs =
      static_cast<std::uint32_t>(declare_parameter<int>("tracker_promote_min_obs", 4));
    // Fusão de classe da câmara (F3).
    tracker_params.fusion_class_c_lidar =
      declare_parameter<double>("fusion_class_c_lidar", 0.85);
    tracker_params.fusion_class_w_cam = declare_parameter<double>("fusion_class_w_cam", 1.0);
    tracker_params.fusion_class_cam_min_conf =
      declare_parameter<double>("fusion_class_cam_min_conf", 0.40);
    tracker_params.birth_confidence =
      declare_parameter<double>("tracker_birth_confidence", 0.3);
    tracker_params.birth_max_range_m =
      declare_parameter<double>("tracker_birth_max_range_m", 8.0);
    // Scorer dinâmico (S) + paralaxe
    tracker_params.score_gain =
      declare_parameter<double>("tracker_score_gain", 0.15);
    tracker_params.score_novelty_repeat =
      declare_parameter<double>("tracker_score_novelty_repeat", 0.10);
    tracker_params.score_consistency_sigma_m =
      declare_parameter<double>("tracker_score_consistency_sigma_m", 0.30);
    tracker_params.score_inconsistent_residual_m =
      declare_parameter<double>("tracker_score_inconsistent_residual_m", 0.5);
    tracker_params.score_penalty =
      declare_parameter<double>("tracker_score_penalty", 0.10);
    tracker_params.parallax_bin_count =
      declare_parameter<int>("tracker_parallax_bin_count", 24);
    tracker_params.promote_min_parallax_bins =
      declare_parameter<int>("tracker_promote_min_parallax_bins", 4);
    tracker_params.promote_score_min =
      declare_parameter<double>("tracker_promote_score_min", 0.5);
    tracker_params.class_min_bearing_delta_rad =
      declare_parameter<double>("tracker_class_min_bearing_delta_rad", 0.15);
    tracker_params.class_correlated_obs_weight =
      declare_parameter<double>("tracker_class_correlated_obs_weight", 0.15);
    // --- Re-associação geométrica de solo (loop closure terrestre) — antes
    //     hardcoded em tracker.hpp; exposto p/ afinar o gargalo da deriva. ---
    tracker_params.enable_geometric_reassoc =
      declare_parameter<bool>("tracker_enable_geometric_reassoc", true);
    tracker_params.geo_min_query =
      declare_parameter<int>("tracker_geo_min_query", 4);
    tracker_params.ground_reloc.triangle_side_tolerance_m =
      declare_parameter<double>("geo_triangle_side_tolerance_m", 0.4);
    tracker_params.ground_reloc.planar_residual_threshold_m =
      declare_parameter<double>("geo_planar_residual_threshold_m", 0.4);
    tracker_params.ground_reloc.diameter_residual_threshold_m =
      declare_parameter<double>("geo_diameter_residual_threshold_m", 0.2);
    tracker_params.ground_reloc.min_overlap_ratio =
      declare_parameter<double>("geo_min_overlap_ratio", 0.5);
    tracker_params.ground_reloc.min_correspondences =
      declare_parameter<int>("geo_min_correspondences", 4);
    tracker_params.ground_reloc.min_triangle_side_m =
      declare_parameter<double>("geo_min_triangle_side_m", 0.5);
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

    // Alinhamento de constelação local (Fase 1): a CADA scan, alinha a
    // constelação observada com o mapa LOCAL (landmarks num raio) para corrigir
    // a deriva incremental antes de associar. Reusa o motor de triângulos do
    // relocalizador, mas com params próprios: mais permissivo no nº mínimo de
    // correspondências (a maioria dos scans vê poucos troncos — R3), mas com
    // redundância obrigatória (>=4: 3 pontos definem exatamente uma SE2, sem
    // margem para validar contra constelação ambígua — R4).
    local_align_enable_ = declare_parameter<bool>("local_align_enable", true);
    local_align_radius_m_ = declare_parameter<double>("local_align_radius_m", 15.0);
    local_align_max_residual_m_ = declare_parameter<double>("local_align_max_residual_m", 0.30);
    RelocalizerParams align_params;
    align_params.diameter_bin_max_m = reloc_params.diameter_bin_max_m;
    align_params.min_correspondences =
      declare_parameter<int>("local_align_min_correspondences", 4);
    align_params.min_overlap_ratio = declare_parameter<double>("local_align_min_overlap", 0.6);
    align_params.planar_residual_threshold_m =
      declare_parameter<double>("local_align_planar_residual_m", 0.5);
    // DBH é mal-condicionado (salta) — não filtrar associação por diâmetro aqui.
    align_params.diameter_residual_threshold_m =
      declare_parameter<double>("local_align_diameter_residual_m", 1.0);
    local_align_reloc_ = std::make_unique<TreeLocRelocalizer>(align_params);

    // Loop closure global no solo (próximo passo da rearquitetura). Periodicamente,
    // reconhece a constelação atual contra o mapa GLOBAL (ancorado em landmarks
    // MADUROS) e, se há erro de fecho, adiciona fatores que criam o ciclo no grafo
    // → o GTSAM des-distorce o caminho todo (o alinhamento local só trava a deriva
    // nova, não corrige o offset acumulado — R2). Suporte EXIGENTE: um loop closure
    // falso des-distorce ERRADO (catastrófico) — R4.
    loop_closure_enable_ = declare_parameter<bool>("loop_closure_enable", true);
    loop_closure_interval_scans_ =
      declare_parameter<int>("loop_closure_interval_scans", 10);
    loop_closure_min_fix_error_m_ =
      declare_parameter<double>("loop_closure_min_fix_error_m", 0.5);
    RelocalizerParams lc_params;
    lc_params.diameter_bin_max_m = reloc_params.diameter_bin_max_m;
    lc_params.min_correspondences = declare_parameter<int>("loop_closure_min_correspondences", 5);
    lc_params.min_overlap_ratio = declare_parameter<double>("loop_closure_min_overlap", 0.6);
    lc_params.planar_residual_threshold_m =
      declare_parameter<double>("loop_closure_planar_residual_m", 0.4);
    lc_params.diameter_residual_threshold_m =
      declare_parameter<double>("loop_closure_diameter_residual_m", 1.0);  // DBH mal-condicionado
    loop_closure_reloc_ = std::make_unique<TreeLocRelocalizer>(lc_params);

    // Fase 3 — posição do landmark a partir da nuvem multi-vista. Quando o buffer
    // multi-vista satura (boa cobertura/arco), a posição do ajuste de cilindro
    // (refit.cx/cy, já em t.xy) entra no backend como prior FORTE — passa a mandar
    // sobre a triangulação bearing×range (mal-condicionada a >8 m). Uma vez por uid.
    multiview_position_prior_enable_ =
      declare_parameter<bool>("multiview_position_prior_enable", true);
    multiview_position_prior_sigma_m_ =
      declare_parameter<double>("multiview_position_prior_sigma_m", 0.08);
    multiview_position_min_coverage_ =
      declare_parameter<double>("multiview_position_min_coverage", 0.30);
    multiview_position_min_frames_ =
      declare_parameter<int>("multiview_position_min_frames", 4);

    // Fusão câmara→LiDAR (F2: associação por projeção; ainda SEM fundir classe).
    // O landmark por-frame está em base_link → projeta-se na imagem com a extrínseca
    // estática base→câmara + K; associa-se à caixa do detetor que o contém.
    fusion_camera_enabled_ = declare_parameter<bool>("fusion_camera_enabled", true);
    fusion_cam_topic_ =
      declare_parameter<std::string>("fusion_cam_topic", "/perception/camera/detections");
    fusion_cam_info_topic_ =
      declare_parameter<std::string>("fusion_cam_info_topic", "/camera/camera_info");
    fusion_cam_sync_tol_ms_ = declare_parameter<double>("fusion_cam_sync_tol_ms", 150.0);
    fusion_cam_min_conf_ = declare_parameter<double>("fusion_cam_min_conf", 0.40);
    const auto extr = declare_parameter<std::vector<double>>(
      "fusion_cam_extrinsic_xyz", std::vector<double>{0.40, 0.0, 0.24});
    cam_extrinsic_ = Eigen::Vector3d(
      extr.size() > 0 ? extr[0] : 0.40, extr.size() > 1 ? extr[1] : 0.0,
      extr.size() > 2 ? extr[2] : 0.24);

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
    if (fusion_camera_enabled_) {
      sub_camera_dets_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        fusion_cam_topic_, 10,
        std::bind(&TreeSlamNode::on_camera_detections, this, std::placeholders::_1));
      sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        fusion_cam_info_topic_, rclcpp::QoS(1).transient_local(),
        std::bind(&TreeSlamNode::on_camera_info, this, std::placeholders::_1));
    }
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

  // Intrínsecos da câmara (uma vez; transient_local garante a entrega tardia).
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    cam_intrinsics_.fx = msg->k[0];
    cam_intrinsics_.cx = msg->k[2];
    cam_intrinsics_.fy = msg->k[4];
    cam_intrinsics_.cy = msg->k[5];
    cam_intrinsics_.width = static_cast<int>(msg->width);
    cam_intrinsics_.height = static_cast<int>(msg->height);
    cam_intrinsics_.valid = cam_intrinsics_.fx > 0.0 && cam_intrinsics_.width > 0;
  }

  // Deteções da câmara (bbox em px + classe). Guarda por stamp; a associação
  // por-frame faz-se no on_landmarks (mesmo padrão diferido dos tree_clusters).
  void on_camera_detections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    const double stamp_sec = stamp_to_sec(msg->header.stamp);
    camera_dets_by_stamp_[stamp_sec] = msg;
    while (camera_dets_by_stamp_.size() > 32) {
      camera_dets_by_stamp_.erase(camera_dets_by_stamp_.begin());
    }
  }

  // Deteções de câmara cujo stamp esteja dentro da tolerância de `stamp_sec`.
  vision_msgs::msg::Detection2DArray::ConstSharedPtr camera_dets_for_stamp(double stamp_sec) const
  {
    const double tol = fusion_cam_sync_tol_ms_ * 1.0e-3;
    double best_dt = tol;
    vision_msgs::msg::Detection2DArray::ConstSharedPtr best;
    for (const auto & kv : camera_dets_by_stamp_) {
      const double dt = std::abs(kv.first - stamp_sec);
      if (dt <= best_dt) {
        best_dt = dt;
        best = kv.second;
      }
    }
    return best;
  }

  // Taxonomia 4→3 do detetor para o índice de classe do SLAM [0=tronco, 1=rocha,
  // 2=obstáculo]. bush e fallen_log → obstáculo. Desconhecido → -1 (não funde).
  static int camera_class_to_index(const std::string & class_id)
  {
    if (class_id == "tree") {return 0;}
    if (class_id == "rock") {return 1;}
    if (class_id == "bush" || class_id == "fallen_log") {return 2;}
    return -1;
  }

  // F3 — associação câmara↔landmark por PROJEÇÃO + FUSÃO de classe. Para cada
  // deteção LiDAR do frame (base_link) com uid, projeta a base na imagem, procura
  // a caixa do detetor que a contém e injeta a classe da câmara no log-odds do
  // track (cap soft no tracker). Loga a taxa de associação.
  void associate_camera_detections(
    const forest_hybrid_msgs::msg::TreeLandmarkArray & msg, const TrackerUpdateReport & report,
    double stamp_sec)
  {
    if (!fusion_camera_enabled_ || !cam_intrinsics_.valid) {
      return;
    }
    const auto cam = camera_dets_for_stamp(stamp_sec);
    if (!cam || cam->detections.empty()) {
      return;
    }
    std::size_t n_proj = 0, n_assoc = 0;
    for (std::size_t i = 0; i < msg.trees.size() && i < report.detection_to_uid.size(); ++i) {
      if (report.detection_to_uid[i] == 0) {
        continue;
      }
      const Eigen::Vector3d p_base(msg.trees[i].base.x, msg.trees[i].base.y, msg.trees[i].base.z);
      const auto pr = project_base_to_image(p_base, cam_extrinsic_, cam_intrinsics_);
      if (!pr.valid) {
        continue;
      }
      ++n_proj;
      for (const auto & det : cam->detections) {
        if (det.results.empty() ||
          det.results.front().hypothesis.score < fusion_cam_min_conf_)
        {
          continue;
        }
        if (pixel_in_bbox(
            pr.u, pr.v, det.bbox.center.position.x, det.bbox.center.position.y,
            det.bbox.size_x, det.bbox.size_y))
        {
          ++n_assoc;
          ++cam_assoc_hits_;
          // F3 — funde a classe da câmara no log-odds deste landmark.
          const int cls = camera_class_to_index(det.results.front().hypothesis.class_id);
          if (cls >= 0) {
            const double bearing = std::atan2(p_base.y(), p_base.x());
            tracker_->fuse_camera_class(
              report.detection_to_uid[i], cls,
              det.results.front().hypothesis.score, bearing);
            ++cam_fused_;
          }
          break;
        }
      }
    }
    cam_assoc_attempts_ += n_proj;
    if (n_proj > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[cam] projetados=%zu associados=%zu fundidos(classe)=%zu (cum %zu/%zu) | %zu caixas",
        n_proj, n_assoc, cam_fused_, cam_assoc_hits_, cam_assoc_attempts_,
        cam->detections.size());
    }
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
    flush_matured_position_priors();
  }

  // Fase 3 — injeta no backend a posição da nuvem dos landmarks que acabaram de
  // saturar (uma vez por uid). A saturação é o gate de qualidade (cobertura/arco
  // multi-vista); a posição do ajuste de cilindro (t.xy) é mais fiável que a
  // triangulação bearing×range. Prior FORTE → manda sobre o bearing×range no grafo.
  void flush_matured_position_priors()
  {
    if (!multiview_position_prior_enable_) {
      return;
    }
    bool sent_any = false;
    for (const auto & t : tracker_->tracks()) {
      // A posição (centro do círculo) é bem-condicionada com MUITO menos cobertura
      // que o diâmetro preciso: não esperar a saturação total (cobertura ≥0.65, que
      // estas árvores raramente atingem) — basta um arco/nº de vistas suficiente.
      const auto & buf = t.multiview_buffer;
      const bool position_ready = buf.saturated() ||
        (buf.n_inlier_frames() >= static_cast<std::uint32_t>(multiview_position_min_frames_) &&
         buf.coverage_ratio() >= multiview_position_min_coverage_);
      if (!position_ready || !feeds_pose_graph(t.uid)) {
        continue;
      }
      if (position_prior_sent_.count(t.uid)) {
        continue;
      }
      backend_->add_landmark_position_prior(
        t.uid, t.xy,
        Eigen::Vector2d(multiview_position_prior_sigma_m_, multiview_position_prior_sigma_m_));
      position_prior_sent_.insert(t.uid);
      ++position_priors_sent_;
      sent_any = true;
      RCLCPP_INFO(
        get_logger(),
        "[mv-pos] landmark %lu saturou: posição pela nuvem (%.2f, %.2f) → prior no grafo (total %zu)",
        static_cast<unsigned long>(t.uid), t.xy.x(), t.xy.y(), position_priors_sent_);
    }
    if (sent_any) {
      backend_->optimize();  // aplica os priors novos (raro: uma vez por landmark)
    }
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

  // Fase 1 — alinhamento de constelação local. Dada a constelação observada
  // (deteções já em mundo, com a pose PREVISTA/derivada) e o mapa local
  // (landmarks do grafo num raio à volta da pose prevista), estima a SE(2) `T`
  // que faz a observação coincidir com o mapa. `T` é a deriva incremental a
  // corrigir. Devolve nullopt quando não há troncos suficientes / o alinhamento
  // não é fiável → o chamador faz fallback à odom (R3). Não toca no backend:
  // a correção entra pelas observações já consistentes, sem fator extra (R1).
  std::optional<Pose2> compute_local_alignment(
    const std::vector<TreeDetection> & detections_world, const Pose2 & predicted_world_pose) const
  {
    if (!local_align_enable_) {
      return std::nullopt;
    }
    std::vector<LandmarkPoint> query;
    query.reserve(detections_world.size());
    for (const auto & d : detections_world) {
      query.push_back(LandmarkPoint{0, d.x, d.y, d.diameter});
    }
    std::vector<LandmarkPoint> local_map;
    const Eigen::Vector2d robot_xy(predicted_world_pose.x, predicted_world_pose.y);
    for (const auto uid : backend_->all_landmark_uids()) {
      if (!feeds_pose_graph(uid)) {
        continue;
      }
      const Eigen::Vector2d xy = backend_->landmark_position(uid);
      if ((xy - robot_xy).norm() > local_align_radius_m_) {
        continue;
      }
      const auto * track = find_track(uid);
      local_map.push_back(LandmarkPoint{uid, xy.x(), xy.y(), track ? track->diameter : 0.0});
    }
    if (local_map.size() < 3) {
      return std::nullopt;
    }
    const auto result = local_align_reloc_->relocalize(query, local_map);
    if (!result.accepted || result.mean_residual_m > local_align_max_residual_m_) {
      return std::nullopt;  // sem alinhamento fiável → fallback odom
    }
    return result.map_to_query_transform;  // T: query(mundo derivado) -> mapa(grafo)
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

    // Fase 1 — alinhamento de constelação local. Estima `T` (deriva incremental)
    // alinhando a constelação observada ao mapa local e CORRIGE pose+deteções de
    // forma consistente (mesmo frame do grafo), antes de associar. Sem `T` fiável
    // (poucos troncos / ambíguo), fica a pose prevista da odom (fallback, R3).
    Pose2 aligned_world_pose = predicted_world_pose;
    if (const auto t = compute_local_alignment(detections_world, predicted_world_pose)) {
      aligned_world_pose = compose(*t, predicted_world_pose);
      const double tc = std::cos(t->theta), ts = std::sin(t->theta);
      Eigen::Matrix2d rt;
      rt << tc, -ts, ts, tc;
      for (auto & d : detections_world) {
        const Eigen::Vector2d w = transform_point(*t, d.x, d.y);
        d.x = w.x();
        d.y = w.y();
        d.base_covariance.topLeftCorner<2, 2>() =
          rt * d.base_covariance.topLeftCorner<2, 2>() * rt.transpose();
      }
      ++local_align_hits_;
    }
    ++local_align_attempts_;

    const auto report = tracker_->update(
      detections_world, stamp_sec,
      Eigen::Vector2d(aligned_world_pose.x, aligned_world_pose.y),
      angular_delta_since_last_scan);

    // Guarda este frame (pose + uid por deteção) para ingestão multi-vista diferida,
    // e tenta logo fechar o par caso o cluster deste stamp já tenha chegado.
    store_pending_multiview(stamp_sec, aligned_world_pose, detections_world, report);
    try_flush_multiview(stamp_sec);

    // F2 — associa as deteções LiDAR deste frame às caixas da câmara por projeção
    // (logging da taxa; ainda não funde classe — isso é a F3).
    associate_camera_detections(*msg, report, stamp_sec);

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
      const double align_frac = local_align_attempts_ > 0 ?
        static_cast<double>(local_align_hits_) / static_cast<double>(local_align_attempts_) : 0.0;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[assoc] dets=%zu assoc=%zu (geo_reawaken=%zu) birth=%zu | mapa: ativos=%zu adormecidos=%zu"
        " | align=%.0f%% loop=%zu",
        detections_world.size(), n_assoc_total, report.reawakened.size(), report.births.size(),
        n_active, n_dormant, align_frac * 100.0, loop_closure_fires_);
    }

    // GROUND: decide se abre keyframe nova (acumula odom) e atribui o índice
    // de keyframe ao qual as observações deste scan vão ser ligadas.
    std::size_t obs_keyframe = backend_->n_keyframes() - 1;
    bool opened_keyframe = false;
    if (backend_->should_open_keyframe(delta_since_keyframe)) {
      // Passa a covariância do delta (proporcional ao movimento) em vez de
      // deixar o backend cair no sigma fixo de 5 cm.
      obs_keyframe = backend_->add_odom_keyframe(
        delta_since_keyframe, odom_sigma_for_delta(delta_since_keyframe));
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
    // test_node_glue.cpp). Fase 1: as deteções estão no frame ALINHADO (corrigido
    // por `T`), por isso a keyframe nova usa `aligned_world_pose` para coincidir
    // o frame — a correção de pose flui pelas observações, sem fator extra (R1).
    const Pose2 obs_pose = opened_keyframe ? aligned_world_pose : last_kf_pose;

    // Fatores bearing-range: só landmarks promovidos tronco/rocha (obstáculo fora do grafo).
    for (std::size_t i = 0; i < detections_world.size(); ++i) {
      const LandmarkUid uid = report.detection_to_uid[i];
      if (uid == 0 || !feeds_pose_graph(uid)) {
        continue;
      }
      const BearingRange br =
        bearing_range_from(obs_pose, detections_world[i].x, detections_world[i].y);
      // Ruído da observação dependente do alcance (sentinela <0 → default backend
      // para ambos, p/ A/B limpo).
      const double range_sigma = obs_range_sigma_for(br.range);
      const std::optional<double> bearing_opt = use_range_dependent_obs_sigma_ ?
        std::optional<double>(obs_bearing_sigma_rad_) : std::nullopt;
      backend_->add_tree_observation(
        uid, obs_keyframe, br.bearing, br.range,
        Eigen::Vector2d(detections_world[i].x, detections_world[i].y),
        bearing_opt,
        range_sigma > 0.0 ? std::optional<double>(range_sigma) : std::nullopt);
    }
    // Rigidez de constelação entre troncos vistos no MESMO scan (geometria
    // local crua em base_link, mais fiável que a estimativa em mundo).
    for (std::size_t a = 0; a < msg->trees.size(); ++a) {
      for (std::size_t b = a + 1; b < msg->trees.size(); ++b) {
        const LandmarkUid ua = report.detection_to_uid[a], ub = report.detection_to_uid[b];
        if (ua == 0 || ub == 0 || !feeds_pose_graph(ua) || !feeds_pose_graph(ub)) {
          continue;
        }
        // Dedup (bug #5): liga cada par UMA vez. Repetir o mesmo fator a cada scan
        // torna a distância artificialmente certa e incha o grafo sem limite.
        const auto pair = std::minmax(ua, ub);
        if (!constellation_pairs_.emplace(pair.first, pair.second).second) {
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

    // Loop closure global: des-distorce o grafo quando reconhece uma região
    // antiga (âncoras maduras) e a odom acumulou erro de fecho. `predicted_world_pose`
    // é a pose por ODOM pura (não a corrigida pelo alinhamento local) — de propósito.
    try_ground_loop_closure(*msg, predicted_world_pose, obs_keyframe);

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

  // Loop closure global no solo. Periodicamente, reconhece a constelação atual
  // (deteções CRUAS em base_link) contra os landmarks MADUROS do mapa global e,
  // se a pose reconhecida discorda da pose por ODOMETRIA pura (erro de fecho
  // acumulado), re-observa esses landmarks a partir da keyframe atual → cria o
  // ciclo no grafo → o GTSAM des-distorce o caminho todo. Comparar com a pose por
  // odom (não a já-corrigida pelo alinhamento local) é deliberado: senão a
  // correção local MASCARA o erro de fecho e o loop nunca disparava (R2).
  void try_ground_loop_closure(
    const forest_hybrid_msgs::msg::TreeLandmarkArray & msg,
    const Pose2 & odom_predicted_pose, std::size_t obs_keyframe)
  {
    if (!loop_closure_enable_) {
      return;
    }
    if (++loop_closure_scan_counter_ < loop_closure_interval_scans_) {
      return;
    }
    loop_closure_scan_counter_ = 0;
    if (msg.trees.size() < 4) {
      return;  // constelação insuficiente para um fecho fiável (R4)
    }

    std::vector<LandmarkPoint> query;
    query.reserve(msg.trees.size());
    for (const auto & tree : msg.trees) {
      query.push_back(LandmarkPoint{0, tree.base.x, tree.base.y, tree.diameter});
    }
    // Âncoras: só landmarks MADUROS (confirmados por paralaxe) — fiáveis para
    // fechar o laço; landmarks recentes/imaturos estão eles próprios derivados.
    std::vector<LandmarkPoint> map_points;
    for (const auto & t : tracker_->tracks()) {
      if (!tracker_->is_confirmed(t) || !feeds_pose_graph(t.uid)) {
        continue;
      }
      const Eigen::Vector2d xy = backend_->landmark_position(t.uid);
      map_points.push_back(LandmarkPoint{t.uid, xy.x(), xy.y(), t.diameter});
    }
    if (map_points.size() < 5) {
      return;
    }

    const auto result = loop_closure_reloc_->relocalize(query, map_points);
    if (!result.accepted) {
      return;
    }
    // `map_to_query_transform` leva a query (base_link) -> map = pose do robô
    // segundo as âncoras maduras. Erro de fecho = vs a pose por odom pura.
    const Pose2 reloc_pose = result.map_to_query_transform;
    const double fix_error =
      std::hypot(reloc_pose.x - odom_predicted_pose.x, reloc_pose.y - odom_predicted_pose.y);
    if (fix_error < loop_closure_min_fix_error_m_) {
      return;  // odom ainda consistente com as âncoras → nada a des-distorcer
    }

    for (const auto & c : result.correspondences) {
      const auto & q = query[c.query_index];
      const double range = std::hypot(q.x, q.y);
      const double bearing = std::atan2(q.y, q.x);
      backend_->add_relocalization_factor(c.map_uid, obs_keyframe, bearing, range);
    }
    backend_->optimize();
    ++loop_closure_fires_;
    RCLCPP_INFO(
      get_logger(),
      "[loop] fecho global: erro_odom=%.2fm overlap=%.2f %zu âncoras maduras → des-distorce",
      fix_error, result.overlap_ratio, result.correspondences.size());
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
        [this](const LandmarkTrack & t) {
          return tracker_->is_confirmed(t) && is_map_output_class(t.committed_class);
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
      // MAPA/inventário: só CONFIRMADOS (promovido + paralaxe + score). Antes era
      // is_promoted (4 obs do mesmo ângulo já punham a árvore no mapa → "verde
      // logo no 1.º ângulo"); agora exige vistas de ângulos distintos.
      if (!tracker_->is_confirmed(t) || !is_map_output_class(t.committed_class)) {
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
      // Confiança publicada = S (scorer dinâmico: qualidade × paralaxe ×
      // consistência). Cresce devagar com vistas boas de ângulos diferentes; não
      // satura a 1.0 com o robô parado. A classe vai em semantic_class; o volume
      // de evidência em n_observations.
      m.confidence = static_cast<float>(t.score);
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
      // Markers no RViz: só CONFIRMADOS (paralaxe), igual ao /slam/tree_map.
      if (!tracker_->is_confirmed(t) || !is_map_output_class(t.committed_class)) {
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

    // ARESTAS de vizinhança (constelação): uma linha por par de landmarks ligados
    // por um fator de rigidez no grafo. É o que responde a "que vizinho está ligado
    // a que árvore" — a topologia do grafo, antes invisível no RViz.
    {
      visualization_msgs::msg::Marker edges;
      edges.header.stamp = now();
      edges.header.frame_id = map_frame_;
      edges.ns = "tree_slam_constellation";
      edges.id = 0;
      edges.type = visualization_msgs::msg::Marker::LINE_LIST;
      edges.action = visualization_msgs::msg::Marker::ADD;
      edges.scale.x = 0.03;  // espessura da linha
      edges.color.r = 0.9F;
      edges.color.g = 0.9F;
      edges.color.b = 0.2F;  // amarelo ténue
      edges.color.a = 0.5F;
      edges.pose.orientation.w = 1.0;
      for (const auto & pr : constellation_pairs_) {
        if (!backend_->has_landmark(pr.first) || !backend_->has_landmark(pr.second)) {
          continue;
        }
        const Eigen::Vector2d pa = backend_->landmark_position(pr.first);
        const Eigen::Vector2d pb = backend_->landmark_position(pr.second);
        geometry_msgs::msg::Point ga;
        ga.x = pa.x();
        ga.y = pa.y();
        ga.z = 0.0;
        geometry_msgs::msg::Point gb;
        gb.x = pb.x();
        gb.y = pb.y();
        gb.z = 0.0;
        edges.points.push_back(ga);
        edges.points.push_back(gb);
      }
      if (!edges.points.empty()) {
        markers.markers.push_back(edges);
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
      // A correção map->odom é só deriva no PLANO (x, y, yaw). Projetar o
      // odom->base para SE(2) antes de inverter — senão consome o roll/pitch/z
      // (atitude SE3 do EKF) e o map->odom CANCELA-OS, achatando o frame `map`
      // (terreno e landmarks inclinam quando o robô inclina). Pura SE(2): a
      // atitude do odom->base passa intacta para map->base.
      const double ob_yaw = tf2::getYaw(t_odom_base_tf.getRotation());
      tf2::Transform t_odom_base_se2;
      t_odom_base_se2.setOrigin(tf2::Vector3(
          t_odom_base_tf.getOrigin().x(), t_odom_base_tf.getOrigin().y(), 0.0));
      tf2::Quaternion q_ob;
      q_ob.setRPY(0.0, 0.0, ob_yaw);
      t_odom_base_se2.setRotation(q_ob);
      last_map_odom_ = t_map_base * t_odom_base_se2.inverse();
    }

    geometry_msgs::msg::TransformStamped out;
    // Pós-datar (now + tolerância) para o map->odom ser válido até à próxima
    // publicação — evita "extrapolation into the future" no Nav2/RPP.
    out.header.stamp = now() + rclcpp::Duration::from_seconds(tf_transform_tolerance_);
    out.header.frame_id = map_frame_;
    out.child_frame_id = odom_frame_;
    tf2::toMsg(last_map_odom_, out.transform);
    tf_broadcaster_.sendTransform(out);
  }

  // --- Parâmetros -----------------------------------------------------
  std::string map_frame_, odom_frame_, base_link_frame_;
  double publish_hz_{10.0};
  double tf_transform_tolerance_{0.2};
  double gnss_good_variance_m2_{4.0};
  int relocalization_max_scans_per_attempt_{10};
  Eigen::Vector3d aerial_hop_sigma_{3.0, 3.0, 0.3};

  // Modelo de ruído de odom proporcional ao movimento (ver construtor).
  bool use_motion_odom_sigma_{true};
  double odom_sigma_base_xy_{0.05};
  double odom_sigma_per_trans_{0.15};
  double odom_sigma_per_rot_xy_{0.10};
  double odom_sigma_base_theta_{0.02};
  double odom_sigma_per_rot_{0.10};
  // Ruído de observação dependente do alcance (ver construtor).
  bool use_range_dependent_obs_sigma_{true};
  double obs_bearing_sigma_rad_{0.05};
  double obs_range_sigma_base_m_{0.10};
  double obs_range_sigma_per_m_{0.03};

  // sigma (x,y,theta) do delta de odom entre keyframes, proporcional ao
  // movimento percorrido. |trans| e |rot| grandes (terreno irregular, viragens)
  // → mais incerteza → o backend deixa os troncos corrigir a pose.
  Eigen::Vector3d odom_sigma_for_delta(const Pose2 & delta) const
  {
    if (!use_motion_odom_sigma_) {
      return backend_default_odom_sigma_;
    }
    const double trans = std::hypot(delta.x, delta.y);
    const double rot = std::abs(delta.theta);
    const double sxy = odom_sigma_base_xy_ + odom_sigma_per_trans_ * trans +
      odom_sigma_per_rot_xy_ * rot;
    const double sth = odom_sigma_base_theta_ + odom_sigma_per_rot_ * rot;
    return Eigen::Vector3d(sxy, sxy, sth);
  }
  // sigma do alcance de uma observação a `range` m: cresce linearmente com a
  // distância (arco parcial mal-condicionado ao longe).
  double obs_range_sigma_for(double range_m) const
  {
    if (!use_range_dependent_obs_sigma_) {
      return -1.0;  // sentinela → usa o default do backend
    }
    return obs_range_sigma_base_m_ + obs_range_sigma_per_m_ * range_m;
  }
  Eigen::Vector3d backend_default_odom_sigma_{0.05, 0.05, 0.03};

  // --- Estado -----------------------------------------------------------
  std::unique_ptr<TreeSlamBackend> backend_;
  std::unique_ptr<LandmarkTracker> tracker_;
  std::unique_ptr<TreeLocRelocalizer> relocalizer_;
  std::unique_ptr<TreeLocRelocalizer> local_align_reloc_;  // Fase 1: alinhamento por-scan
  std::unique_ptr<TreeLocRelocalizer> loop_closure_reloc_;  // loop closure global no solo
  std::unique_ptr<ModeManager> mode_manager_;

  bool local_align_enable_{true};
  double local_align_radius_m_{15.0};
  double local_align_max_residual_m_{0.30};
  std::size_t local_align_hits_{0};       // scans com alinhamento aceite (R3: cobertura)
  std::size_t local_align_attempts_{0};   // scans GROUND processados

  bool loop_closure_enable_{true};
  int loop_closure_interval_scans_{10};
  double loop_closure_min_fix_error_m_{0.5};
  int loop_closure_scan_counter_{0};
  std::size_t loop_closure_fires_{0};     // loop closures aceites (des-distorções)

  bool multiview_position_prior_enable_{true};
  double multiview_position_prior_sigma_m_{0.08};
  double multiview_position_min_coverage_{0.30};
  int multiview_position_min_frames_{4};
  std::unordered_set<LandmarkUid> position_prior_sent_;  // uids já com prior da nuvem
  std::size_t position_priors_sent_{0};
  // Pares de landmarks já ligados por um fator de constelação (canónico min<max).
  // Dedup: o fator de rigidez entra UMA vez por par (bug #5: sem isto o mesmo par
  // era religado a cada scan → grafo artificialmente rígido). Também é a fonte das
  // ARESTAS de vizinhança no RViz (o utilizador vê que árvore liga a que vizinho).
  std::set<std::pair<LandmarkUid, LandmarkUid>> constellation_pairs_;

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
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_camera_dets_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;

  // Fusão câmara→LiDAR (F2)
  bool fusion_camera_enabled_{true};
  std::string fusion_cam_topic_;
  std::string fusion_cam_info_topic_;
  double fusion_cam_sync_tol_ms_{150.0};
  double fusion_cam_min_conf_{0.40};
  Eigen::Vector3d cam_extrinsic_{0.40, 0.0, 0.24};
  CameraIntrinsics cam_intrinsics_;
  std::map<double, vision_msgs::msg::Detection2DArray::ConstSharedPtr> camera_dets_by_stamp_;
  std::size_t cam_assoc_hits_{0};
  std::size_t cam_assoc_attempts_{0};
  std::size_t cam_fused_{0};  // nº de fusões de classe da câmara (F3)
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

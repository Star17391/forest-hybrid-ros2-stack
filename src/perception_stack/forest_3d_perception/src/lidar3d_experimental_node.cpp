/**
 * @file lidar3d_experimental_node.cpp
 * @brief Parallel experimental LiDAR pipeline. Legacy node untouched.
 *
 * Stage 0 (CSF):          ground / non_ground segmentation.
 * Stage 1 (stem band):    nDSM band over non_ground + 2D XY clustering → clusters.
 *
 * Clustering uses the robotic-forestry pattern (trunk band + 2D clustering),
 * NOT naive Euclidean 3D over the whole non-ground cloud (which merges canopies).
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "forest_hybrid_msgs/msg/semantic_class.hpp"
#include "forest_hybrid_msgs/msg/tree_landmark_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "forest_3d_perception/cylinder_fit.hpp"
#include "forest_3d_perception/experimental/cluster_classifier.hpp"
#include "forest_3d_perception/experimental/columnar_ground_recovery.hpp"
#include "forest_3d_perception/experimental/object_on_ground_filter.hpp"
#include "forest_3d_perception/experimental/pipeline_debug.hpp"
#include "forest_3d_perception/experimental/sprint1_pipeline.hpp"
#include "forest_3d_perception/experimental/stem_band_clustering.hpp"
#include "forest_3d_perception/experimental/stem_region_growing.hpp"

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

namespace
{

forest_3d_perception::experimental::CsfParams load_csf_params(rclcpp::Node & node)
{
  forest_3d_perception::experimental::CsfParams p;
  p.cloth_resolution = node.get_parameter("csf.cloth_resolution").as_double();
  p.rigidness = node.get_parameter("csf.rigidness").as_int();
  p.iterations = node.get_parameter("csf.iterations").as_int();
  p.class_threshold = node.get_parameter("csf.class_threshold").as_double();
  p.time_step = node.get_parameter("csf.time_step").as_double();
  p.slope_smooth = node.get_parameter("csf.slope_smooth").as_bool();
  return p;
}

forest_3d_perception::experimental::ClusteringParams load_clustering_params(rclcpp::Node & node)
{
  forest_3d_perception::experimental::ClusteringParams p;
  p.tolerance = node.get_parameter("clustering.tolerance").as_double();
  p.min_cluster_size = node.get_parameter("clustering.min_cluster_size").as_int();
  p.max_cluster_size = node.get_parameter("clustering.max_cluster_size").as_int();
  return p;
}

forest_3d_perception::experimental::StemBandParams load_stem_params(rclcpp::Node & node)
{
  forest_3d_perception::experimental::StemBandParams p;
  p.ground_grid_resolution_m =
    static_cast<float>(node.get_parameter("stem.ground_grid_resolution_m").as_double());
  p.band_min_m = static_cast<float>(node.get_parameter("stem.band_min_m").as_double());
  p.band_max_m = static_cast<float>(node.get_parameter("stem.band_max_m").as_double());
  p.cluster_tolerance_m =
    static_cast<float>(node.get_parameter("stem.cluster_tolerance_m").as_double());
  p.cluster_min_pts = node.get_parameter("stem.cluster_min_pts").as_int();
  p.cluster_max_pts = node.get_parameter("stem.cluster_max_pts").as_int();
  return p;
}

forest_3d_perception::experimental::ObjectOnGroundParams load_object_filter_params(
  rclcpp::Node & node)
{
  forest_3d_perception::experimental::ObjectOnGroundParams p;
  p.enabled = node.get_parameter("csf_post.enabled").as_bool();
  p.cell_m = static_cast<float>(node.get_parameter("csf_post.cell_m").as_double());
  p.window_m = static_cast<float>(node.get_parameter("csf_post.window_m").as_double());
  p.percentile = static_cast<float>(node.get_parameter("csf_post.percentile").as_double());
  p.height_threshold_m =
    static_cast<float>(node.get_parameter("csf_post.height_threshold_m").as_double());
  p.min_points_per_cell = node.get_parameter("csf_post.min_points_per_cell").as_int();
  return p;
}

forest_3d_perception::experimental::ColumnarRecoveryParams load_columnar_recovery_params(
  rclcpp::Node & node)
{
  forest_3d_perception::experimental::ColumnarRecoveryParams p;
  p.enabled = node.get_parameter("columnar_recovery.enabled").as_bool();
  p.cell_m = static_cast<float>(node.get_parameter("columnar_recovery.cell_m").as_double());
  p.ground_margin_m =
    static_cast<float>(node.get_parameter("columnar_recovery.ground_margin_m").as_double());
  p.min_object_height_m =
    static_cast<float>(node.get_parameter("columnar_recovery.min_object_height_m").as_double());
  p.neighbor_radius_cells = node.get_parameter("columnar_recovery.neighbor_radius_cells").as_int();
  p.floor_window_cells = node.get_parameter("columnar_recovery.floor_window_cells").as_int();
  p.floor_percentile =
    static_cast<float>(node.get_parameter("columnar_recovery.floor_percentile").as_double());
  p.max_recover_m =
    static_cast<float>(node.get_parameter("columnar_recovery.max_recover_m").as_double());
  return p;
}

forest_3d_perception::experimental::RegionGrowParams load_region_grow_params(rclcpp::Node & node)
{
  forest_3d_perception::experimental::RegionGrowParams p;
  p.enabled = node.get_parameter("region_grow.enabled").as_bool();
  p.ground_grid_resolution_m =
    static_cast<float>(node.get_parameter("region_grow.ground_grid_resolution_m").as_double());
  p.ground_inpaint_passes = node.get_parameter("region_grow.ground_inpaint_passes").as_int();
  p.seed_max_hag_m =
    static_cast<float>(node.get_parameter("region_grow.seed_max_hag_m").as_double());
  p.growth_radius_m =
    static_cast<float>(node.get_parameter("region_grow.growth_radius_m").as_double());
  p.growth_z_scale =
    static_cast<float>(node.get_parameter("region_grow.growth_z_scale").as_double());
  p.growth_max_hag_m =
    static_cast<float>(node.get_parameter("region_grow.growth_max_hag_m").as_double());
  p.ground_tolerance_m =
    static_cast<float>(node.get_parameter("region_grow.ground_tolerance_m").as_double());
  p.min_region_pts = node.get_parameter("region_grow.min_region_pts").as_int();
  p.max_region_pts = node.get_parameter("region_grow.max_region_pts").as_int();
  return p;
}

forest_3d_perception::experimental::ClassifierParams load_classify_params(rclcpp::Node & node)
{
  forest_3d_perception::experimental::ClassifierParams p;
  p.slice_height_m =
    static_cast<float>(node.get_parameter("classify.slice_height_m").as_double());
  p.slice_min_pts = node.get_parameter("classify.slice_min_pts").as_int();
  p.trunk_radius_grow_factor =
    static_cast<float>(node.get_parameter("classify.trunk_radius_grow_factor").as_double());
  p.trunk_radius_abs_margin_m =
    static_cast<float>(node.get_parameter("classify.trunk_radius_abs_margin_m").as_double());
  p.trunk_center_jump_m =
    static_cast<float>(node.get_parameter("classify.trunk_center_jump_m").as_double());
  p.trunk_core_min_height_m =
    static_cast<float>(node.get_parameter("classify.trunk_core_min_height_m").as_double());
  p.trunk_min_verticality =
    static_cast<float>(node.get_parameter("classify.trunk_min_verticality").as_double());
  p.trunk_min_linearity =
    static_cast<float>(node.get_parameter("classify.trunk_min_linearity").as_double());
  p.canopy_min_count = node.get_parameter("canopy.min_count").as_int();
  p.rock_max_height_m =
    static_cast<float>(node.get_parameter("classify.rock_max_height_m").as_double());
  p.rock_max_aspect =
    static_cast<float>(node.get_parameter("classify.rock_max_aspect").as_double());
  p.rock_max_surface_variation =
    static_cast<float>(node.get_parameter("classify.rock_max_surface_variation").as_double());
  p.rock_max_scatter =
    static_cast<float>(node.get_parameter("classify.rock_max_scatter").as_double());
  p.rock_max_local_roughness =
    static_cast<float>(node.get_parameter("classify.rock_max_local_roughness").as_double());
  p.shrub_min_scatter =
    static_cast<float>(node.get_parameter("classify.shrub_min_scatter").as_double());
  p.shrub_max_height_m =
    static_cast<float>(node.get_parameter("classify.shrub_max_height_m").as_double());
  p.min_points = node.get_parameter("classify.min_points").as_int();
  return p;
}

// Per-point semantic label ids for the /perception/lidar/semantic_points cloud.
//   0 unknown | 1 ground | 2 trunk | 3 rock | 4 shrub | 5 obstacle
constexpr float kSemUnknown = 0.0f;
constexpr float kSemGround = 1.0f;

float semantic_id(forest_3d_perception::experimental::ClusterClass c)
{
  using CC = forest_3d_perception::experimental::ClusterClass;
  switch (c) {
    case CC::Trunk: return 2.0f;
    case CC::Rock: return 3.0f;
    case CC::Shrub: return 4.0f;
    case CC::Obstacle: return 5.0f;
    default: return kSemUnknown;
  }
}

float semantic_id_from_scores(const std::array<float, 3> & scores)
{
  using forest_3d_perception::experimental::cluster_class_from_scores;
  return semantic_id(cluster_class_from_scores(scores));
}

uint8_t tree_landmark_semantic_class(const std::array<float, 3> & scores)
{
  using forest_hybrid_msgs::msg::SemanticClass;
  using forest_3d_perception::experimental::argmax_class_index;
  using forest_3d_perception::experimental::kScoreTrunk;
  if (argmax_class_index(scores) == static_cast<int>(kScoreTrunk)) {
    return SemanticClass::CLASS_VEGETATION_RIGID;
  }
  return SemanticClass::CLASS_OBSTACLE;
}

/** Bbox geometry for clusters without a valid vertical-cylinder fit. */
void fill_bbox_landmark_geometry(
  forest_hybrid_msgs::msg::TreeLandmark & t,
  const forest_3d_perception::experimental::PointCluster & cl)
{
  const auto & pts = cl.cloud->points;
  float xmin = pts[0].x;
  float xmax = pts[0].x;
  float ymin = pts[0].y;
  float ymax = pts[0].y;
  float zmin = pts[0].z;
  float zmax = pts[0].z;
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  for (const auto & p : pts) {
    sx += p.x;
    sy += p.y;
    sz += p.z;
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
    zmin = std::min(zmin, p.z);
    zmax = std::max(zmax, p.z);
  }
  const double inv = 1.0 / static_cast<double>(pts.size());
  t.base.x = sx * inv;
  t.base.y = sy * inv;
  t.base.z = zmin;
  const float extent_x = xmax - xmin;
  const float extent_y = ymax - ymin;
  t.diameter = std::max(extent_x, extent_y);
  t.height = zmax - zmin;
  t.diameter_stddev = 0.2f * t.diameter;
  const float var_xy = (t.diameter * 0.25f) * (t.diameter * 0.25f);
  t.base_covariance = {
    var_xy, 0.0, 0.0,
    0.0, var_xy, 0.0,
    0.0, 0.0, 0.01f};
}

}  // namespace

class Lidar3dExperimentalNode : public rclcpp::Node
{
public:
  Lidar3dExperimentalNode()
  : Node("lidar3d_experimental_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    sprint1_.apply_params();
    stem_clusterer_.params = load_stem_params(*this);

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&Lidar3dExperimentalNode::on_cloud, this, std::placeholders::_1));

    pub_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, qos);
    pub_non_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(non_ground_topic_, qos);
    pub_stem_band_ = create_publisher<sensor_msgs::msg::PointCloud2>(stem_band_topic_, qos);
    pub_stem_candidates_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(stem_candidates_topic_, qos);
    pub_clusters_ = create_publisher<sensor_msgs::msg::PointCloud2>(clusters_topic_, qos);
    pub_cluster_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(cluster_markers_topic_, 10);
    pub_debug_ = create_publisher<std_msgs::msg::String>(debug_stats_topic_, 10);
    pub_semantic_points_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(semantic_points_topic_, qos);
    pub_tree_landmarks_ =
      create_publisher<forest_hybrid_msgs::msg::TreeLandmarkArray>(tree_landmarks_topic_, 10);
    pub_tree_landmark_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(tree_landmark_markers_topic_, 10);
    pub_trunk_fit_points_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(trunk_fit_points_topic_, qos);
    pub_tree_clusters_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(tree_clusters_topic_, qos);

    if (debug_publish_stages_) {
      pub_dbg_voxel_ = create_publisher<sensor_msgs::msg::PointCloud2>(dbg_voxel_topic_, qos);
      pub_dbg_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(dbg_ground_topic_, qos);
      pub_dbg_non_ground_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(dbg_non_ground_topic_, qos);
      pub_dbg_clusters_ = create_publisher<sensor_msgs::msg::PointCloud2>(dbg_clusters_topic_, qos);
    }

    if (gravity_align_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
          if (q.length2() < 1e-6) {
            return;
          }
          double yaw;
          tf2::Matrix3x3(q).getRPY(imu_roll_, imu_pitch_, yaw);
          imu_stamp_ = rclcpp::Time(msg->header.stamp);
          have_imu_ = true;
        });
    }

    param_cb_ = add_on_set_parameters_callback(
      std::bind(&Lidar3dExperimentalNode::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "experimental pipeline: debug_stage=%d (0=full 1=voxel 2=csf 3=cluster-3d)", debug_stage_);
    RCLCPP_INFO(get_logger(), "  IN  %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  OUT ground=%s", ground_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  OUT non_ground=%s", non_ground_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  OUT stem_band=%s", stem_band_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  OUT clusters=%s", clusters_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/sensors/lidar/points");
    processing_frame_ =
      declare_parameter<std::string>("processing_frame", "marble_hd2/base_link");
    tf_timeout_ = declare_parameter<double>("tf_timeout_sec", 0.5);
    enabled_ = declare_parameter<bool>("enabled", true);
    debug_stage_ = declare_parameter<int>("debug_stage", 0);
    debug_publish_stages_ = declare_parameter<bool>("debug_publish_stages", true);
    debug_log_interval_ = declare_parameter<int>("debug_log_interval", 30);

    voxel_leaf_ = declare_parameter<double>("voxel_leaf_size_m", 0.08);
    min_range_ = declare_parameter<double>("min_range_m", 0.3);
    max_range_ = declare_parameter<double>("max_range_m", 15.0);
    min_z_ = declare_parameter<double>("min_z_m", -1.0);
    max_z_ = declare_parameter<double>("max_z_m", 5.0);
    crop_min_points_ = declare_parameter<int>("crop_min_points", 30);

    ground_topic_ =
      declare_parameter<std::string>("ground_topic", "/perception/lidar3d/experimental/ground");
    non_ground_topic_ = declare_parameter<std::string>(
      "non_ground_topic", "/perception/lidar3d/experimental/non_ground");
    stem_band_topic_ = declare_parameter<std::string>(
      "stem_band_topic", "/perception/lidar3d/experimental/stem_band");
    stem_candidates_topic_ = declare_parameter<std::string>(
      "stem_candidates_topic", "/perception/lidar3d/experimental/stem_candidates");
    clusters_topic_ = declare_parameter<std::string>(
      "clusters_topic", "/perception/lidar3d/experimental/clusters");
    cluster_markers_topic_ = declare_parameter<std::string>(
      "cluster_markers_topic", "/perception/lidar3d/experimental/cluster_markers");
    debug_stats_topic_ = declare_parameter<std::string>(
      "debug_stats_topic", "/perception/lidar3d/experimental/debug_stats");

    // Layer-contract outputs (LiDAR perception → map/SLAM layer), in base_link.
    semantic_points_topic_ = declare_parameter<std::string>(
      "semantic_points_topic", "/perception/lidar/semantic_points");
    tree_landmarks_topic_ = declare_parameter<std::string>(
      "tree_landmarks_topic", "/perception/lidar/tree_landmarks");
    tree_landmark_markers_topic_ = declare_parameter<std::string>(
      "tree_landmark_markers_topic", "/perception/lidar/tree_landmark_markers");
    trunk_fit_points_topic_ = declare_parameter<std::string>(
      "trunk_fit_points_topic", "/perception/lidar3d/experimental/trunk_fit_points");
    // Inliers do fit por tronco, etiquetados com o índice da árvore em
    // tree_landmarks (campo intensity). Contrato STATELESS para o Tree-SLAM
    // acumular geometria multi-view por landmark. Mesmo stamp/frame que
    // tree_landmarks para o consumidor sincronizar.
    tree_clusters_topic_ = declare_parameter<std::string>(
      "tree_clusters_topic", "/perception/lidar/tree_clusters");

    dbg_voxel_topic_ = declare_parameter<std::string>(
      "debug_voxel_topic", "/perception/lidar3d/experimental/debug/stage_voxel");
    dbg_ground_topic_ = declare_parameter<std::string>(
      "debug_ground_topic", "/perception/lidar3d/experimental/debug/stage_ground");
    dbg_non_ground_topic_ = declare_parameter<std::string>(
      "debug_non_ground_topic", "/perception/lidar3d/experimental/debug/stage_non_ground");
    dbg_clusters_topic_ = declare_parameter<std::string>(
      "debug_clusters_topic", "/perception/lidar3d/experimental/debug/stage_clusters");

    declare_parameter<double>("csf.cloth_resolution", 0.5);
    declare_parameter<int>("csf.rigidness", 3);
    declare_parameter<int>("csf.iterations", 500);
    declare_parameter<double>("csf.class_threshold", 0.5);
    declare_parameter<double>("csf.time_step", 0.65);
    declare_parameter<bool>("csf.slope_smooth", true);

    // Euclidean 3D clustering — only used in debug_stage=3 (cluster-only, CSF bypass).
    declare_parameter<double>("clustering.tolerance", 0.25);
    declare_parameter<int>("clustering.min_cluster_size", 10);
    declare_parameter<int>("clustering.max_cluster_size", 5000);

    // Stem-band 2D clustering (the real method).
    declare_parameter<double>("stem.ground_grid_resolution_m", 0.20);
    declare_parameter<double>("stem.band_min_m", 0.30);
    declare_parameter<double>("stem.band_max_m", 3.00);
    declare_parameter<double>("stem.cluster_tolerance_m", 0.20);
    declare_parameter<int>("stem.cluster_min_pts", 6);
    declare_parameter<int>("stem.cluster_max_pts", 3000);

    // CSF post-pass: recover fallen rocks absorbed as ground.
    declare_parameter<bool>("csf_post.enabled", true);
    declare_parameter<double>("csf_post.cell_m", 0.20);
    declare_parameter<double>("csf_post.window_m", 1.00);
    declare_parameter<double>("csf_post.percentile", 0.15);
    declare_parameter<double>("csf_post.height_threshold_m", 0.12);
    declare_parameter<int>("csf_post.min_points_per_cell", 2);

    // Columnar ground recovery: pull stolen object bases by vertical connectivity.
    declare_parameter<bool>("columnar_recovery.enabled", true);
    declare_parameter<double>("columnar_recovery.cell_m", 0.20);
    declare_parameter<double>("columnar_recovery.ground_margin_m", 0.06);
    declare_parameter<double>("columnar_recovery.min_object_height_m", 0.40);
    declare_parameter<int>("columnar_recovery.neighbor_radius_cells", 1);
    declare_parameter<int>("columnar_recovery.floor_window_cells", 2);
    declare_parameter<double>("columnar_recovery.floor_percentile", 0.15);
    declare_parameter<double>("columnar_recovery.max_recover_m", 0.50);

    // Sprint 3.5 — Option 2: ground-seeded vertical region growing (default).
    // Anchors every cluster to the ground (rocks/trunks grow from the base,
    // floating canopy excluded) and does not fragment trees.
    declare_parameter<bool>("region_grow.enabled", true);
    declare_parameter<double>("region_grow.ground_grid_resolution_m", 0.20);
    declare_parameter<int>("region_grow.ground_inpaint_passes", 6);
    declare_parameter<double>("region_grow.seed_max_hag_m", 0.70);
    declare_parameter<double>("region_grow.growth_radius_m", 0.30);
    declare_parameter<double>("region_grow.growth_z_scale", 0.20);
    declare_parameter<double>("region_grow.growth_max_hag_m", 3.00);
    declare_parameter<double>("region_grow.ground_tolerance_m", 0.20);
    declare_parameter<int>("region_grow.min_region_pts", 5);
    declare_parameter<int>("region_grow.max_region_pts", 5000);

    // Sprint 3 (Option B): slice-based classification — TRUNK / ROCK / SHRUB /
    // OBSTACLE. Trunk = vertical stem core (shape, not width); OBSTACLE is the
    // only catch-all.
    declare_parameter<double>("classify.slice_height_m", 0.20);
    declare_parameter<int>("classify.slice_min_pts", 2);
    declare_parameter<double>("classify.trunk_radius_grow_factor", 2.2);
    declare_parameter<double>("classify.trunk_radius_abs_margin_m", 0.10);
    declare_parameter<double>("classify.trunk_center_jump_m", 0.30);
    declare_parameter<double>("classify.trunk_core_min_height_m", 0.80);
    declare_parameter<double>("classify.trunk_min_verticality", 0.55);
    declare_parameter<double>("classify.trunk_min_linearity", 0.40);
    declare_parameter<double>("classify.rock_max_height_m", 1.50);
    declare_parameter<double>("classify.rock_max_aspect", 1.20);
    declare_parameter<double>("classify.rock_max_surface_variation", 0.10);
    declare_parameter<double>("classify.rock_max_scatter", 0.35);
    declare_parameter<double>("classify.rock_max_local_roughness", 0.06);
    declare_parameter<double>("classify.shrub_min_scatter", 0.25);

    // Canopy check: tree iff there are non-ground points ABOVE the candidate's top
    // (relative to the candidate, not an absolute height — works for short trees).
    canopy_enabled_ = declare_parameter<bool>("canopy.enabled", true);
    canopy_cell_ = static_cast<float>(declare_parameter<double>("canopy.cell_m", 0.30));
    canopy_radius_ = static_cast<float>(declare_parameter<double>("canopy.radius_m", 1.20));
    canopy_margin_ = static_cast<float>(declare_parameter<double>("canopy.margin_m", 0.30));
    canopy_connect_step_ =
      static_cast<float>(declare_parameter<double>("canopy.connect_step_m", 0.50));
    declare_parameter<int>("canopy.min_count", 12);  // read by the classifier (crown bonus)
    declare_parameter<double>("classify.shrub_max_height_m", 1.50);
    declare_parameter<int>("classify.min_points", 4);

    // Vertical cylinder fit (centroid + median radius) for robust DBH (Sprint 4).
    declare_parameter<double>("cyl_fit.min_height_m", 0.30);
    declare_parameter<double>("cyl_fit.max_radius_m", 0.80);
    declare_parameter<double>("cyl_fit.max_rmse_m", 0.05);
    declare_parameter<double>("cyl_fit.min_inlier_ratio", 0.30);
    declare_parameter<double>("cyl_fit.inlier_dist_m", 0.04);
    declare_parameter<double>("cyl_fit.max_slice_height_m", 2.50);
    cyl_min_height_ = get_parameter("cyl_fit.min_height_m").as_double();
    cyl_max_radius_ = get_parameter("cyl_fit.max_radius_m").as_double();
    cyl_max_rmse_ = get_parameter("cyl_fit.max_rmse_m").as_double();
    cyl_min_inlier_ = get_parameter("cyl_fit.min_inlier_ratio").as_double();
    cyl_inlier_dist_ = get_parameter("cyl_fit.inlier_dist_m").as_double();
    cyl_max_slice_h_ = get_parameter("cyl_fit.max_slice_height_m").as_double();
    // Stem-aware DBH band: low = margem acima da base (ignora chão/raízes), high =
    // altura máx. de procura, stem_grow = quão maior que o raio base uma fatia pode
    // ser antes de ser considerada copa (corta aí). Ver cylinder_fit.hpp.
    cyl_dbh_band_low_ = declare_parameter<double>("cyl_fit.dbh_band_low_m", 0.15);
    cyl_dbh_band_high_ = declare_parameter<double>("cyl_fit.dbh_band_high_m", 2.5);
    cyl_stem_grow_ = declare_parameter<double>("cyl_fit.stem_grow_factor", 1.8);
    cyl_stem_axis_jump_ = declare_parameter<double>("cyl_fit.stem_axis_jump_m", 0.20);

    // Base-covariance model (uncertainty of the trunk-base centre).
    cov_range_k_ = declare_parameter<double>("base_cov.range_k", 0.01);
    cov_sigma_floor_m_ = declare_parameter<double>("base_cov.sigma_floor_m", 0.02);
    cov_sigma_z_m_ = declare_parameter<double>("base_cov.sigma_z_m", 0.05);

    // Caminho B — deteção SEM solo (troncos atrás/ao lado, onde o LiDAR inclinado
    // não vê chão; é o que fazia o SLAM ir a LOST). Reaproveita o MESMO clustering+
    // classificador; só admite por FORMA (sem nDSM). Emite com covariância inflada
    // (base não observada) -> o SLAM pesa-os menos mas mantém o tracking vivo.
    path_b_enabled_ = declare_parameter<bool>("path_b.enabled", true);
    path_b_cov_inflate_xy_ = declare_parameter<double>("path_b.cov_inflate_xy", 4.0);
    path_b_z_var_ = declare_parameter<double>("path_b.z_var_m2", 1.0);
    // Gate estrito de emissão do caminho B (correção > cobertura). Um tronco a
    // sério tem um core vertical ALTO e contínuo (>=1.5 m) + cilindro com DBH em
    // gama + largura contida + verticalidade/linearidade altas ao nível do cluster
    // (rejeita "agulhas" verticais esculpidas de copa/folhagem por meridianos).
    path_b_min_core_m_ = declare_parameter<double>("path_b.min_core_m", 1.50);
    path_b_dbh_min_m_ = declare_parameter<double>("path_b.dbh_min_m", 0.05);
    path_b_dbh_max_m_ = declare_parameter<double>("path_b.dbh_max_m", 0.60);
    path_b_max_hsize_m_ = declare_parameter<double>("path_b.max_hsize_m", 0.80);
    path_b_min_verticality_ = declare_parameter<double>("path_b.min_verticality", 0.70);
    path_b_min_linearity_ = declare_parameter<double>("path_b.min_linearity", 0.60);
    // Extensão vertical mínima do cluster do caminho B já na FORMAÇÃO: mata os
    // fragmentos de copa curtos (zspan ~0.04-1.4 m) antes de virarem cluster.
    path_b_min_zspan_m_ = declare_parameter<double>("path_b.min_zspan_m", 1.50);

    // Gravity alignment: correct roll/pitch via IMU before publishing base.
    imu_topic_ =
      declare_parameter<std::string>("gravity_align.imu_topic", "/sensors/imu/data");
    gravity_align_ = declare_parameter<bool>("gravity_align.enabled", true);
    imu_timeout_ = declare_parameter<double>("gravity_align.imu_timeout_sec", 0.5);

    reload_pipeline_params();
  }

  void reload_pipeline_params()
  {
    sprint1_.params.csf = load_csf_params(*this);
    sprint1_.params.clustering = load_clustering_params(*this);
    sprint1_.apply_params();
    stem_clusterer_.params = load_stem_params(*this);
    object_filter_.params = load_object_filter_params(*this);
    columnar_recovery_.params = load_columnar_recovery_params(*this);
    region_grower_.params = load_region_grow_params(*this);
    classifier_.params = load_classify_params(*this);
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (const auto & p : params) {
      const auto & n = p.get_name();
      try {
        if (n == "enabled") {
          enabled_ = p.as_bool();
        } else if (n == "debug_stage") {
          debug_stage_ = p.as_int();
        } else if (n == "debug_log_interval") {
          debug_log_interval_ = p.as_int();
        } else if (n == "voxel_leaf_size_m") {
          voxel_leaf_ = p.as_double();
        } else if (n == "min_range_m") {
          min_range_ = p.as_double();
        } else if (n == "max_range_m") {
          max_range_ = p.as_double();
        } else if (n == "min_z_m") {
          min_z_ = p.as_double();
        } else if (n == "max_z_m") {
          max_z_ = p.as_double();
        } else if (n == "crop_min_points") {
          crop_min_points_ = p.as_int();
        } else if (
          n == "csf.cloth_resolution" || n == "csf.rigidness" || n == "csf.iterations" ||
          n == "csf.class_threshold" || n == "csf.time_step" || n == "csf.slope_smooth" ||
          n == "clustering.tolerance" || n == "clustering.min_cluster_size" ||
          n == "clustering.max_cluster_size" ||
          n.rfind("stem.", 0) == 0 ||
          n.rfind("region_grow.", 0) == 0 ||
          n.rfind("csf_post.", 0) == 0 || n.rfind("classify.", 0) == 0 ||
          n.rfind("columnar_recovery.", 0) == 0)
        {
          /* reload below */
        } else {
          continue;
        }
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = e.what();
        return result;
      }
    }
    reload_pipeline_params();
    return result;
  }

  static std::size_t count_non_finite(const CloudT & cloud)
  {
    std::size_t n = 0;
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ++n;
      }
    }
    return n;
  }

  void crop_cloud(const CloudT::Ptr & in, CloudT::Ptr & out) const
  {
    out->clear();
    out->reserve(in->size());
    const double max_r2 = max_range_ * max_range_;
    const double min_r2 = min_range_ * min_range_;
    for (const auto & p : in->points) {
      // Drop non-finite returns FIRST: NaN/Inf survive every < / > test below
      // (all comparisons with NaN are false) and would reach CSF, where
      // int(NaN/step)=INT_MIN corrupts grid indexing -> SIGSEGV.
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
      if (r2 < min_r2 || r2 > max_r2) {
        continue;
      }
      if (p.z < min_z_ || p.z > max_z_) {
        continue;
      }
      out->push_back(p);
    }
    // All remaining points are finite: honor the PCL dense-cloud contract so
    // downstream filters (VoxelGrid) don't skip the finiteness fast-path.
    out->is_dense = true;
    out->height = 1;
    out->width = static_cast<std::uint32_t>(out->size());
  }

  void publish_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const CloudT::Ptr & cloud,
    const rclcpp::Time & stamp,
    const std::string & frame_id) const
  {
    if (!pub) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    if (!cloud || cloud->empty()) {
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id;
      msg.height = 1;
      msg.width = 0;
      pub->publish(msg);
      return;
    }
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    pub->publish(msg);
  }

  void publish_funnel(const forest_3d_perception::experimental::PipelineFunnelStats & funnel) const
  {
    if (!pub_debug_) {
      return;
    }
    std_msgs::msg::String dbg;
    dbg.data = funnel.to_json();
    pub_debug_->publish(dbg);
  }

  /** RGB for each class: trunk=blue, rock=red, shrub=green, unknown=gray. */
  static void class_color(
    forest_3d_perception::experimental::ClusterClass cls,
    float & r, float & g, float & b)
  {
    using CC = forest_3d_perception::experimental::ClusterClass;
    switch (cls) {
      case CC::Trunk:    r = 0.10f; g = 0.45f; b = 1.00f; break;  // blue
      case CC::Rock:     r = 1.00f; g = 0.15f; b = 0.10f; break;  // red
      case CC::Shrub:    r = 0.20f; g = 0.85f; b = 0.25f; break;  // green
      case CC::Obstacle: r = 1.00f; g = 0.55f; b = 0.00f; break;  // orange
      default:           r = 0.60f; g = 0.60f; b = 0.60f; break;  // gray
    }
  }

  void publish_cluster_markers(
    const std::vector<forest_3d_perception::experimental::PointCluster> & clusters,
    const std::vector<forest_3d_perception::experimental::ScoredCluster> & scored,
    const rclcpp::Time & stamp,
    const std::string & frame_id) const
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int mid = 0;
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      const auto & c = clusters[i];
      if (!c.cloud || c.cloud->empty()) {
        continue;
      }
      const auto & sc = scored[i];
      const float cx = sc.feat.centroid_x;
      const float cy = sc.feat.centroid_y;
      const float cz = sc.feat.centroid_z;

      float r;
      float g;
      float b;
      class_color(
        forest_3d_perception::experimental::cluster_class_from_scores(sc.class_scores), r, g, b);

      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id;
      m.ns = "experimental_clusters";
      m.id = mid++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = cx;
      m.pose.position.y = cy;
      m.pose.position.z = cz;
      m.scale.x = m.scale.y = m.scale.z = 0.25;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.85f;
      m.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(m);

      visualization_msgs::msg::Marker t;
      t.header.stamp = stamp;
      t.header.frame_id = frame_id;
      t.ns = "experimental_class_labels";
      t.id = mid++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose.position.x = cx;
      t.pose.position.y = cy;
      t.pose.position.z = cz + 0.5f * sc.feat.height_span + 0.3f;
      t.scale.z = 0.3;
      t.color.r = r;
      t.color.g = g;
      t.color.b = b;
      t.color.a = 1.0f;
      char buf[64];
      std::snprintf(
        buf, sizeof(buf), "T%.0f R%.0f O%.0f",
        sc.class_scores[0] * 100.0f, sc.class_scores[1] * 100.0f,
        sc.class_scores[2] * 100.0f);
      t.text = buf;
      t.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(t);
    }
    if (pub_cluster_markers_) {
      pub_cluster_markers_->publish(arr);
    }
  }

  void log_funnel_throttled(const forest_3d_perception::experimental::PipelineFunnelStats & f) const
  {
    if (debug_log_interval_ <= 0) {
      return;
    }
    ++frame_count_;
    if (frame_count_ % static_cast<std::size_t>(debug_log_interval_) != 0) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "FUNNEL status=%s stage=%d raw=%zu crop=%zu voxel=%zu ground=%zu nonground=%zu "
      "recovered=%zu band=%zu clusters=%zu csf_ground%%=%.1f in_frame=%s",
      forest_3d_perception::experimental::exit_status_string(f.status),
      f.debug_stage, f.n_raw, f.n_crop, f.n_voxel, f.n_ground, f.n_non_ground,
      n_recovered_last_, n_band_last_, f.n_clusters, f.csf_ground_pct, f.input_frame.c_str());
    if (f.status != forest_3d_perception::experimental::PipelineExitStatus::Ok) {
      RCLCPP_WARN(
        get_logger(), "Pipeline exit before full publish: %s",
        forest_3d_perception::experimental::exit_status_string(f.status));
    }
  }

  void publish_labeled_clusters(
    const CloudT::Ptr & non_ground,
    const std::vector<forest_3d_perception::experimental::PointCluster> & clusters,
    const std::vector<forest_3d_perception::experimental::ScoredCluster> & scored,
    const rclcpp::Time & stamp) const
  {
    if (non_ground && !non_ground->empty() && !clusters.empty()) {
      auto labeled = forest_3d_perception::experimental::EuclideanClustering::to_labeled_cloud(
        *non_ground, clusters);
      sensor_msgs::msg::PointCloud2 cluster_msg;
      pcl::toROSMsg(*labeled, cluster_msg);
      cluster_msg.header.stamp = stamp;
      cluster_msg.header.frame_id = processing_frame_;
      pub_clusters_->publish(cluster_msg);
      if (debug_publish_stages_ && pub_dbg_clusters_) {
        pub_dbg_clusters_->publish(cluster_msg);
      }
      publish_cluster_markers(clusters, scored, stamp, processing_frame_);
    } else if (pub_clusters_) {
      sensor_msgs::msg::PointCloud2 empty_msg;
      empty_msg.header.stamp = stamp;
      empty_msg.header.frame_id = processing_frame_;
      empty_msg.height = 1;
      empty_msg.width = 0;
      pub_clusters_->publish(empty_msg);
    }
  }

  /**
   * Layer-contract output #1: per-point semantic cloud (XYZI, intensity = class
   * id). Ground points -> ground; each classified cluster's points -> its class;
   * remaining non-ground -> unknown. Consumed by the costmap/volumetric map.
   */
  void publish_semantic_points(
    const CloudT::Ptr & ground,
    const CloudT::Ptr & non_ground,
    const std::vector<forest_3d_perception::experimental::PointCluster> & clusters,
    const std::vector<forest_3d_perception::experimental::ScoredCluster> & scored,
    const rclcpp::Time & stamp) const
  {
    if (!pub_semantic_points_) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr sem(new pcl::PointCloud<pcl::PointXYZI>);
    const std::size_t ng = non_ground ? non_ground->size() : 0;
    const std::size_t gr = ground ? ground->size() : 0;
    sem->reserve(ng + gr);

    if (ground) {
      for (const auto & p : ground->points) {
        pcl::PointXYZI q;
        q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = kSemGround;
        sem->push_back(q);
      }
    }
    if (non_ground) {
      std::vector<float> ng_label(ng, kSemUnknown);
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        const float id = semantic_id_from_scores(scored[i].class_scores);
        for (std::size_t idx : clusters[i].point_indices) {
          if (idx < ng) {
            ng_label[idx] = id;
          }
        }
      }
      for (std::size_t i = 0; i < ng; ++i) {
        const auto & p = non_ground->points[i];
        pcl::PointXYZI q;
        q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = ng_label[i];
        sem->push_back(q);
      }
    }
    sem->width = static_cast<std::uint32_t>(sem->size());
    sem->height = 1;
    sem->is_dense = true;

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*sem, out);
    out.header.stamp = stamp;
    out.header.frame_id = processing_frame_;
    pub_semantic_points_->publish(out);
  }

  // XY height grid of the non-ground cloud: cell -> z values. Used by the canopy
  // check to ask "are there points ABOVE this candidate's top, in its XY column?".
  using HeightGrid = std::unordered_map<std::int64_t, std::vector<float>>;

  static std::int64_t cell_key(float x, float y, float cell)
  {
    const std::int32_t ix = static_cast<std::int32_t>(std::floor(x / cell));
    const std::int32_t iy = static_cast<std::int32_t>(std::floor(y / cell));
    return (static_cast<std::int64_t>(ix) << 32) ^
           (static_cast<std::int64_t>(static_cast<std::uint32_t>(iy)));
  }

  static HeightGrid build_height_grid(const CloudT & cloud, float cell)
  {
    HeightGrid g;
    g.reserve(cloud.size());
    for (const auto & p : cloud.points) {
      g[cell_key(p.x, p.y, cell)].push_back(p.z);
    }
    return g;
  }

  /**
   * Canopy evidence WITH VERTICAL CONTINUITY. A tree has no gap between trunk and
   * crown — there are points right above its top. A rock has nothing contiguous
   * above; any points there (e.g. a neighbouring tree's crown) are far away with a
   * big vertical GAP. So we collect the column's points above the top, sort them,
   * and count only the ones that form an unbroken run upward: stop at the first gap
   * larger than connect_step. A rock with distant points above scores 0 (the gap
   * breaks the run immediately). RELATIVE to the candidate's top → works for short
   * trees. Cell-box approximation of the radius.
   */
  static int canopy_above(
    float cx, float cy, float z_top, const HeightGrid & grid,
    float cell, float radius, float margin, float connect_step)
  {
    const int rc = static_cast<int>(std::ceil(radius / cell));
    const std::int32_t cix = static_cast<std::int32_t>(std::floor(cx / cell));
    const std::int32_t ciy = static_cast<std::int32_t>(std::floor(cy / cell));
    const float z_min = z_top + margin;
    std::vector<float> zs_above;
    for (int dx = -rc; dx <= rc; ++dx) {
      for (int dy = -rc; dy <= rc; ++dy) {
        const std::int64_t key = (static_cast<std::int64_t>(cix + dx) << 32) ^
          (static_cast<std::int64_t>(static_cast<std::uint32_t>(ciy + dy)));
        auto it = grid.find(key);
        if (it == grid.end()) {
          continue;
        }
        for (float z : it->second) {
          if (z > z_min) {
            zs_above.push_back(z);
          }
        }
      }
    }
    if (zs_above.empty()) {
      return 0;
    }
    std::sort(zs_above.begin(), zs_above.end());
    // First point above must be contiguous to the top (within connect_step of z_top),
    // then keep counting while the run is unbroken.
    int count = 0;
    float prev = z_top;
    for (float z : zs_above) {
      if (z - prev > connect_step) {
        break;  // vertical gap → discontinuity → not a tree's crown
      }
      ++count;
      prev = z;
    }
    return count;
  }

  /**
   * Crown evidence per cluster, from the FULL non-ground cloud (the canopy of a tall
   * tree is cut by the z-crop, so this can be 0 even for a tree — it is a BONUS the
   * single-pass classifier weighs, never a veto). One contiguous count per cluster.
   */
  std::vector<float> compute_canopies(
    const std::vector<forest_3d_perception::experimental::PointCluster> & clusters,
    const CloudT::Ptr & non_ground) const
  {
    std::vector<float> out(clusters.size(), 0.0f);
    if (!canopy_enabled_ || !non_ground || non_ground->empty()) {
      return out;
    }
    const HeightGrid grid = build_height_grid(*non_ground, canopy_cell_);
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      const auto & cl = clusters[i];
      if (!cl.cloud || cl.cloud->empty()) {
        continue;
      }
      double sx = 0.0, sy = 0.0;
      float z_top = -std::numeric_limits<float>::max();
      for (const auto & p : cl.cloud->points) {
        sx += p.x; sy += p.y;
        z_top = std::max(z_top, p.z);
      }
      const double inv = 1.0 / static_cast<double>(cl.cloud->size());
      out[i] = static_cast<float>(canopy_above(
        static_cast<float>(sx * inv), static_cast<float>(sy * inv), z_top, grid,
        canopy_cell_, canopy_radius_, canopy_margin_, canopy_connect_step_));
    }
    return out;
  }

  /**
   * Gravity-alignment rotation that removes the body's roll/pitch tilt while
   * preserving yaw: R = Rx(-roll) * Ry(-pitch). A point measured in the tilted
   * base_link is brought into a level (gravity-aligned) frame by p_level = R * p.
   * The perception cloud lives in base_link (body frame); base_link tilts with
   * the robot regardless of the EKF, so trunks appear leaning. The map layer uses
   * the same IMU-derived correction (traversability_mapping_node).
   */
  static void gravity_rotation(double roll, double pitch, double R[3][3])
  {
    const double cr = std::cos(-roll), sr = std::sin(-roll);
    const double cp = std::cos(-pitch), sp = std::sin(-pitch);
    R[0][0] = cp;       R[0][1] = 0.0; R[0][2] = sp;
    R[1][0] = sr * sp;  R[1][1] = cr;  R[1][2] = -sr * cp;
    R[2][0] = -cr * sp; R[2][1] = sr;  R[2][2] = cr * cp;
  }

  /** Apply a 3x3 rotation to every point of a cloud (about the sensor origin). */
  static CloudT::Ptr rotate_cloud(const CloudT & in, const double R[3][3])
  {
    CloudT::Ptr out(new CloudT);
    out->reserve(in.size());
    for (const auto & p : in.points) {
      PointT q;
      q.x = static_cast<float>(R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z);
      q.y = static_cast<float>(R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z);
      q.z = static_cast<float>(R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z);
      out->push_back(q);
    }
    out->width = static_cast<std::uint32_t>(out->size());
    out->height = 1;
    out->is_dense = true;
    return out;
  }

  /**
   * Base-position covariance (row-major 3x3) for the SLAM Mahalanobis gate.
   *
   * NOT the raw point spread (that is the trunk THICKNESS, ~radius). The SLAM
   * needs the uncertainty of WHERE THE BASE IS, so:
   *   σ_centroid² = point_dispersion / N        (statistical centre estimate)
   *   + (range_k · range)²                       (LiDAR range-dependent noise)
   *   + σ_floor²                                 (occlusion/systematic floor)
   * The statistical part keeps its XY correlation; the range/floor terms are
   * isotropic (diagonal only). Z uses the ground-estimation uncertainty.
   * Cloud must already be in the gravity-aligned frame.
   */
  static std::array<double, 9> compute_base_covariance_3d(
    const CloudT & cloud, float z_base, float z_stem_top,
    double range, double range_k, double sigma_floor_m, double sigma_z_m)
  {
    std::array<double, 9> cov{};
    std::vector<std::pair<double, double>> pts;
    pts.reserve(cloud.size());
    for (const auto & p : cloud.points) {
      if (p.z >= z_base && p.z <= z_stem_top) {
        pts.emplace_back(p.x, p.y);
      }
    }
    if (pts.size() < 5) {
      pts.clear();
      for (const auto & p : cloud.points) {
        pts.emplace_back(p.x, p.y);
      }
    }
    const double range_var = (range_k * range) * (range_k * range);
    const double floor_var = sigma_floor_m * sigma_floor_m;
    const double iso = range_var + floor_var;  // isotropic diagonal addition
    if (pts.size() < 3) {
      cov[0] = cov[4] = 0.01 * 0.01 + iso;
      cov[8] = sigma_z_m * sigma_z_m + range_var;
      return cov;
    }
    double sx = 0.0, sy = 0.0;
    for (const auto & [x, y] : pts) { sx += x; sy += y; }
    const double n = static_cast<double>(pts.size());
    const double mx = sx / n;
    const double my = sy / n;
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const auto & [x, y] : pts) {
      const double dx = x - mx, dy = y - my;
      sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    // Population dispersion → centroid-estimate covariance: divide by N
    // (σ_centroid² = σ_points² / N), using N-1 for the unbiased dispersion.
    const double disp_inv = 1.0 / ((n - 1.0) * n);
    cov[0] = sxx * disp_inv + iso;  // σ_xx
    cov[1] = sxy * disp_inv;        // σ_xy (correlation kept, no isotropic term)
    cov[3] = sxy * disp_inv;        // σ_yx (symmetric)
    cov[4] = syy * disp_inv + iso;  // σ_yy
    cov[8] = sigma_z_m * sigma_z_m + range_var;  // σ_zz: ground est. + range
    return cov;
  }

  /** Rotate a point and its 3x3 covariance from the level frame back into
   *  base_link: p_bl = Rᵀ·p_level, Σ_bl = Rᵀ·Σ_level·R (R is the gravity rotation). */
  static void level_to_base_link(
    const double R[3][3], geometry_msgs::msg::Point & p, std::array<double, 9> & cov)
  {
    const double x = p.x, y = p.y, z = p.z;
    // Rᵀ·p (Rᵀ[i][j] = R[j][i])
    p.x = R[0][0] * x + R[1][0] * y + R[2][0] * z;
    p.y = R[0][1] * x + R[1][1] * y + R[2][1] * z;
    p.z = R[0][2] * x + R[1][2] * y + R[2][2] * z;
    // M = Rᵀ·Σ  →  Σ_bl = M·R
    double M[9] = {};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) {
          M[r * 3 + c] += R[k][r] * cov[k * 3 + c];  // Rᵀ[r][k]=R[k][r]
        }
      }
    }
    std::fill(cov.begin(), cov.end(), 0.0);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) {
          cov[r * 3 + c] += M[r * 3 + k] * R[k][c];
        }
      }
    }
  }

  /** Honest DBH 1-sigma [m]. The DOMINANT term is the ANGULAR COVERAGE of the
   *  visible arc: a partial arc makes the radius ill-conditioned, so the true DBH
   *  error reaches decimetres while the RMSE stays ~2 cm (measured offline — the
   *  RMSE is blind to this, which made the old sigma lie to the SLAM). arc_coverage
   *  is sagitta/chord ≈ tan(arc/4): ~0.38 = full circle (small sigma), ~0.14 = short
   *  arc (large sigma). Range + rmse kept as secondary terms. Calibrated to the
   *  offline arc-sweep probe (DBH err ≈ 0.015·(0.38/cov)³). This lets the map
   *  ACCUMULATE multi-view: it trusts the wide-arc frames and discounts the rest —
   *  the only way a per-frame-ill-conditioned DBH stabilises (it cannot per frame). */
  static float dbh_stddev(
    float rmse, double range, float arc_coverage, double range_k, double sigma_floor)
  {
    constexpr double kArcK = 0.015;     // sigma [m] at full coverage
    constexpr double kArcRef = 0.38;    // sagitta/chord of a fully visible arc
    constexpr double kArcFloor = 0.12;  // clamp tiny coverage (avoid blow-up)
    constexpr double kArcMax = 0.50;    // cap [m]
    const double cov = std::max(static_cast<double>(arc_coverage), kArcFloor);
    const double ratio = kArcRef / cov;
    const double sigma_arc = std::min(kArcK * ratio * ratio * ratio, kArcMax);
    return static_cast<float>(std::sqrt(
      static_cast<double>(rmse) * rmse +
      sigma_arc * sigma_arc +
      (range_k * range) * (range_k * range) +
      sigma_floor * sigma_floor));
  }

  /**
   * Layer-contract output #2: per-frame tree landmarks (TRUNK clusters only) as
   * parametric primitives — base (x,y,z), diameter, height, confidence,
   * base_covariance + diameter_stddev for the SLAM Mahalanobis gate.
   *
   * GRAVITY ALIGNMENT (split concern — fixed after the SE(3) EKF migration):
   * - DBH/height/rmse are measured on the gravity-LEVELLED cloud (cylinder fit
   *   assumes a vertical axis; a trunk leaning in tilted base_link inflates DBH).
   *   These are rotation-invariant scalars, published directly.
   * - base (x,y,z) and base_covariance are rotated BACK into base_link (level→base_link)
   *   so the published frame_id=base_link is honest. The consumer's base_link→map TF
   *   (now SE(3), carrying real roll/pitch) does the levelling — publishing already-
   *   levelled coords here would DOUBLE-correct. So gravity_align only sharpens the
   *   DBH; it never moves the published coordinates out of base_link.
   * Stateless — no tracking, no IDs (prev_trunks_ is a diagnostic-only 1-frame buffer).
   */
  void publish_tree_landmarks(
    const std::vector<forest_3d_perception::experimental::PointCluster> & clusters,
    const std::vector<forest_3d_perception::experimental::ScoredCluster> & scored,
    const rclcpp::Time & stamp,
    forest_3d_perception::experimental::PipelineFunnelStats & funnel,
    const std::vector<bool> & cluster_no_ground = {}) const
  {
    if (!pub_tree_landmarks_) {
      return;
    }
    using forest_3d_perception::CylinderObservation;
    using forest_3d_perception::CylinderReject;
    using forest_3d_perception::fit_vertical_cylinder;

    const bool use_gravity = gravity_align_ && have_imu_ &&
      (stamp - imu_stamp_).seconds() < imu_timeout_;

    double R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    if (use_gravity) {
      gravity_rotation(imu_roll_, imu_pitch_, R);
    }

    forest_hybrid_msgs::msg::TreeLandmarkArray arr;
    arr.header.stamp = stamp;
    arr.header.frame_id = processing_frame_;

    std::size_t n_clusters = clusters.size();
    std::size_t n_structural = 0;
    std::size_t n_accept_cylinder = 0;
    std::size_t n_reject_radius = 0;
    std::size_t n_reject_height = 0;
    std::size_t n_reject_verticality = 0;
    std::size_t n_reject_points = 0;
    std::size_t n_dominant_trunk = 0;
    std::size_t n_dominant_rock = 0;
    std::size_t n_dominant_obstacle = 0;

    std::vector<std::array<float, 3>> cur_trunks;
    CloudT::Ptr trunk_fit_cloud(new CloudT);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cluster_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

    for (std::size_t i = 0; i < clusters.size(); ++i) {
      if (!classifier_.is_structural_candidate(scored[i])) {
        continue;
      }
      ++n_structural;
      const auto & sc = scored[i];
      const auto & cl = clusters[i];
      if (!cl.cloud || cl.cloud->empty()) {
        continue;
      }

      const int dom = forest_3d_perception::experimental::argmax_class_index(sc.class_scores);
      if (dom == 0) {
        ++n_dominant_trunk;
      } else if (dom == 1) {
        ++n_dominant_rock;
      } else {
        ++n_dominant_obstacle;
      }

      std::vector<std::size_t> band_indices;
      CloudT::Ptr fit_cloud = use_gravity ? rotate_cloud(*cl.cloud, R) : cl.cloud;
      std::vector<std::size_t> idx(fit_cloud->size());
      std::iota(idx.begin(), idx.end(), 0u);

      CylinderObservation cyl;
      const auto reject = fit_vertical_cylinder(
        *fit_cloud, idx, cyl,
        cyl_min_height_, cyl_max_radius_, cyl_max_rmse_,
        cyl_min_inlier_, cyl_inlier_dist_, cyl_max_slice_h_,
        cyl_dbh_band_low_, cyl_dbh_band_high_, cyl_stem_grow_, cyl_stem_axis_jump_,
        &band_indices);

      // GATE ESTRITO do caminho B (sem solo). Como a copa não pode ser cortada
      // por banda relativa ao solo, o filtro de colunas verticais já removeu
      // copa/dispersão; aqui só se emite quando é GEOMETRICAMENTE um tronco limpo:
      // cilindro aceite + core vertical suficiente + DBH em gama de tronco +
      // largura contida (rejeita fusões de troncos e arcos parciais enviesados).
      // Correção acima de cobertura: o que não for tronco limpo NÃO é emitido.
      const bool is_path_b = (i < cluster_no_ground.size() && cluster_no_ground[i]);
      if (is_path_b) {
        const float dbh_pb =
          (reject == CylinderReject::Accepted) ? 2.0f * cyl.radius : -1.0f;
        const bool pb_trunk =
          reject == CylinderReject::Accepted &&
          sc.feat.trunk_core_height >= path_b_min_core_m_ &&
          dbh_pb >= path_b_dbh_min_m_ && dbh_pb <= path_b_dbh_max_m_ &&
          sc.feat.horizontal_size <= path_b_max_hsize_m_ &&
          sc.feat.verticality >= path_b_min_verticality_ &&
          sc.feat.linearity >= path_b_min_linearity_;
        if (!pb_trunk) {
          continue;
        }
      }

      forest_hybrid_msgs::msg::TreeLandmark t;
      t.class_scores = {
        sc.class_scores[0], sc.class_scores[1], sc.class_scores[2]};
      // O scorer suave foi calibrado para o caminho A (clusters com rugosidade de
      // casca + copa). Um tronco do caminho B, depois do filtro vertical, é uma
      // coluna LIMPA sem copa, que o scorer lê erradamente como ROCHA. Como já
      // passou o gate geométrico estrito acima, sabemos que é tronco: força a
      // probabilidade de tronco (senão o SLAM excluí-lo-ia do DBH multi-vista).
      if (is_path_b) {
        const float tp = 0.70f + 0.25f * cyl.inlier_ratio;  // confiança via inliers
        t.class_scores = {tp, 0.5f * (1.0f - tp), 0.5f * (1.0f - tp)};
      }
      t.semantic_class = tree_landmark_semantic_class(t.class_scores);
      const float max_score = std::max(
        {t.class_scores[0], t.class_scores[1], t.class_scores[2]});

      if (reject == CylinderReject::Accepted) {
        t.base.x = static_cast<double>(cyl.cx);
        t.base.y = static_cast<double>(cyl.cy);
        t.base.z = static_cast<double>(cyl.z_base);
        t.diameter = 2.0f * cyl.radius;
        t.height = cyl.height;
        {
          const double rng = std::sqrt(
            static_cast<double>(cyl.cx) * cyl.cx + static_cast<double>(cyl.cy) * cyl.cy);
          t.diameter_stddev = dbh_stddev(
            cyl.rmse, rng, cyl.arc_coverage, cov_range_k_, cov_sigma_floor_m_);
        }
        t.confidence = max_score * (0.4f + 0.6f * cyl.inlier_ratio);
        ++n_accept_cylinder;

        const double range = std::sqrt(t.base.x * t.base.x + t.base.y * t.base.y);
        const float dbh_for_cov_top = static_cast<float>(t.base.z) +
          static_cast<float>(cyl_max_slice_h_);
        t.base_covariance = compute_base_covariance_3d(
          *fit_cloud, static_cast<float>(t.base.z), dbh_for_cov_top,
          range, cov_range_k_, cov_sigma_floor_m_, cov_sigma_z_m_);
        // Caminho B (base não observada): infla a covariância — XY menos certa e
        // cota Z muito incerta. O SLAM pesa-os menos, mas mantém o tracking vivo
        // (evita o LOST) e corrige via re-observação. O DBH/raio não é afetado.
        if (i < cluster_no_ground.size() && cluster_no_ground[i]) {
          auto & C = t.base_covariance;
          C[0] *= path_b_cov_inflate_xy_; C[1] *= path_b_cov_inflate_xy_;
          C[3] *= path_b_cov_inflate_xy_; C[4] *= path_b_cov_inflate_xy_;
          C[2] = C[5] = C[6] = C[7] = 0.0;
          C[8] = std::max(C[8], path_b_z_var_);
        }
        if (use_gravity) {
          level_to_base_link(R, t.base, t.base_covariance);
        }
        cur_trunks.push_back({static_cast<float>(t.base.x),
                              static_cast<float>(t.base.y), t.diameter});
      } else {
        switch (reject) {
          case CylinderReject::TooWide:     ++n_reject_radius; break;
          case CylinderReject::TooShort:    ++n_reject_height; break;
          case CylinderReject::HighRmse:
          case CylinderReject::LowInliers:  ++n_reject_verticality; break;
          case CylinderReject::TooFewPoints:++n_reject_points; break;
          default: break;
        }
        fill_bbox_landmark_geometry(t, cl);
        t.confidence = max_score * 0.5f;
      }

      if (t.diameter < 0.1f || t.diameter > 3.0f) {
        continue;
      }

      // tree_clusters: emite os inliers do fit DEPOIS de o landmark passar o
      // filtro de sanidade, com o índice FINAL (arr.trees.size() neste ponto =
      // índice que o landmark terá após o push_back abaixo). Emitir dentro do
      // ramo "cilindro aceite" antes deste filtro dessincronizava o contrato
      // intensity=índice quando um cilindro aceite era descartado por diâmetro.
      if (reject == CylinderReject::Accepted) {
        const float tree_idx = static_cast<float>(arr.trees.size());
        for (std::size_t bi : band_indices) {
          if (bi < cl.cloud->size()) {
            const auto & p = cl.cloud->points[bi];
            trunk_fit_cloud->push_back(p);
            pcl::PointXYZI q;
            q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = tree_idx;
            tree_cluster_cloud->push_back(q);
          }
        }
      }

      arr.trees.push_back(t);
    }

    pub_tree_landmarks_->publish(arr);
    publish_cloud(pub_trunk_fit_points_, trunk_fit_cloud, stamp, processing_frame_);

    if (pub_tree_clusters_) {
      tree_cluster_cloud->width = static_cast<std::uint32_t>(tree_cluster_cloud->size());
      tree_cluster_cloud->height = 1;
      tree_cluster_cloud->is_dense = true;
      sensor_msgs::msg::PointCloud2 clusters_msg;
      pcl::toROSMsg(*tree_cluster_cloud, clusters_msg);
      clusters_msg.header.stamp = stamp;
      clusters_msg.header.frame_id = processing_frame_;
      pub_tree_clusters_->publish(clusters_msg);
    }

    float dbh_stability_pct = -1.0f;
    {
      std::vector<float> rel_changes;
      for (const auto & cur : cur_trunks) {
        float best_d2 = kStabilityMatchR2;
        float best_prev_dbh = -1.0f;
        for (const auto & prev : prev_trunks_) {
          const float dx = cur[0] - prev[0];
          const float dy = cur[1] - prev[1];
          const float d2 = dx * dx + dy * dy;
          if (d2 < best_d2) { best_d2 = d2; best_prev_dbh = prev[2]; }
        }
        if (best_prev_dbh > 1e-3f) {
          rel_changes.push_back(std::abs(cur[2] - best_prev_dbh) / best_prev_dbh);
        }
      }
      if (!rel_changes.empty()) {
        std::sort(rel_changes.begin(), rel_changes.end());
        dbh_stability_pct = 100.0f * rel_changes[rel_changes.size() / 2];
      }
    }
    prev_trunks_ = std::move(cur_trunks);

    if (pub_tree_landmark_markers_) {
      visualization_msgs::msg::MarkerArray mkarr;
      visualization_msgs::msg::Marker clr;
      clr.header.stamp = stamp;
      clr.header.frame_id = processing_frame_;
      clr.action = visualization_msgs::msg::Marker::DELETEALL;
      mkarr.markers.push_back(clr);

      int mid = 0;
      for (const auto & t : arr.trees) {
        float mr = 0.6f;
        float mg = 0.6f;
        float mb = 0.6f;
        const int dom = forest_3d_perception::experimental::argmax_class_index(
          {t.class_scores[0], t.class_scores[1], t.class_scores[2]});
        if (dom == 0) {
          mr = 0.55f; mg = 0.35f; mb = 0.10f;
        } else if (dom == 1) {
          mr = mg = mb = 0.55f;
        } else {
          mr = 1.0f; mg = 0.55f; mb = 0.0f;
        }

        visualization_msgs::msg::Marker cyl;
        cyl.header.stamp = stamp;
        cyl.header.frame_id = processing_frame_;
        cyl.ns = "tree_landmarks_cyl";
        cyl.id = mid++;
        cyl.type = visualization_msgs::msg::Marker::CYLINDER;
        cyl.action = visualization_msgs::msg::Marker::ADD;
        cyl.pose.position.x = t.base.x;
        cyl.pose.position.y = t.base.y;
        cyl.pose.position.z = t.base.z + t.height * 0.5;
        cyl.pose.orientation.w = 1.0;
        cyl.scale.x = static_cast<double>(t.diameter);
        cyl.scale.y = static_cast<double>(t.diameter);
        cyl.scale.z = static_cast<double>(t.height);
        cyl.color.r = mr;
        cyl.color.g = mg;
        cyl.color.b = mb;
        cyl.color.a = 0.55f * t.confidence;
        cyl.lifetime = rclcpp::Duration::from_seconds(0.6);
        mkarr.markers.push_back(cyl);

        visualization_msgs::msg::Marker txt;
        txt.header = cyl.header;
        txt.ns = "tree_landmarks_txt";
        txt.id = mid++;
        txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::msg::Marker::ADD;
        txt.pose.position.x = t.base.x;
        txt.pose.position.y = t.base.y;
        txt.pose.position.z = t.base.z + t.height + 0.3;
        txt.scale.z = 0.20;
        txt.color.r = mr;
        txt.color.g = mg;
        txt.color.b = mb;
        txt.color.a = 1.0f;
        char buf[64];
        std::snprintf(
          buf, sizeof(buf), "T%.0f R%.0f O%.0f D%.0fcm",
          t.class_scores[0] * 100.0f, t.class_scores[1] * 100.0f,
          t.class_scores[2] * 100.0f, t.diameter * 100.0f);
        txt.text = buf;
        txt.lifetime = rclcpp::Duration::from_seconds(0.6);
        mkarr.markers.push_back(txt);
      }
      pub_tree_landmark_markers_->publish(mkarr);
    }

    if (debug_log_interval_ > 0 && frame_count_ % static_cast<std::size_t>(debug_log_interval_) == 0) {
      char stab[24];
      if (dbh_stability_pct >= 0.0f) {
        std::snprintf(stab, sizeof(stab), "%.1f%%", dbh_stability_pct);
      } else {
        std::snprintf(stab, sizeof(stab), "n/a");
      }
      RCLCPP_INFO(
        get_logger(),
        "LANDMARKS clusters=%zu structural=%zu emitted=%zu dom[T=%zu R=%zu O=%zu] "
        "cyl_accept=%zu reject[r=%zu h=%zu v=%zu pts=%zu] gravity=%s dbh_stab=%s",
        n_clusters, n_structural, arr.trees.size(),
        n_dominant_trunk, n_dominant_rock, n_dominant_obstacle,
        n_accept_cylinder, n_reject_radius, n_reject_height, n_reject_verticality,
        n_reject_points, use_gravity ? "on" : "off", stab);
    }

    funnel.n_structural_candidates = n_structural;
    funnel.n_landmarks_emitted = arr.trees.size();
    funnel.n_dominant_trunk = n_dominant_trunk;
    funnel.n_dominant_rock = n_dominant_rock;
    funnel.n_dominant_obstacle = n_dominant_obstacle;
    funnel.n_trunk_classified = n_structural;
    funnel.n_trunk_accept = n_accept_cylinder;
    funnel.n_trunk_reject_radius = n_reject_radius;
    funnel.n_trunk_reject_height = n_reject_height;
    funnel.n_trunk_reject_verticality = n_reject_verticality;
    funnel.n_trunk_reject_points = n_reject_points;
    funnel.dbh_stability_pct = dbh_stability_pct;
    funnel.gravity_aligned = use_gravity;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    using PS = forest_3d_perception::experimental::PipelineExitStatus;
    forest_3d_perception::experimental::PipelineFunnelStats funnel;
    funnel.debug_stage = debug_stage_;
    funnel.pipeline_sprint = 1;
    funnel.n_input_msg = 1;
    funnel.input_frame = msg->header.frame_id;
    funnel.processing_frame = processing_frame_;
    n_band_last_ = 0;
    n_recovered_last_ = 0;

    const rclcpp::Time stamp(msg->header.stamp);
    const auto stage_mode = forest_3d_perception::experimental::debug_stage_from_int(debug_stage_);

    if (!enabled_) {
      funnel.status = PS::Disabled;
      publish_funnel(funnel);
      log_funnel_throttled(funnel);
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_tf;
    try {
      auto tf = tf_buffer_.lookupTransform(
        processing_frame_, msg->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      tf2::doTransform(*msg, cloud_tf, tf);
    } catch (const tf2::TransformException & ex) {
      funnel.status = PS::TfFail;
      publish_funnel(funnel);
      log_funnel_throttled(funnel);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s -> %s FAILED: %s (no outputs published)",
        msg->header.frame_id.c_str(), processing_frame_.c_str(), ex.what());
      return;
    }
    cloud_tf.header.frame_id = processing_frame_;

    CloudT::Ptr raw(new CloudT);
    pcl::fromROSMsg(cloud_tf, *raw);
    funnel.n_raw = raw->size();
    if (raw->empty()) {
      funnel.status = PS::RawEmpty;
      publish_funnel(funnel);
      log_funnel_throttled(funnel);
      return;
    }

    CloudT::Ptr cropped(new CloudT);
    crop_cloud(raw, cropped);
    funnel.n_crop = cropped->size();
    funnel.n_non_finite = count_non_finite(*raw);
    forest_3d_perception::experimental::update_crop_z_bounds(*cropped, funnel);
    if (cropped->size() < static_cast<std::size_t>(crop_min_points_)) {
      funnel.status = PS::CropTooFew;
      publish_funnel(funnel);
      log_funnel_throttled(funnel);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "crop_too_few: %zu pts (min %d) — check min_range/max_range/min_z/max_z",
        cropped->size(), crop_min_points_);
      return;
    }

    CloudT::Ptr voxel(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cropped);
    vg.setLeafSize(
      static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_),
      static_cast<float>(voxel_leaf_));
    vg.filter(*voxel);
    funnel.n_voxel = voxel->size();
    if (voxel->empty()) {
      funnel.status = PS::VoxelEmpty;
      publish_funnel(funnel);
      log_funnel_throttled(funnel);
      return;
    }

    if (debug_publish_stages_) {
      publish_cloud(pub_dbg_voxel_, voxel, stamp, processing_frame_);
    }

    /* Stage 1: voxel only — publish voxel as visible output, skip CSF/cluster */
    if (stage_mode == forest_3d_perception::experimental::DebugStageMode::VoxelOnly) {
      publish_cloud(pub_ground_, voxel, stamp, processing_frame_);
      publish_cloud(pub_non_ground_, CloudT::Ptr(new CloudT), stamp, processing_frame_);
      funnel.status = PS::Ok;
      funnel.n_ground = voxel->size();
      publish_funnel(funnel);
      log_funnel_throttled(funnel);
      return;
    }

    CloudT::Ptr ground(new CloudT);
    CloudT::Ptr non_ground(new CloudT);
    std::vector<forest_3d_perception::experimental::PointCluster> clusters;
    CloudT::Ptr band_cloud(new CloudT);
    CloudT::Ptr stem_candidates(new CloudT);
    // Cloud that the clusters' point_indices reference (for the labeled cloud).
    CloudT::Ptr cluster_ref_cloud;

    if (stage_mode == forest_3d_perception::experimental::DebugStageMode::ClusterOnly) {
      /* Stage 3: clustering only — feed full voxel as non-ground, Euclidean 3D (CSF bypass) */
      non_ground = voxel;
      funnel.n_non_ground = non_ground->size();
      auto r = sprint1_.clusterer.cluster(*non_ground);
      clusters = std::move(r.clusters);
      cluster_ref_cloud = non_ground;
    } else {
      /* Stage 2 or full: CSF */
      const auto csf_out = sprint1_.csf_segmenter.segment(*voxel);
      ground = csf_out.ground;
      non_ground = csf_out.non_ground;
      // Post-pass: recover fallen rocks that CSF absorbed as ground.
      if (object_filter_.params.enabled && ground && !ground->empty()) {
        CloudT::Ptr refined(new CloudT);
        CloudT::Ptr rocks(new CloudT);
        object_filter_.refine(*ground, *refined, *rocks);
        ground = refined;
        if (non_ground) { *non_ground += *rocks; }
      }
      // Columnar recovery: pull stolen object bases (trunk/rock feet) from the
      // ground where the XY column has a real object above the terrain floor.
      if (columnar_recovery_.params.enabled && ground && !ground->empty() && non_ground) {
        CloudT::Ptr refined(new CloudT);
        CloudT::Ptr recovered(new CloudT);
        const auto st = columnar_recovery_.refine(*ground, *non_ground, *refined, *recovered);
        ground = refined;
        *non_ground += *recovered;
        n_recovered_last_ = st.n_recovered;
      }
      funnel.n_ground = ground ? ground->size() : 0;
      funnel.n_non_ground = non_ground ? non_ground->size() : 0;
      if (funnel.n_voxel > 0) {
        funnel.csf_ground_pct =
          100.0 * static_cast<double>(funnel.n_ground) / static_cast<double>(funnel.n_voxel);
      }

      if (debug_publish_stages_) {
        publish_cloud(pub_dbg_ground_, ground, stamp, processing_frame_);
        publish_cloud(pub_dbg_non_ground_, non_ground, stamp, processing_frame_);
      }

      if (stage_mode == forest_3d_perception::experimental::DebugStageMode::CsfOnly) {
        publish_cloud(pub_ground_, ground, stamp, processing_frame_);
        publish_cloud(pub_non_ground_, non_ground, stamp, processing_frame_);
        funnel.status = PS::Ok;
        publish_funnel(funnel);
        log_funnel_throttled(funnel);
        return;
      }

      if (ground && non_ground && !non_ground->empty()) {
        if (region_grower_.params.enabled) {
          /* Sprint 3.5 — Option 2 (default): ground-seeded vertical region
             growing. Every cluster is grown from a base seed up to
             growth_max_hag, so all clusters are ground-anchored (rocks/trunks
             connect to the ground; floating canopy excluded) and trees are NOT
             fragmented. */
          auto rg = region_grower_.grow(*ground, *non_ground);
          band_cloud = rg.working_cloud;     // candidates (HAG in [~0, max])
          stem_candidates = rg.grown_cloud;  // points that grew into a region
          n_band_last_ = rg.n_working;
          funnel.n_working = rg.n_working;
          funnel.n_seeds = rg.n_seeds;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "REGION_GROW working=%zu seeds=%zu grown=%zu clusters=%zu | "
            "started=%zu discard_small=%zu(pts=%zu,max=%zu) discard_large=%zu "
            "(seed_max_hag=%.2f radius=%.2f z_scale=%.2f min_pts=%d)",
            rg.n_working, rg.n_seeds, rg.n_grown, rg.clusters.size(),
            rg.n_regions_started, rg.n_discarded_small, rg.pts_in_discarded_small,
            rg.largest_discarded_small, rg.n_discarded_large,
            region_grower_.params.seed_max_hag_m, region_grower_.params.growth_radius_m,
            region_grower_.params.growth_z_scale, region_grower_.params.min_region_pts);
          clusters = std::move(rg.clusters);
        }
      }
      // Cluster indices reference the non-ground cloud.
      cluster_ref_cloud = non_ground;
    }

    // --- Caminho B: troncos SEM solo (HAG=NaN), que o caminho A largou ---------
    // Os pontos não-solo sem chão por baixo (atrás/ao lado do robô) são agrupados
    // e UNIDOS aos clusters do caminho A. Passam pelo MESMO classificador (que já
    // filtra lixo por forma) e fit; só mudam na emissão: covariância inflada
    // (base não observada). A máscara marca quais vieram do caminho B.
    std::vector<bool> cluster_no_ground(clusters.size(), false);
    if (path_b_enabled_ && ground && non_ground && !non_ground->empty()) {
      const auto ng = stem_clusterer_.extract_no_ground(*ground, *non_ground);
      // Filtro de colunas verticais: sem solo não há corte de banda, logo a copa
      // entraria e faria ponte 2D entre árvores (blob gigante). Este filtro
      // geométrico mantém só superfície de fuste (vertical/linear) ANTES do
      // clustering, eliminando a contaminação por copa.
      const auto ng_cols = stem_clusterer_.filter_vertical_columns(*non_ground, ng.indices);
      if (!ng_cols.empty()) {
        auto cb = stem_clusterer_.cluster_band_subset(
          *non_ground, ng_cols, static_cast<int>(clusters.size()));
        for (auto & c : cb) {
          // Backstop ao nível do cluster: o filtro de colunas verticais ainda
          // esculpe "agulhas" verticais curtas de fragmentos de copa (meridianos
          // da folhagem). Um fuste é uma estrutura vertical ALTA; um fragmento de
          // copa é curto. Exige extensão vertical mínima JÁ NA FORMAÇÃO (não só
          // na emissão) para que estes nem cheguem a virar cluster/marcador.
          if (!c.cloud || c.cloud->empty()) {
            continue;
          }
          float zlo = c.cloud->points.front().z;
          float zhi = zlo;
          for (const auto & p : c.cloud->points) {
            zlo = std::min(zlo, p.z);
            zhi = std::max(zhi, p.z);
          }
          if ((zhi - zlo) < static_cast<float>(path_b_min_zspan_m_)) {
            continue;
          }
          clusters.push_back(std::move(c));
          cluster_no_ground.push_back(true);
        }
      }
    }

    funnel.n_clusters = clusters.size();
    funnel.cluster_summaries =
      forest_3d_perception::experimental::summarize_clusters(clusters);

    // Single-pass classification: compute crown evidence per cluster, then classify
    // all criteria at once (vertical/PCA + smoothness + crown-as-bonus). No later
    // re-classification.
    const std::vector<float> canopies = compute_canopies(clusters, non_ground);
    const auto scored = classifier_.score_all(clusters, canopies);

    if (debug_log_interval_ > 0 && !scored.empty()) {
      std::size_t n_dom_trunk = 0;
      std::size_t n_dom_rock = 0;
      std::size_t n_dom_obstacle = 0;
      std::size_t n_structural = 0;
      for (const auto & s : scored) {
        if (!classifier_.is_structural_candidate(s)) {
          continue;
        }
        ++n_structural;
        const int dom = forest_3d_perception::experimental::argmax_class_index(s.class_scores);
        if (dom == 0) {
          ++n_dom_trunk;
        } else if (dom == 1) {
          ++n_dom_rock;
        } else {
          ++n_dom_obstacle;
        }
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SCORE n_clusters=%zu structural=%zu dom[T=%zu R=%zu O=%zu]",
        scored.size(), n_structural, n_dom_trunk, n_dom_rock, n_dom_obstacle);
      const std::size_t dump = std::min<std::size_t>(scored.size(), 6);
      for (std::size_t i = 0; i < dump; ++i) {
        const auto & f = scored[i].feat;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "  cl[%zu] T=%.2f R=%.2f O=%.2f n=%d h=%.2f vert=%.2f lin=%.2f surf=%.3f",
          i, scored[i].class_scores[0], scored[i].class_scores[1],
          scored[i].class_scores[2], f.n_points, f.height_span, f.verticality,
          f.linearity, f.surface_variation);
      }
    }

    publish_cloud(pub_ground_, ground, stamp, processing_frame_);
    publish_cloud(pub_non_ground_, non_ground, stamp, processing_frame_);
    publish_cloud(pub_stem_candidates_, stem_candidates, stamp, processing_frame_);
    publish_cloud(pub_stem_band_, band_cloud, stamp, processing_frame_);
    publish_labeled_clusters(
      cluster_ref_cloud ? cluster_ref_cloud : non_ground, clusters, scored, stamp);

    publish_semantic_points(
      ground, cluster_ref_cloud ? cluster_ref_cloud : non_ground, clusters, scored, stamp);
    publish_tree_landmarks(clusters, scored, stamp, funnel, cluster_no_ground);

    funnel.status = PS::Ok;
    publish_funnel(funnel);
    log_funnel_throttled(funnel);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  forest_3d_perception::experimental::Sprint1Pipeline sprint1_;
  forest_3d_perception::experimental::StemBandClusterer stem_clusterer_;
  forest_3d_perception::experimental::ObjectOnGroundFilter object_filter_;
  forest_3d_perception::experimental::ColumnarGroundRecovery columnar_recovery_;
  mutable std::size_t n_recovered_last_{0};
  forest_3d_perception::experimental::StemRegionGrower region_grower_;
  forest_3d_perception::experimental::ClusterClassifier classifier_;

  std::string input_topic_;
  std::string processing_frame_;
  std::string ground_topic_;
  std::string non_ground_topic_;
  std::string stem_band_topic_;
  std::string stem_candidates_topic_;
  std::string clusters_topic_;
  std::string cluster_markers_topic_;
  std::string debug_stats_topic_;
  std::string semantic_points_topic_;
  std::string tree_landmarks_topic_;
  std::string tree_landmark_markers_topic_;
  std::string trunk_fit_points_topic_;
  std::string tree_clusters_topic_;
  std::string dbg_voxel_topic_;
  std::string dbg_ground_topic_;
  std::string dbg_non_ground_topic_;
  std::string dbg_clusters_topic_;
  double tf_timeout_{0.5};
  double voxel_leaf_{0.08};
  double min_range_{0.3};
  double max_range_{15.0};
  double min_z_{-1.0};
  double max_z_{5.0};
  int crop_min_points_{30};
  bool enabled_{true};
  int debug_stage_{0};
  bool debug_publish_stages_{true};
  int debug_log_interval_{30};
  mutable std::size_t frame_count_{0};
  mutable std::size_t n_band_last_{0};

  // IMU for gravity alignment (roll/pitch correction before landmark publish).
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  mutable bool have_imu_{false};
  mutable double imu_roll_{0.0};
  mutable double imu_pitch_{0.0};
  mutable rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
  std::string imu_topic_;
  bool gravity_align_{true};
  double imu_timeout_{0.5};
  // Cylinder fit parameters (read once at startup; not hot-reloadable).
  double cyl_min_height_{0.30};
  double cyl_max_radius_{0.80};
  double cyl_max_rmse_{0.05};
  double cyl_min_inlier_{0.30};
  double cyl_inlier_dist_{0.04};
  double cyl_max_slice_h_{2.50};
  double cyl_dbh_band_low_{0.3};
  double cyl_dbh_band_high_{2.5};
  double cyl_stem_grow_{1.8};
  double cyl_stem_axis_jump_{0.20};
  // Canopy check parameters.
  bool canopy_enabled_{true};
  float canopy_cell_{0.30f};
  float canopy_radius_{1.20f};
  float canopy_margin_{0.30f};
  float canopy_connect_step_{0.50f};
  // Base-covariance model parameters.
  double cov_range_k_{0.01};
  double cov_sigma_floor_m_{0.02};
  double cov_sigma_z_m_{0.05};
  // Caminho B (deteção sem solo).
  bool path_b_enabled_{true};
  double path_b_cov_inflate_xy_{4.0};
  double path_b_z_var_{1.0};
  double path_b_min_core_m_{1.50};
  double path_b_dbh_min_m_{0.05};
  double path_b_dbh_max_m_{0.60};
  double path_b_max_hsize_m_{0.80};
  double path_b_min_verticality_{0.70};
  double path_b_min_linearity_{0.60};
  double path_b_min_zspan_m_{1.50};
  // Inter-frame DBH-stability diagnostic (last frame's trunks: x, y, dbh).
  static constexpr float kStabilityMatchR2 = 0.30f * 0.30f;
  mutable std::vector<std::array<float, 3>> prev_trunks_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_stem_band_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_stem_candidates_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_semantic_points_;
  rclcpp::Publisher<forest_hybrid_msgs::msg::TreeLandmarkArray>::SharedPtr pub_tree_landmarks_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tree_landmark_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_trunk_fit_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tree_clusters_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dbg_voxel_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dbg_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dbg_non_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dbg_clusters_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar3dExperimentalNode>());
  rclcpp::shutdown();
  return 0;
}

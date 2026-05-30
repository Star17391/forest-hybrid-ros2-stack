/**
 * @file lidar3d_segmentation_node.cpp
 * @brief Fase 1: 3D LiDAR segmentation — 2.5D ground grid, tree trunks, obstacles.
 *
 * Pipeline:
 *   raw cloud (laser frame)
 *   → TF to base_link
 *   → voxel downsample
 *   → range/height crop
 *   → ground: 2.5D height grid (default) or RANSAC plane (legacy)
 *   → Euclidean clustering of non-ground
 *   → trunk classification (vertical extent + radius heuristic)
 *   → publish: ground, trunks, obstacles, trunk markers
 *
 * Designed for real-time on tracked forest robot (~5 Hz LiDAR, <120k points).
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "forest_3d_perception/cylinder_fit.hpp"
#include "forest_3d_perception/ground_connectivity.hpp"
#include "forest_3d_perception/ground_reference_debug.hpp"
#include "forest_3d_perception/landmark_tracker.hpp"
#include "forest_3d_perception/ndsm_field.hpp"
#include "forest_3d_perception/terrain_grid_2d.hpp"
#include "forest_3d_perception/trunk_column_extractor.hpp"
#include "forest_3d_perception/trunk_detector_euclidean.hpp"
#include "forest_3d_perception/trunk_slice_detector.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

namespace
{

struct TrunkCandidate {
  Eigen::Vector3f centroid;
  float height;
  float radius;
  float verticality;
  float z_base{0.0f};
  float cylinder_rmse{0.0f};
  int track_id{-1};
  float confidence{0.0f};
};

enum class TrunkRejectReason {
  Accepted,
  TooFewPoints,
  TooShort,
  TooWide,
  NotVertical,
};

struct ConnectivityDebug
{
  bool enabled{false};
  std::size_t n_ground_raw{0};
  std::size_t n_ground_connected{0};
  std::size_t n_suspended{0};
  double gcr_pct{0.0};
  double suspended_pct{0.0};
  double mean_abs_dz_connected{0.0};
  std::size_t n_connected_cells{0};
};

}  // namespace

class Lidar3dSegmentationNode : public rclcpp::Node
{
public:
  Lidar3dSegmentationNode()
  : Node("lidar3d_segmentation_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/sensors/lidar/points");
    processing_frame_ = declare_parameter<std::string>("processing_frame", "marble_hd2/base_link");

    voxel_leaf_ = declare_parameter<double>("voxel_leaf_size_m", 0.08);
    trunk_voxel_leaf_m_ = declare_parameter<double>("trunk_voxel_leaf_size_m", 0.0);
    min_range_ = declare_parameter<double>("min_range_m", 0.3);
    max_range_ = declare_parameter<double>("max_range_m", 15.0);
    min_z_ = declare_parameter<double>("min_z_m", -1.0);
    max_z_ = declare_parameter<double>("max_z_m", 5.0);

    ground_dist_ = declare_parameter<double>("ground_ransac_distance_m", 0.08);
    ground_max_iter_ = declare_parameter<int>("ground_ransac_max_iterations", 100);
    ground_max_slope_ = declare_parameter<double>("ground_max_slope_deg", 15.0);
    ground_z_band_ = declare_parameter<double>("ground_z_band_m", 0.30);

    ground_method_ = declare_parameter<std::string>("ground_method", "grid");
    grid_size_x_m_ = declare_parameter<double>("grid_size_x_m", 30.0);
    grid_size_y_m_ = declare_parameter<double>("grid_size_y_m", 30.0);
    grid_resolution_m_ = declare_parameter<double>("grid_resolution_m", 0.25);
    grid_ground_height_thresh_m_ = declare_parameter<double>("grid_ground_height_thresh_m", 0.12);
    grid_hole_depth_m_ = declare_parameter<double>("grid_hole_depth_m", 0.15);
    grid_inpaint_passes_ = declare_parameter<int>("grid_inpaint_passes", 4);
    grid_height_percentile_ = declare_parameter<double>("grid_height_percentile", 0.10);
    grid_smooth_max_step_m_ = declare_parameter<double>("grid_smooth_max_step_m", 0.40);
    grid_smooth_clamp_passes_ = declare_parameter<int>("grid_smooth_clamp_passes", 2);
    grid_smooth_median_radius_cells_ = declare_parameter<int>("grid_smooth_median_radius_cells", 1);
    grid_ground_neighbor_cells_ = declare_parameter<int>("grid_ground_neighbor_cells", 2);
    terrain_mesh_topic_ =
      declare_parameter<std::string>("terrain_mesh_topic", "/perception/lidar3d/terrain_mesh");
    publish_terrain_mesh_ = declare_parameter<bool>("publish_terrain_mesh", true);

    ground_connectivity_enable_ = declare_parameter<bool>("ground_connectivity_enable", true);
    connectivity_max_step_m_ = declare_parameter<double>("ground_connectivity_max_step_m", 0.15);
    connectivity_seed_radius_m_ = declare_parameter<double>("ground_connectivity_seed_radius_m", 2.0);
    publish_suspended_debug_ = declare_parameter<bool>("publish_ground_suspended_debug", true);
    suspended_topic_ =
      declare_parameter<std::string>("ground_suspended_topic", "/perception/lidar3d/ground_suspended");

    cluster_tol_ = declare_parameter<double>("trunk_cluster_tolerance_m", 0.25);
    min_cluster_ = declare_parameter<int>("trunk_min_cluster_size", 15);
    max_cluster_ = declare_parameter<int>("trunk_max_cluster_size", 2000);
    trunk_min_height_ = declare_parameter<double>("trunk_min_height_m", 0.55);
    trunk_max_radius_ = declare_parameter<double>("trunk_max_radius_m", 0.65);
    trunk_min_vert_ = declare_parameter<double>("trunk_min_verticality", 0.65);

    trunk_method_ = declare_parameter<std::string>("trunk_method", "cluster");
    ndsm_trunk_min_m_ = declare_parameter<double>("ndsm_trunk_min_m", 0.35);
    ndsm_trunk_max_m_ = declare_parameter<double>("ndsm_trunk_max_m", 4.0);
    column_min_points_per_cell_ = declare_parameter<int>("column_min_points_per_cell", 2);
    column_min_cells_ = declare_parameter<int>("column_min_cells_per_column", 2);
    column_min_points_ = declare_parameter<int>("column_min_points_per_column", 10);
    column_max_points_ = declare_parameter<int>("column_max_points_per_column", 600);
    column_max_columns_ = declare_parameter<int>("column_max_columns_per_frame", 20);
    cylinder_min_height_m_ = declare_parameter<double>("cylinder_min_height_m", 0.45);
    cylinder_max_radius_m_ = declare_parameter<double>("cylinder_max_radius_m", 0.65);
    cylinder_max_rmse_m_ = declare_parameter<double>("cylinder_max_rmse_m", 0.14);
    cylinder_min_inlier_ratio_ = declare_parameter<double>("cylinder_min_inlier_ratio", 0.45);
    cylinder_inlier_dist_m_ = declare_parameter<double>("cylinder_inlier_dist_m", 0.10);
    cylinder_max_slice_height_m_ =
      declare_parameter<double>("cylinder_max_slice_height_m", 2.5);

    slice_height_m_ = declare_parameter<double>("slice_height_m", 0.18);
    slice_max_count_ = declare_parameter<int>("slice_max_count", 48);
    slice_cluster_cell_m_ = declare_parameter<double>("slice_cluster_cell_m", 0.14);
    slice_min_points_per_cluster_ = declare_parameter<int>("slice_min_points_per_cluster", 4);
    slice_min_slices_for_trunk_ = declare_parameter<int>("slice_min_slices_for_trunk", 4);
    slice_assoc_max_xy_m_ = declare_parameter<double>("slice_assoc_max_xy_m", 0.55);
    slice_assoc_max_gap_slices_ = declare_parameter<int>("slice_assoc_max_gap_slices", 2);
    slice_min_continuity_score_ = declare_parameter<double>("slice_min_continuity_score", 0.52);
    slice_min_vertical_persistence_ =
      declare_parameter<double>("slice_min_vertical_persistence", 0.55);
    slice_max_centroid_drift_ = declare_parameter<double>("slice_max_centroid_drift", 1.2);
    slice_max_radius_cv_ = declare_parameter<double>("slice_max_radius_cv", 0.35);
    slice_ground_anchor_max_dz_m_ =
      declare_parameter<double>("slice_ground_anchor_max_dz_m", 0.55);
    slice_max_stems_per_frame_ = declare_parameter<int>("slice_max_stems_per_frame", 16);
    landmark_assoc_xy_m_ = declare_parameter<double>("landmark_assoc_max_xy_m", 0.85);
    landmark_max_misses_ = declare_parameter<int>("landmark_max_misses", 6);
    publish_ndsm_debug_ = declare_parameter<bool>("publish_ndsm_debug", false);
    publish_slice_pipeline_debug_ =
      declare_parameter<bool>("publish_slice_pipeline_debug", false);
    ndsm_debug_topic_ =
      declare_parameter<std::string>("ndsm_debug_topic", "/perception/lidar3d/ndsm_debug");
    slice_debug_ndsm_topic_ = declare_parameter<std::string>(
      "slice_debug_ndsm_topic", "/perception/lidar3d/debug/slice_ndsm_band");
    slice_debug_clusters_topic_ = declare_parameter<std::string>(
      "slice_debug_clusters_topic", "/perception/lidar3d/debug/slice_clusters_2d");
    slice_debug_rejected_topic_ = declare_parameter<std::string>(
      "slice_debug_rejected_topic", "/perception/lidar3d/debug/slice_rejected");
    slice_debug_accepted_topic_ = declare_parameter<std::string>(
      "slice_debug_accepted_topic", "/perception/lidar3d/debug/slice_accepted");
    tree_landmarks_topic_ =
      declare_parameter<std::string>("tree_landmarks_topic", "/perception/lidar3d/tree_landmarks");

    ground_topic_ = declare_parameter<std::string>("ground_topic", "/perception/lidar3d/ground");
    trunks_topic_ = declare_parameter<std::string>("trunks_topic", "/perception/lidar3d/trunks");
    obstacles_topic_ = declare_parameter<std::string>("obstacles_topic", "/perception/lidar3d/obstacles");
    markers_topic_ = declare_parameter<std::string>("markers_topic", "/perception/lidar3d/trunk_markers");
    tf_timeout_ = declare_parameter<double>("tf_timeout_sec", 0.1);
    publish_debug_stats_ = declare_parameter<bool>("publish_debug_stats", true);
    debug_stats_topic_ =
      declare_parameter<std::string>("debug_stats_topic", "/perception/lidar3d/debug_stats");
    publish_ground_ref_debug_ = declare_parameter<bool>("publish_ground_ref_debug", false);
    ground_ref_h_topic_ = declare_parameter<std::string>(
      "ground_ref_h_topic", "/perception/lidar3d/debug/ground_ref_h");
    ground_ref_zg_topic_ = declare_parameter<std::string>(
      "ground_ref_zg_topic", "/perception/lidar3d/debug/ground_ref_zg");
    ground_ref_no_obs_cells_topic_ = declare_parameter<std::string>(
      "ground_ref_no_obs_cells_topic", "/perception/lidar3d/debug/ground_ref_no_obs_cells");
    ground_ref_confidence_cells_topic_ = declare_parameter<std::string>(
      "ground_ref_confidence_cells_topic",
      "/perception/lidar3d/debug/ground_ref_confidence_cells");
    always_publish_clouds_ = declare_parameter<bool>("always_publish_segmentation_clouds", true);
    pipeline_log_interval_ = declare_parameter<int>("pipeline_log_interval_frames", 25);

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&Lidar3dSegmentationNode::on_cloud, this, std::placeholders::_1));

    pub_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, qos);
    if (publish_suspended_debug_) {
      pub_suspended_ = create_publisher<sensor_msgs::msg::PointCloud2>(suspended_topic_, qos);
    }
    pub_trunks_ = create_publisher<sensor_msgs::msg::PointCloud2>(trunks_topic_, qos);
    pub_obstacles_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacles_topic_, qos);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);
    pub_tree_landmarks_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(tree_landmarks_topic_, 10);
    if (publish_ndsm_debug_) {
      pub_ndsm_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ndsm_debug_topic_, qos);
    }
    if (publish_slice_pipeline_debug_) {
      pub_slice_ndsm_ = create_publisher<sensor_msgs::msg::PointCloud2>(slice_debug_ndsm_topic_, qos);
      pub_slice_clusters_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(slice_debug_clusters_topic_, qos);
      pub_slice_rejected_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(slice_debug_rejected_topic_, qos);
      pub_slice_accepted_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(slice_debug_accepted_topic_, qos);
    }
    if (publish_terrain_mesh_) {
      pub_terrain_mesh_ = create_publisher<visualization_msgs::msg::Marker>(terrain_mesh_topic_, 10);
    }
    if (publish_debug_stats_) {
      pub_debug_ = create_publisher<std_msgs::msg::String>(debug_stats_topic_, 10);
    }
    if (publish_ground_ref_debug_) {
      pub_ground_ref_h_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(ground_ref_h_topic_, qos);
      pub_ground_ref_zg_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(ground_ref_zg_topic_, qos);
      pub_ground_ref_no_obs_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_ref_no_obs_cells_topic_, qos);
      pub_ground_ref_confidence_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_ref_confidence_cells_topic_, qos);
    }

    apply_runtime_configuration();
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&Lidar3dSegmentationNode::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "3D segmentation [%s]: %s → ground/trunks/obstacles (frame: %s, voxel: %.2f m, range: %.1f–%.1f m)",
      ground_method_.c_str(), input_topic_.c_str(), processing_frame_.c_str(),
      voxel_leaf_, min_range_, max_range_);
    if (ground_method_ == "grid") {
      RCLCPP_INFO(
        get_logger(),
        "  terrain grid %.0fx%.0f m @ %.2f m/cell, p%.0f%% + clamp %.2fm, ground_thresh=%.2f m",
        grid_size_x_m_, grid_size_y_m_, grid_resolution_m_,
        grid_height_percentile_ * 100.0, grid_smooth_max_step_m_, grid_ground_height_thresh_m_);
      if (ground_connectivity_enable_) {
        RCLCPP_INFO(
          get_logger(),
          "  ground connectivity: max_step=%.2f m seed_r=%.1f m",
          connectivity_max_step_m_, connectivity_seed_radius_m_);
      }
    }
    if (trunk_method_ == "slice") {
      RCLCPP_INFO(
        get_logger(),
        "  trunk pipeline: euclidean+nDSM [%.2f–%.2f m] tol=%.2f min_pts=%d → PCA → cylinder → tracker",
        ndsm_trunk_min_m_, ndsm_trunk_max_m_, cluster_tol_, slice_min_points_per_cluster_);
    } else if (trunk_method_ == "column") {
      RCLCPP_INFO(
        get_logger(),
        "  trunk pipeline: column+nDSM [%.2f–%.2f m] → cylinder → tracker",
        ndsm_trunk_min_m_, ndsm_trunk_max_m_);
    } else {
      RCLCPP_INFO(get_logger(), "  trunk pipeline: legacy 3D cluster+bbox");
    }
  }

private:
  void apply_runtime_configuration()
  {
    terrain_grid_.configure(grid_size_x_m_, grid_size_y_m_, grid_resolution_m_);
    terrain_grid_.ground_height_thresh_m = static_cast<float>(grid_ground_height_thresh_m_);
    terrain_grid_.hole_depth_m = static_cast<float>(grid_hole_depth_m_);
    terrain_grid_.inpaint_max_passes = grid_inpaint_passes_;
    terrain_grid_.height_percentile = grid_height_percentile_;
    terrain_grid_.smooth_max_step_m = grid_smooth_max_step_m_;
    terrain_grid_.smooth_clamp_passes = grid_smooth_clamp_passes_;
    terrain_grid_.smooth_median_radius_cells = grid_smooth_median_radius_cells_;
    terrain_grid_.ground_neighbor_radius_cells = grid_ground_neighbor_cells_;

    ground_connectivity_.params.max_surface_step_m = connectivity_max_step_m_;
    ground_connectivity_.params.seed_radius_m = connectivity_seed_radius_m_;

    column_extractor_.params.ndsm_min_m = static_cast<float>(ndsm_trunk_min_m_);
    column_extractor_.params.ndsm_max_m = static_cast<float>(ndsm_trunk_max_m_);
    column_extractor_.params.min_points_per_cell = column_min_points_per_cell_;
    column_extractor_.params.min_cells_per_column = column_min_cells_;
    column_extractor_.params.min_points_per_column =
      static_cast<std::size_t>(std::max(5, column_min_points_));
    column_extractor_.params.max_points_per_column =
      static_cast<std::size_t>(std::max(50, column_max_points_));
    column_extractor_.params.max_columns_per_frame =
      static_cast<std::size_t>(std::max(4, column_max_columns_));
    column_extractor_.cylinder_min_height_m = cylinder_min_height_m_;
    column_extractor_.cylinder_max_radius_m = cylinder_max_radius_m_;
    column_extractor_.cylinder_max_rmse_m = cylinder_max_rmse_m_;
    column_extractor_.cylinder_min_inlier_ratio = cylinder_min_inlier_ratio_;
    column_extractor_.cylinder_inlier_dist_m = cylinder_inlier_dist_m_;
    column_extractor_.cylinder_max_slice_height_m = cylinder_max_slice_height_m_;

    // Legacy slice detector params (kept synced for compatibility)
    slice_detector_.params.ndsm_min_m = static_cast<float>(ndsm_trunk_min_m_);
    slice_detector_.params.ndsm_max_m = static_cast<float>(ndsm_trunk_max_m_);
    slice_detector_.params.slice_height_m = static_cast<float>(slice_height_m_);
    slice_detector_.params.max_slices = slice_max_count_;
    slice_detector_.params.cluster_cell_m = static_cast<float>(slice_cluster_cell_m_);
    slice_detector_.params.min_points_per_cluster = slice_min_points_per_cluster_;
    slice_detector_.params.min_slices_for_trunk = slice_min_slices_for_trunk_;
    slice_detector_.params.assoc_max_xy_m = static_cast<float>(slice_assoc_max_xy_m_);
    slice_detector_.params.assoc_max_gap_slices = slice_assoc_max_gap_slices_;
    slice_detector_.params.min_continuity_score = static_cast<float>(slice_min_continuity_score_);
    slice_detector_.params.min_vertical_persistence =
      static_cast<float>(slice_min_vertical_persistence_);
    slice_detector_.params.max_centroid_drift_ratio = static_cast<float>(slice_max_centroid_drift_);
    slice_detector_.params.max_radius_cv = static_cast<float>(slice_max_radius_cv_);
    slice_detector_.params.ground_anchor_max_dz_m =
      static_cast<float>(slice_ground_anchor_max_dz_m_);
    slice_detector_.params.max_stems_per_frame =
      static_cast<std::size_t>(std::max(4, slice_max_stems_per_frame_));
    slice_detector_.cylinder_min_height_m = cylinder_min_height_m_;
    slice_detector_.cylinder_max_radius_m = cylinder_max_radius_m_;
    slice_detector_.cylinder_max_rmse_m = cylinder_max_rmse_m_;
    slice_detector_.cylinder_min_inlier_ratio = cylinder_min_inlier_ratio_;
    slice_detector_.cylinder_inlier_dist_m = cylinder_inlier_dist_m_;
    slice_detector_.cylinder_max_slice_height_m = cylinder_max_slice_height_m_;

    euclidean_detector_.params.ndsm_min_m = static_cast<float>(ndsm_trunk_min_m_);
    euclidean_detector_.params.ndsm_max_m = static_cast<float>(ndsm_trunk_max_m_);
    euclidean_detector_.params.cluster_tolerance_m = static_cast<float>(cluster_tol_);
    euclidean_detector_.params.min_cluster_size = std::max(4, slice_min_points_per_cluster_);
    euclidean_detector_.params.max_cluster_size = max_cluster_;
    euclidean_detector_.params.min_height_m = static_cast<float>(cylinder_min_height_m_);
    euclidean_detector_.params.max_diameter_m = static_cast<float>(cylinder_max_radius_m_ * 2.0);
    euclidean_detector_.params.min_verticality = static_cast<float>(trunk_min_vert_);
    euclidean_detector_.params.cylinder_max_rmse_m = static_cast<float>(cylinder_max_rmse_m_);
    euclidean_detector_.params.cylinder_min_inlier_ratio =
      static_cast<float>(cylinder_min_inlier_ratio_);
    euclidean_detector_.params.cylinder_inlier_dist_m = static_cast<float>(cylinder_inlier_dist_m_);
    euclidean_detector_.params.cylinder_max_radius_m = static_cast<float>(cylinder_max_radius_m_);
    euclidean_detector_.params.cylinder_max_height_m = static_cast<float>(ndsm_trunk_max_m_);
    euclidean_detector_.params.max_detections_per_frame = slice_max_stems_per_frame_;

    landmark_tracker_.params.assoc_max_xy_m = landmark_assoc_xy_m_;
    landmark_tracker_.params.max_misses = landmark_max_misses_;
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    static const std::unordered_set<std::string> kRestartOnly = {
      "input_topic", "processing_frame",
      "ground_topic", "trunks_topic", "obstacles_topic", "markers_topic",
      "tree_landmarks_topic", "debug_stats_topic", "ndsm_debug_topic",
      "slice_debug_ndsm_topic", "slice_debug_clusters_topic",
      "slice_debug_rejected_topic", "slice_debug_accepted_topic",
      "terrain_mesh_topic", "ground_suspended_topic",
      "publish_ndsm_debug", "publish_slice_pipeline_debug", "publish_debug_stats",
      "publish_terrain_mesh", "publish_ground_suspended_debug",
      "publish_ground_ref_debug", "ground_ref_h_topic", "ground_ref_zg_topic",
      "ground_ref_no_obs_cells_topic", "ground_ref_confidence_cells_topic"
    };

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    std::vector<std::string> changed;
    changed.reserve(params.size());

    for (const auto & p : params) {
      const auto & n = p.get_name();
      if (kRestartOnly.count(n) != 0U) {
        result.successful = false;
        result.reason = "parameter requires node restart: " + n;
        return result;
      }

      try {
        if (n == "voxel_leaf_size_m") voxel_leaf_ = p.as_double();
        else if (n == "trunk_voxel_leaf_size_m") trunk_voxel_leaf_m_ = p.as_double();
        else if (n == "min_range_m") min_range_ = p.as_double();
        else if (n == "max_range_m") max_range_ = p.as_double();
        else if (n == "min_z_m") min_z_ = p.as_double();
        else if (n == "max_z_m") max_z_ = p.as_double();
        else if (n == "ground_ransac_distance_m") ground_dist_ = p.as_double();
        else if (n == "ground_ransac_max_iterations") ground_max_iter_ = p.as_int();
        else if (n == "ground_max_slope_deg") ground_max_slope_ = p.as_double();
        else if (n == "ground_z_band_m") ground_z_band_ = p.as_double();
        else if (n == "ground_method") ground_method_ = p.as_string();
        else if (n == "grid_size_x_m") grid_size_x_m_ = p.as_double();
        else if (n == "grid_size_y_m") grid_size_y_m_ = p.as_double();
        else if (n == "grid_resolution_m") grid_resolution_m_ = p.as_double();
        else if (n == "grid_ground_height_thresh_m") grid_ground_height_thresh_m_ = p.as_double();
        else if (n == "grid_hole_depth_m") grid_hole_depth_m_ = p.as_double();
        else if (n == "grid_inpaint_passes") grid_inpaint_passes_ = p.as_int();
        else if (n == "grid_height_percentile") grid_height_percentile_ = p.as_double();
        else if (n == "grid_smooth_max_step_m") grid_smooth_max_step_m_ = p.as_double();
        else if (n == "grid_smooth_clamp_passes") grid_smooth_clamp_passes_ = p.as_int();
        else if (n == "grid_smooth_median_radius_cells") grid_smooth_median_radius_cells_ = p.as_int();
        else if (n == "grid_ground_neighbor_cells") grid_ground_neighbor_cells_ = p.as_int();
        else if (n == "ground_connectivity_enable") ground_connectivity_enable_ = p.as_bool();
        else if (n == "ground_connectivity_max_step_m") connectivity_max_step_m_ = p.as_double();
        else if (n == "ground_connectivity_seed_radius_m") connectivity_seed_radius_m_ = p.as_double();
        else if (n == "trunk_cluster_tolerance_m") cluster_tol_ = p.as_double();
        else if (n == "trunk_min_cluster_size") min_cluster_ = p.as_int();
        else if (n == "trunk_max_cluster_size") max_cluster_ = p.as_int();
        else if (n == "trunk_min_height_m") trunk_min_height_ = p.as_double();
        else if (n == "trunk_max_radius_m") trunk_max_radius_ = p.as_double();
        else if (n == "trunk_min_verticality") trunk_min_vert_ = p.as_double();
        else if (n == "trunk_method") trunk_method_ = p.as_string();
        else if (n == "ndsm_trunk_min_m") ndsm_trunk_min_m_ = p.as_double();
        else if (n == "ndsm_trunk_max_m") ndsm_trunk_max_m_ = p.as_double();
        else if (n == "column_min_points_per_cell") column_min_points_per_cell_ = p.as_int();
        else if (n == "column_min_cells_per_column") column_min_cells_ = p.as_int();
        else if (n == "column_min_points_per_column") column_min_points_ = p.as_int();
        else if (n == "column_max_points_per_column") column_max_points_ = p.as_int();
        else if (n == "column_max_columns_per_frame") column_max_columns_ = p.as_int();
        else if (n == "cylinder_min_height_m") cylinder_min_height_m_ = p.as_double();
        else if (n == "cylinder_max_radius_m") cylinder_max_radius_m_ = p.as_double();
        else if (n == "cylinder_max_rmse_m") cylinder_max_rmse_m_ = p.as_double();
        else if (n == "cylinder_min_inlier_ratio") cylinder_min_inlier_ratio_ = p.as_double();
        else if (n == "cylinder_inlier_dist_m") cylinder_inlier_dist_m_ = p.as_double();
        else if (n == "cylinder_max_slice_height_m") cylinder_max_slice_height_m_ = p.as_double();
        else if (n == "slice_height_m") slice_height_m_ = p.as_double();
        else if (n == "slice_max_count") slice_max_count_ = p.as_int();
        else if (n == "slice_cluster_cell_m") slice_cluster_cell_m_ = p.as_double();
        else if (n == "slice_min_points_per_cluster") slice_min_points_per_cluster_ = p.as_int();
        else if (n == "slice_min_slices_for_trunk") slice_min_slices_for_trunk_ = p.as_int();
        else if (n == "slice_assoc_max_xy_m") slice_assoc_max_xy_m_ = p.as_double();
        else if (n == "slice_assoc_max_gap_slices") slice_assoc_max_gap_slices_ = p.as_int();
        else if (n == "slice_min_continuity_score") slice_min_continuity_score_ = p.as_double();
        else if (n == "slice_min_vertical_persistence") slice_min_vertical_persistence_ = p.as_double();
        else if (n == "slice_max_centroid_drift") slice_max_centroid_drift_ = p.as_double();
        else if (n == "slice_max_radius_cv") slice_max_radius_cv_ = p.as_double();
        else if (n == "slice_ground_anchor_max_dz_m") slice_ground_anchor_max_dz_m_ = p.as_double();
        else if (n == "slice_max_stems_per_frame") slice_max_stems_per_frame_ = p.as_int();
        else if (n == "landmark_assoc_max_xy_m") landmark_assoc_xy_m_ = p.as_double();
        else if (n == "landmark_max_misses") landmark_max_misses_ = p.as_int();
        else if (n == "always_publish_segmentation_clouds") always_publish_clouds_ = p.as_bool();
        else if (n == "pipeline_log_interval_frames") pipeline_log_interval_ = p.as_int();
        else {
          // Ignore unknown/unhandled params.
          continue;
        }
        changed.push_back(n);
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = "invalid value for " + n + ": " + e.what();
        return result;
      }
    }

    if (!changed.empty()) {
      apply_runtime_configuration();
      std::ostringstream oss;
      for (std::size_t i = 0; i < changed.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << changed[i];
      }
      RCLCPP_INFO(get_logger(), "updated runtime params: %s", oss.str().c_str());
    }
    return result;
  }

  struct ColumnPipelineDebug
  {
    std::size_t n_band{0};
    std::size_t n_columns{0};
    std::size_t n_detections{0};
    std::size_t n_tracks{0};
    std::size_t rej_sparse{0};
    std::size_t rej_cylinder{0};
    std::size_t rej_rmse{0};
    std::size_t rej_height{0};
    std::size_t rej_radius{0};
    double mean_ndsm_h{0.0};
    double mean_cyl_rmse{0.0};
    double mean_continuity_score{0.0};
    std::size_t rej_slice_continuity{0};
    std::size_t rej_slice_ground{0};
    std::size_t slice_n_stems{0};
    std::size_t slice_n_2d_clusters{0};
    std::size_t slice_n_slices_ok{0};
    std::size_t slice_rej_sparse_slices{0};
    std::size_t slice_rej_cont_score{0};
    std::size_t slice_rej_cont_persist{0};
    std::size_t slice_rej_cont_drift{0};
    std::size_t slice_rej_ground_dz{0};
    std::size_t slice_rej_ground_cell{0};
    std::size_t slice_rej_cylinder{0};
    int slice_best_rej_slices{0};
    float slice_best_cont{0.0f};
    float slice_best_drift{0.0f};
    float slice_best_bottom_dz{0.0f};
    std::size_t funnel_nonground{0};
    std::size_t funnel_band_skip_nan{0};
    std::size_t funnel_band_skip_h{0};
    std::size_t funnel_grid_cells{0};
    std::size_t funnel_rej_cluster_small{0};
    int funnel_max_c2d_slice{0};
    forest_3d_perception::GroundRefFrameStats ground_ref{};
    forest_3d_perception::GroundRefGridStats ground_ref_grid{};
  };

  void fill_slice_debug(ColumnPipelineDebug & dbg, const forest_3d_perception::TrunkPipelineFunnel & s)
  {
    dbg.n_band = s.n_band_points;
    dbg.n_columns = s.n_accepted_cylinders;
    dbg.rej_sparse = s.reject_sparse_few_slices + s.reject_sparse_few_points;
    dbg.rej_cylinder = s.reject_cylinder_short + s.reject_cylinder_wide +
      s.reject_cylinder_rmse + s.reject_cylinder_inliers;
    dbg.rej_slice_continuity = s.reject_cont_score + s.reject_cont_persistence +
      s.reject_cont_drift + s.reject_cont_radius_cv;
    dbg.rej_slice_ground = s.reject_ground_dz + s.reject_ground_cell + s.reject_ground_z_nan;
    dbg.slice_n_stems = s.n_stems_finished;
    dbg.slice_n_2d_clusters = s.n_clusters_2d;
    dbg.slice_n_slices_ok = s.n_slices_nonempty;
    dbg.slice_rej_sparse_slices = s.reject_sparse_few_slices;
    dbg.slice_rej_cont_score = s.reject_cont_score;
    dbg.slice_rej_cont_persist = s.reject_cont_persistence;
    dbg.slice_rej_cont_drift = s.reject_cont_drift;
    dbg.slice_rej_ground_dz = s.reject_ground_dz;
    dbg.slice_rej_ground_cell = s.reject_ground_cell;
    dbg.slice_rej_cylinder = dbg.rej_cylinder;
    dbg.slice_best_rej_slices = s.best_rej_slices;
    dbg.slice_best_cont = s.best_rej_continuity;
    dbg.slice_best_drift = s.best_rej_drift;
    dbg.slice_best_bottom_dz = s.best_rej_bottom_dz;
    dbg.funnel_nonground = s.n_nonground_in;
    dbg.funnel_band_skip_nan = s.n_band_skip_nan_ground;
    dbg.funnel_band_skip_h = s.n_band_skip_height;
    dbg.funnel_grid_cells = s.n_grid_cells_occupied;
    dbg.funnel_rej_cluster_small = s.reject_cluster_too_small;
    dbg.funnel_max_c2d_slice = s.max_clusters_in_slice;
  }

  void log_trunk_funnel_throttled(const forest_3d_perception::TrunkPipelineFunnel & f) const
  {
    if (pipeline_log_interval_ <= 0 || trunk_method_ != "slice") {
      return;
    }
    if (pipeline_frame_count_ % static_cast<std::size_t>(pipeline_log_interval_) != 0) {
      return;
    }
    const char * dominant = "none";
    std::size_t dom_n = 0;
    auto consider = [&](const char * name, std::size_t n) {
      if (n > dom_n) {
        dom_n = n;
        dominant = name;
      }
    };
    consider("cluster_2d_small", f.reject_cluster_too_small);
    consider("sparse_slices", f.reject_sparse_few_slices);
    consider("cont_score", f.reject_cont_score);
    consider("cont_persist", f.reject_cont_persistence);
    consider("cont_drift", f.reject_cont_drift);
    consider("ground_dz", f.reject_ground_dz);
    consider("ground_cell", f.reject_ground_cell);
    consider("cylinder", f.reject_cylinder_short + f.reject_cylinder_wide +
      f.reject_cylinder_rmse + f.reject_cylinder_inliers);

    RCLCPP_INFO(
      get_logger(),
      "TRUNK_FUNNEL nonground=%zu band=%zu (skip_nan=%zu skip_h=%zu) "
      "slices=%zu/%zu cells=%zu c2d=%zu (rej_small=%zu max/slice=%d) stems=%zu cyl=%zu | dominant=%s",
      f.n_nonground_in, f.n_band_points, f.n_band_skip_nan_ground, f.n_band_skip_height,
      f.n_slices_nonempty, f.n_slices_used, f.n_grid_cells_occupied, f.n_clusters_2d,
      f.reject_cluster_too_small, f.max_clusters_in_slice, f.n_stems_finished, f.n_accepted_cylinders,
      dominant);

    if (f.n_accepted_cylinders == 0 && f.n_clusters_2d == 0 && f.n_band_points > 50) {
      RCLCPP_WARN(
        get_logger(),
        "TRUNK_FUNNEL: band has points but 0 clusters — check 2D BFS/min_points (cell=%.2fm min=%d)",
        slice_cluster_cell_m_, slice_min_points_per_cluster_);
    }
    if (f.n_accepted_cylinders == 0 &&
      (f.n_reject_samples > 0 || f.best_rej_slices > 0))
    {
      RCLCPP_WARN(
        get_logger(),
        "  best_near_miss: slices=%d cont=%.2f persist=%.2f drift=%.2f bottom_dz=%.2fm",
        f.best_rej_slices, f.best_rej_continuity, f.best_rej_persist, f.best_rej_drift,
        f.best_rej_bottom_dz);
      for (int i = 0; i < f.n_reject_samples; ++i) {
        const auto & r = f.reject_samples[i];
        RCLCPP_WARN(
          get_logger(),
          "  reject_sample[%d] reason=%s slices=%d xy=(%.2f,%.2f) thr=%.3f meas=%.3f cont=%.2f drift=%.2f",
          i, r.reason, r.n_slices, r.cx, r.cy, r.threshold, r.measured, r.continuity, r.drift);
      }
    }
  }

  void publish_debug_json(
    const sensor_msgs::msg::PointCloud2 & hdr,
    const char * status,
    std::size_t n_raw,
    std::size_t n_crop,
    std::size_t n_voxel,
    std::size_t n_ground,
    std::size_t n_nonground,
    std::size_t n_holes,
    std::size_t n_clusters,
    std::size_t n_trunk_clusters,
    std::size_t rej_small,
    std::size_t rej_height,
    std::size_t rej_radius,
    std::size_t rej_verticality,
    bool ground_plane_rejected,
    double grid_coverage_pct,
    double grid_mean_abs_dz,
    std::size_t grid_unknown_pts,
    std::size_t grid_cells_clamped,
    const ConnectivityDebug & conn,
    const ColumnPipelineDebug * col = nullptr) const
  {
    if (!publish_debug_stats_ || !pub_debug_) {
      return;
    }
    static const ColumnPipelineDebug kEmptyCol;
    const ColumnPipelineDebug & c = col ? *col : kEmptyCol;
    std::ostringstream oss;
    oss << "{\"status\":\"" << status << "\""
        << ",\"ground_method\":\"" << ground_method_ << "\""
        << ",\"trunk_method\":\"" << trunk_method_ << "\""
        << ",\"stamp\":" << hdr.header.stamp.sec << "." << hdr.header.stamp.nanosec
        << ",\"n_raw\":" << n_raw
        << ",\"n_crop\":" << n_crop
        << ",\"n_voxel\":" << n_voxel
        << ",\"n_ground\":" << n_ground
        << ",\"n_nonground\":" << n_nonground
        << ",\"n_holes\":" << n_holes
        << ",\"n_clusters\":" << n_clusters
        << ",\"n_trunk\":" << n_trunk_clusters
        << ",\"reject_cluster_small\":" << rej_small
        << ",\"reject_height\":" << rej_height
        << ",\"reject_radius\":" << rej_radius
        << ",\"reject_verticality\":" << rej_verticality
        << ",\"reject_column_sparse\":" << c.rej_sparse
        << ",\"reject_cylinder\":" << c.rej_cylinder
        << ",\"reject_cylinder_rmse\":" << c.rej_rmse
        << ",\"n_ndsm_band\":" << c.n_band
        << ",\"n_columns_found\":" << c.n_columns
        << ",\"n_tracks\":" << c.n_tracks
        << ",\"mean_ndsm_h_trunk_m\":" << c.mean_ndsm_h
        << ",\"mean_cylinder_rmse_m\":" << c.mean_cyl_rmse
        << ",\"mean_continuity_score\":" << c.mean_continuity_score
        << ",\"reject_slice_continuity\":" << c.rej_slice_continuity
        << ",\"reject_slice_ground\":" << c.rej_slice_ground
        << ",\"slice_n_stems\":" << c.slice_n_stems
        << ",\"slice_n_2d_clusters\":" << c.slice_n_2d_clusters
        << ",\"slice_n_slices_ok\":" << c.slice_n_slices_ok
        << ",\"slice_rej_sparse_slices\":" << c.slice_rej_sparse_slices
        << ",\"slice_rej_cont_score\":" << c.slice_rej_cont_score
        << ",\"slice_rej_cont_persist\":" << c.slice_rej_cont_persist
        << ",\"slice_rej_cont_drift\":" << c.slice_rej_cont_drift
        << ",\"slice_rej_ground_dz\":" << c.slice_rej_ground_dz
        << ",\"slice_rej_ground_cell\":" << c.slice_rej_ground_cell
        << ",\"slice_rej_cylinder\":" << c.slice_rej_cylinder
        << ",\"slice_best_rej_slices\":" << c.slice_best_rej_slices
        << ",\"slice_best_cont\":" << c.slice_best_cont
        << ",\"slice_best_drift\":" << c.slice_best_drift
        << ",\"slice_best_bottom_dz\":" << c.slice_best_bottom_dz
        << ",\"funnel_nonground\":" << c.funnel_nonground
        << ",\"funnel_band_skip_nan\":" << c.funnel_band_skip_nan
        << ",\"funnel_band_skip_h\":" << c.funnel_band_skip_h
        << ",\"funnel_grid_cells\":" << c.funnel_grid_cells
        << ",\"funnel_rej_cluster_small\":" << c.funnel_rej_cluster_small
        << ",\"funnel_max_c2d_slice\":" << c.funnel_max_c2d_slice
        << ",\"ground_plane_rejected\":" << (ground_plane_rejected ? "true" : "false")
        << ",\"grid_coverage_pct\":" << grid_coverage_pct
        << ",\"grid_mean_abs_dz_ground_m\":" << grid_mean_abs_dz
        << ",\"grid_unknown_pts\":" << grid_unknown_pts
        << ",\"grid_cells_clamped\":" << grid_cells_clamped
        << ",\"ground_connectivity\":" << (conn.enabled ? "true" : "false")
        << ",\"n_ground_raw\":" << conn.n_ground_raw
        << ",\"n_ground_connected\":" << conn.n_ground_connected
        << ",\"n_suspended\":" << conn.n_suspended
        << ",\"gcr_pct\":" << conn.gcr_pct
        << ",\"suspended_pct_of_raw\":" << conn.suspended_pct
        << ",\"mean_abs_dz_connected_m\":" << conn.mean_abs_dz_connected
        << ",\"n_connected_cells\":" << conn.n_connected_cells
        << ",\"gr_mesh_obs_cells\":" << c.ground_ref_grid.mesh_cells_observed
        << ",\"gr_mesh_inpaint_cells\":" << c.ground_ref_grid.mesh_cells_inpainted
        << ",\"gr_mesh_unknown_cells\":" << c.ground_ref_grid.mesh_cells_unknown
        << ",\"gr_surface_percentile_cells\":" << c.ground_ref_grid.surface_cells_percentile
        << ",\"gr_surface_inpaint_cells\":" << c.ground_ref_grid.surface_cells_inpainted
        << ",\"gr_nonground_pts\":" << c.ground_ref.n_points
        << ",\"gr_h_below_ndsm_min\":" << c.ground_ref.n_h_below_ndsm_min
        << ",\"gr_h_in_ndsm_band\":" << c.ground_ref.n_h_in_ndsm_band
        << ",\"gr_h_above_ndsm_max\":" << c.ground_ref.n_h_above_ndsm_max
        << ",\"gr_under_inpaint_cell\":" << c.ground_ref.n_under_inpainted_cell
        << ",\"gr_under_no_obs_cell\":" << c.ground_ref.n_under_no_obs_cell
        << ",\"gr_mean_h_m\":" << c.ground_ref.mean_h
        << ",\"gr_mean_inpaint_delta_m\":" << c.ground_ref.mean_inpaint_delta_m
        << ",\"gr_mean_zg_minus_obs_m\":" << c.ground_ref.mean_zg_minus_z_obs_m
        << "}";
    std_msgs::msg::String out;
    out.data = oss.str();
    pub_debug_->publish(out);
  }

  void log_pipeline_throttled(
    const char * status,
    std::size_t n_raw,
    std::size_t n_voxel,
    std::size_t n_ground,
    std::size_t n_nonground,
    std::size_t n_trunks,
    std::size_t n_obstacles,
    std::size_t n_clusters,
    const ColumnPipelineDebug * col) const
  {
    if (pipeline_log_interval_ <= 0) {
      return;
    }
    ++pipeline_frame_count_;
    if (pipeline_frame_count_ % static_cast<std::size_t>(pipeline_log_interval_) != 0) {
      return;
    }
    const ColumnPipelineDebug & c = col ? *col : ColumnPipelineDebug();
    if (trunk_method_ == "slice") {
      RCLCPP_INFO(
        get_logger(),
        "pipeline [%s] raw=%zu voxel=%zu ground=%zu nonground=%zu trunks=%zu obstacles=%zu "
        "cols=%zu | slice: band=%zu sl_ok=%zu c2d=%zu stems=%zu tracks=%zu",
        status, n_raw, n_voxel, n_ground, n_nonground, n_trunks, n_obstacles, c.n_columns,
        c.n_band, c.slice_n_slices_ok, c.slice_n_2d_clusters, c.slice_n_stems, c.n_tracks);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "pipeline [%s] raw=%zu voxel=%zu ground=%zu nonground=%zu trunks=%zu obstacles=%zu "
        "clusters=%zu | trunk_dbg: band=%zu cols=%zu tracks=%zu cont=%.2f",
        status, n_raw, n_voxel, n_ground, n_nonground, n_trunks, n_obstacles, n_clusters,
        c.n_band, c.n_columns, c.n_tracks, c.mean_continuity_score);
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const ConnectivityDebug empty_conn;
    const auto publish_fail = [&](const char * status) {
      publish_debug_json(
        *msg, status, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0.0, 0.0, 0, 0, empty_conn);
    };

    // Transform to processing frame
    sensor_msgs::msg::PointCloud2 cloud_base;
    try {
      auto tf = tf_buffer_.lookupTransform(
        processing_frame_, msg->header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      tf2::doTransform(*msg, cloud_base, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "TF %s→%s: %s", msg->header.frame_id.c_str(), processing_frame_.c_str(), ex.what());
      publish_fail("tf_fail");
      return;
    }
    cloud_base.header.frame_id = processing_frame_;
    cloud_base.header.stamp = msg->header.stamp;

    // Convert to PCL
    CloudT::Ptr cloud_raw(new CloudT);
    pcl::fromROSMsg(cloud_base, *cloud_raw);

    if (cloud_raw->empty()) {
      publish_fail("empty_raw");
      return;
    }

    // Preprocessing: range crop + height crop
    CloudT::Ptr cloud_cropped(new CloudT);
    crop_cloud(cloud_raw, cloud_cropped);

    if (cloud_cropped->size() < 30) {
      publish_debug_json(
        *msg, "too_few_crop", cloud_raw->size(), cloud_cropped->size(), 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0.0, 0.0, 0, 0, empty_conn);
      return;
    }

    // Voxel downsample
    CloudT::Ptr cloud_ds(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_cropped);
    vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    vg.filter(*cloud_ds);

    CloudT::Ptr cloud_ground(new CloudT);
    CloudT::Ptr cloud_nonground(new CloudT);
    CloudT::Ptr cloud_holes(new CloudT);
    bool ground_plane_rejected = false;
    double grid_coverage_pct = 0.0;
    double grid_mean_abs_dz = 0.0;
    std::size_t grid_unknown_pts = 0;
    std::size_t grid_cells_clamped = 0;
    ConnectivityDebug conn_dbg;

    if (ground_method_ == "grid") {
      const auto build = terrain_grid_.build_from_cloud(*cloud_ds);
      grid_coverage_pct = build.coverage_pct;
      grid_cells_clamped = build.cells_clamped;
      if (ground_connectivity_enable_) {
        const auto seg = ground_connectivity_.segment(terrain_grid_, *cloud_ds);
        cloud_ground = seg.ground_connected;
        cloud_holes = seg.holes;
        cloud_nonground = seg.nonground;
        grid_mean_abs_dz = seg.mean_abs_dz_connected;
        grid_unknown_pts = seg.n_unknown;
        conn_dbg.enabled = true;
        conn_dbg.n_ground_raw = seg.n_ground_raw;
        conn_dbg.n_ground_connected = seg.n_ground_connected;
        conn_dbg.n_suspended = seg.n_suspended;
        conn_dbg.gcr_pct = seg.gcr_pct;
        conn_dbg.suspended_pct = seg.suspended_pct_of_raw;
        conn_dbg.mean_abs_dz_connected = seg.mean_abs_dz_connected;
        conn_dbg.n_connected_cells = seg.n_connected_cells;
        cell_connected_ = seg.cell_connected;
        if (publish_suspended_debug_ && pub_suspended_) {
          publish_cloud(pub_suspended_, seg.ground_suspended, msg->header.stamp);
        }
        // Suspended low vegetation → non-ground for clustering / obstacles
        if (!seg.ground_suspended->empty()) {
          *cloud_nonground += *seg.ground_suspended;
        }
      } else {
        const auto seg = terrain_grid_.segment_cloud(*cloud_ds);
        cloud_ground = seg.ground;
        cloud_holes = seg.holes;
        cloud_nonground = seg.nonground;
        grid_mean_abs_dz = seg.mean_abs_dz_ground;
        grid_unknown_pts = seg.n_unknown;
      }
      if (!cloud_ground->empty()) {
        terrain_grid_.build_mesh_surface_from_cloud(*cloud_ground);
      }
      if (publish_terrain_mesh_) {
        publish_terrain_mesh(msg->header.stamp);
      }
    } else {
      pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr ground_coeffs(new pcl::ModelCoefficients);
      ground_plane_rejected = segment_ground(cloud_ds, ground_inliers, ground_coeffs);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(cloud_ds);
      extract.setIndices(ground_inliers);
      extract.setNegative(false);
      extract.filter(*cloud_ground);
      extract.setNegative(true);
      extract.filter(*cloud_nonground);
    }

    CloudT::Ptr cloud_trunks(new CloudT);
    CloudT::Ptr cloud_obstacles(new CloudT);
    std::vector<TrunkCandidate> trunk_candidates;
    std::vector<TrunkCandidate> tracked_landmarks;
    std::size_t rej_small = 0;
    std::size_t rej_height = 0;
    std::size_t rej_radius = 0;
    std::size_t rej_verticality = 0;
    std::vector<pcl::PointIndices> clusters;
    ColumnPipelineDebug col_dbg;

    const bool use_slice_trunks =
      (trunk_method_ == "slice" && ground_method_ == "grid");
    const bool use_column_trunks =
      (!use_slice_trunks && trunk_method_ == "column" && ground_method_ == "grid");

    if (use_slice_trunks) {
      try {
        const CloudT::Ptr cloud_trunk_in =
          build_trunk_nonground_cloud(cloud_cropped, cloud_nonground);
        process_trunks_euclidean(
          cloud_trunk_in, cloud_trunks, cloud_obstacles, trunk_candidates,
          tracked_landmarks, col_dbg, msg->header.stamp);
        rej_small = col_dbg.rej_sparse;
        rej_height = col_dbg.rej_height;
        rej_radius = col_dbg.rej_radius;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "slice trunk pipeline failed (%s) — fallback to cluster", ex.what());
        cloud_trunks->clear();
        cloud_obstacles->clear();
        trunk_candidates.clear();
        tracked_landmarks.clear();
        col_dbg = ColumnPipelineDebug();
        run_cluster_trunk_pipeline(
          cloud_nonground, cloud_trunks, cloud_obstacles, trunk_candidates,
          clusters, rej_small, rej_height, rej_radius, rej_verticality);
      } catch (...) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "slice trunk pipeline failed (unknown) — fallback to cluster");
        cloud_trunks->clear();
        cloud_obstacles->clear();
        trunk_candidates.clear();
        tracked_landmarks.clear();
        col_dbg = ColumnPipelineDebug();
        run_cluster_trunk_pipeline(
          cloud_nonground, cloud_trunks, cloud_obstacles, trunk_candidates,
          clusters, rej_small, rej_height, rej_radius, rej_verticality);
      }
    } else if (use_column_trunks) {
      try {
        process_trunks_column(
          cloud_nonground, cloud_trunks, cloud_obstacles, trunk_candidates,
          tracked_landmarks, col_dbg, msg->header.stamp);
        rej_height = col_dbg.rej_height;
        rej_radius = col_dbg.rej_radius;
        rej_small = col_dbg.rej_sparse;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "column trunk pipeline failed (%s) — fallback to cluster", ex.what());
        cloud_trunks->clear();
        cloud_obstacles->clear();
        trunk_candidates.clear();
        tracked_landmarks.clear();
        col_dbg = ColumnPipelineDebug();
        run_cluster_trunk_pipeline(
          cloud_nonground, cloud_trunks, cloud_obstacles, trunk_candidates,
          clusters, rej_small, rej_height, rej_radius, rej_verticality);
      } catch (...) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "column trunk pipeline failed (unknown) — fallback to cluster");
        cloud_trunks->clear();
        cloud_obstacles->clear();
        trunk_candidates.clear();
        tracked_landmarks.clear();
        col_dbg = ColumnPipelineDebug();
        run_cluster_trunk_pipeline(
          cloud_nonground, cloud_trunks, cloud_obstacles, trunk_candidates,
          clusters, rej_small, rej_height, rej_radius, rej_verticality);
      }
    } else {
      run_cluster_trunk_pipeline(
        cloud_nonground, cloud_trunks, cloud_obstacles, trunk_candidates,
        clusters, rej_small, rej_height, rej_radius, rej_verticality);
    }

    // Depth holes (below local surface) count as obstacles for navigation
    if (!cloud_holes->empty()) {
      *cloud_obstacles += *cloud_holes;
    }

    publish_ground_reference_debug(cloud_nonground, col_dbg, msg->header.stamp);

    // Publish results
    publish_cloud(pub_ground_, cloud_ground, msg->header.stamp);
    publish_cloud(pub_trunks_, cloud_trunks, msg->header.stamp);
    publish_cloud(pub_obstacles_, cloud_obstacles, msg->header.stamp);
    const std::vector<TrunkCandidate> & markers_src =
      trunk_candidates.empty() && !tracked_landmarks.empty() ? tracked_landmarks : trunk_candidates;
    publish_trunk_markers(markers_src, msg->header.stamp);
    if (!tracked_landmarks.empty()) {
      publish_tracked_landmark_markers(tracked_landmarks, msg->header.stamp);
    }

    col_dbg.n_detections = trunk_candidates.size();
    const std::size_t n_clusters_report =
      (use_slice_trunks || use_column_trunks) ? col_dbg.n_columns : clusters.size();
    log_pipeline_throttled(
      "ok", cloud_raw->size(), cloud_ds->size(), cloud_ground->size(), cloud_nonground->size(),
      cloud_trunks->size(), cloud_obstacles->size(), n_clusters_report,
      (use_slice_trunks || use_column_trunks) ? &col_dbg : nullptr);
    publish_debug_json(
      *msg, "ok", cloud_raw->size(), cloud_cropped->size(), cloud_ds->size(),
      cloud_ground->size(), cloud_nonground->size(), cloud_holes->size(),
      n_clusters_report, trunk_candidates.size(),
      rej_small, rej_height, rej_radius, rej_verticality, ground_plane_rejected,
      grid_coverage_pct, grid_mean_abs_dz, grid_unknown_pts, grid_cells_clamped, conn_dbg,
      &col_dbg);

    RCLCPP_DEBUG(get_logger(),
      "seg: in=%zu ds=%zu ground=%zu trunks=%zu(%zu) obstacles=%zu",
      cloud_raw->size(), cloud_ds->size(), cloud_ground->size(),
      cloud_trunks->size(), trunk_candidates.size(), cloud_obstacles->size());
  }

  void run_cluster_trunk_pipeline(
    const CloudT::Ptr & cloud_nonground,
    CloudT::Ptr & cloud_trunks,
    CloudT::Ptr & cloud_obstacles,
    std::vector<TrunkCandidate> & trunk_candidates,
    std::vector<pcl::PointIndices> & clusters,
    std::size_t & rej_small,
    std::size_t & rej_height,
    std::size_t & rej_radius,
    std::size_t & rej_verticality)
  {
    clusters.clear();
    if (cloud_nonground->size() >= static_cast<std::size_t>(min_cluster_)) {
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
      tree->setInputCloud(cloud_nonground);
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(cluster_tol_);
      ec.setMinClusterSize(min_cluster_);
      ec.setMaxClusterSize(max_cluster_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_nonground);
      ec.extract(clusters);
    }

    std::vector<bool> nonground_used(cloud_nonground->size(), false);

    for (const auto & cluster : clusters) {
      CloudT::Ptr cluster_cloud(new CloudT);
      for (int idx : cluster.indices) {
        if (idx >= 0 && static_cast<std::size_t>(idx) < nonground_used.size()) {
          nonground_used[static_cast<std::size_t>(idx)] = true;
        }
        cluster_cloud->push_back((*cloud_nonground)[idx]);
      }

      TrunkCandidate tc;
      const auto reason = classify_cluster(cluster_cloud, tc);
      if (reason == TrunkRejectReason::Accepted) {
        *cloud_trunks += *cluster_cloud;
        trunk_candidates.push_back(tc);
      } else {
        *cloud_obstacles += *cluster_cloud;
        switch (reason) {
          case TrunkRejectReason::TooFewPoints:
            ++rej_small;
            break;
          case TrunkRejectReason::TooShort:
            ++rej_height;
            break;
          case TrunkRejectReason::TooWide:
            ++rej_radius;
            break;
          case TrunkRejectReason::NotVertical:
            ++rej_verticality;
            break;
          default:
            break;
        }
      }
    }

    for (std::size_t i = 0; i < cloud_nonground->size(); ++i) {
      if (!nonground_used[i]) {
        cloud_obstacles->push_back((*cloud_nonground)[i]);
      }
    }
  }

  void process_trunks_slice(
    const CloudT::Ptr & cloud_nonground,
    CloudT::Ptr & cloud_trunks,
    CloudT::Ptr & cloud_obstacles,
    std::vector<TrunkCandidate> & trunk_candidates,
    std::vector<TrunkCandidate> & tracked_landmarks,
    ColumnPipelineDebug & slice_dbg,
    const builtin_interfaces::msg::Time & stamp)
  {
    forest_3d_perception::TrunkPipelineFunnel funnel;
    forest_3d_perception::SliceDebugClouds slice_clouds;
    forest_3d_perception::SliceDebugClouds * slice_clouds_ptr =
      publish_slice_pipeline_debug_ ? &slice_clouds : nullptr;
    const auto detections = slice_detector_.detect(
      *cloud_nonground, terrain_grid_, cell_connected_, &funnel, slice_clouds_ptr);

    fill_slice_debug(slice_dbg, funnel);
    slice_dbg.rej_height = 0;
    slice_dbg.rej_radius = 0;
    log_trunk_funnel_throttled(funnel);

    if (publish_slice_pipeline_debug_) {
      if (pub_slice_ndsm_ && slice_clouds.ndsm_band && !slice_clouds.ndsm_band->empty()) {
        publish_cloud(pub_slice_ndsm_, slice_clouds.ndsm_band, stamp);
      }
      if (pub_slice_clusters_ && slice_clouds.clusters_2d && !slice_clouds.clusters_2d->empty()) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*slice_clouds.clusters_2d, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = processing_frame_;
        pub_slice_clusters_->publish(msg);
      }
      if (pub_slice_rejected_ && slice_clouds.rejected_points &&
        !slice_clouds.rejected_points->empty())
      {
        publish_cloud(pub_slice_rejected_, slice_clouds.rejected_points, stamp);
      }
      if (pub_slice_accepted_ && slice_clouds.accepted_points &&
        !slice_clouds.accepted_points->empty())
      {
        publish_cloud(pub_slice_accepted_, slice_clouds.accepted_points, stamp);
      }
    }

    std::vector<bool> used(cloud_nonground->size(), false);
    std::vector<forest_3d_perception::CylinderObservation> cylinders;
    cylinders.reserve(detections.size());
    double sum_rmse = 0.0;
    double sum_cont = 0.0;

    for (const auto & det : detections) {
      cylinders.push_back(det.cylinder);
      sum_rmse += static_cast<double>(det.cylinder.rmse);
      sum_cont += static_cast<double>(det.metrics.continuity_score);

      TrunkCandidate tc;
      tc.centroid.x() = det.cylinder.cx;
      tc.centroid.y() = det.cylinder.cy;
      tc.centroid.z() = det.cylinder.z_base + 0.5f * det.cylinder.height;
      tc.z_base = det.cylinder.z_base;
      tc.height = det.cylinder.height;
      tc.radius = det.cylinder.radius;
      tc.cylinder_rmse = det.cylinder.rmse;
      tc.verticality = std::clamp(
        1.0f - det.metrics.centroid_drift / static_cast<float>(slice_max_centroid_drift_), 0.0f, 1.0f);
      tc.confidence = det.metrics.continuity_score;
      trunk_candidates.push_back(tc);

      for (std::size_t idx : det.point_indices) {
        if (idx < used.size()) {
          used[idx] = true;
          cloud_trunks->push_back((*cloud_nonground)[idx]);
        }
      }
    }
    if (!detections.empty()) {
      slice_dbg.mean_cyl_rmse = sum_rmse / static_cast<double>(detections.size());
      slice_dbg.mean_continuity_score = sum_cont / static_cast<double>(detections.size());
    }

    const auto tracks = landmark_tracker_.update(cylinders);
    slice_dbg.n_tracks = tracks.size();

    for (auto & tc : trunk_candidates) {
      float best_d = 1e9f;
      int best_id = -1;
      for (const auto & tr : tracks) {
        const float dx = tc.centroid.x() - tr.cx;
        const float dy = tc.centroid.y() - tr.cy;
        const float d = std::hypot(dx, dy);
        if (d < best_d) {
          best_d = d;
          best_id = tr.id;
          tc.confidence = tr.confidence;
        }
      }
      if (best_id >= 0 && best_d < static_cast<float>(landmark_assoc_xy_m_)) {
        tc.track_id = best_id;
      }
    }

    for (const auto & tr : tracks) {
      TrunkCandidate tc;
      tc.centroid.x() = tr.cx;
      tc.centroid.y() = tr.cy;
      tc.centroid.z() = tr.z_base + 0.5f * tr.height;
      tc.z_base = tr.z_base;
      tc.height = tr.height;
      tc.radius = tr.radius;
      tc.cylinder_rmse = tr.cylinder_rmse;
      tc.verticality = 0.95f;
      tc.track_id = tr.id;
      tc.confidence = tr.confidence;
      tracked_landmarks.push_back(tc);
    }

    for (std::size_t i = 0; i < cloud_nonground->size(); ++i) {
      if (!used[i]) {
        cloud_obstacles->push_back((*cloud_nonground)[i]);
      }
    }

    if (publish_ndsm_debug_ && pub_ndsm_debug_) {
      forest_3d_perception::NdsmStats ndsm_stats;
      const auto band = forest_3d_perception::NdsmField::compute_trunk_band(
        *cloud_nonground, terrain_grid_,
        slice_detector_.params.ndsm_min_m,
        slice_detector_.params.ndsm_max_m,
        &ndsm_stats);
      slice_dbg.mean_ndsm_h = ndsm_stats.mean_h_trunk_band;
      CloudT::Ptr ndsm_cloud(new CloudT);
      ndsm_cloud->reserve(band.size());
      for (const auto & np : band) {
        ndsm_cloud->push_back((*cloud_nonground)[np.index]);
      }
      publish_cloud(pub_ndsm_debug_, ndsm_cloud, stamp);
    }
  }

  void process_trunks_euclidean(
    const CloudT::Ptr & cloud_nonground,
    CloudT::Ptr & cloud_trunks,
    CloudT::Ptr & cloud_obstacles,
    std::vector<TrunkCandidate> & trunk_candidates,
    std::vector<TrunkCandidate> & tracked_landmarks,
    ColumnPipelineDebug & dbg,
    const builtin_interfaces::msg::Time & stamp)
  {
    forest_3d_perception::TrunkPipelineFunnel funnel;
    forest_3d_perception::EuclideanDebugClouds euc_debug;
    forest_3d_perception::EuclideanDebugClouds * euc_debug_ptr =
      publish_slice_pipeline_debug_ ? &euc_debug : nullptr;

    const auto detections = euclidean_detector_.detect(
      *cloud_nonground, terrain_grid_, cell_connected_, &funnel, euc_debug_ptr);

    fill_slice_debug(dbg, funnel);
    dbg.rej_height = 0;
    dbg.rej_radius = 0;
    log_trunk_funnel_throttled(funnel);

    if (publish_slice_pipeline_debug_) {
      if (pub_slice_ndsm_ && euc_debug.ndsm_band && !euc_debug.ndsm_band->empty()) {
        publish_cloud(pub_slice_ndsm_, euc_debug.ndsm_band, stamp);
      }
      if (pub_slice_rejected_ && euc_debug.rejected && !euc_debug.rejected->empty()) {
        publish_cloud(pub_slice_rejected_, euc_debug.rejected, stamp);
      }
      if (pub_slice_accepted_ && euc_debug.accepted && !euc_debug.accepted->empty()) {
        publish_cloud(pub_slice_accepted_, euc_debug.accepted, stamp);
      }
    }

    std::vector<bool> used(cloud_nonground->size(), false);
    std::vector<forest_3d_perception::CylinderObservation> cylinders;
    cylinders.reserve(detections.size());
    double sum_rmse = 0.0;

    for (const auto & det : detections) {
      cylinders.push_back(det.cylinder);
      sum_rmse += static_cast<double>(det.cylinder.rmse);

      TrunkCandidate tc;
      tc.centroid.x() = det.cylinder.cx;
      tc.centroid.y() = det.cylinder.cy;
      tc.centroid.z() = det.cylinder.z_base + 0.5f * det.cylinder.height;
      tc.z_base = det.cylinder.z_base;
      tc.height = det.cylinder.height;
      tc.radius = det.cylinder.radius;
      tc.cylinder_rmse = det.cylinder.rmse;
      tc.verticality = det.verticality;
      tc.confidence = det.score;
      trunk_candidates.push_back(tc);

      for (std::size_t idx : det.point_indices) {
        if (idx < used.size()) {
          used[idx] = true;
          cloud_trunks->push_back((*cloud_nonground)[idx]);
        }
      }
    }
    if (!detections.empty()) {
      dbg.mean_cyl_rmse = sum_rmse / static_cast<double>(detections.size());
      dbg.mean_continuity_score = static_cast<double>(detections.front().score);
    }

    const auto tracks = landmark_tracker_.update(cylinders);
    dbg.n_tracks = tracks.size();

    // Associate detections with tracks
    for (auto & tc : trunk_candidates) {
      float best_d = 1e9f;
      int best_id = -1;
      for (const auto & tr : tracks) {
        const float dx = tc.centroid.x() - tr.cx;
        const float dy = tc.centroid.y() - tr.cy;
        const float d = std::hypot(dx, dy);
        if (d < best_d) {
          best_d = d;
          best_id = tr.id;
          tc.confidence = tr.confidence;
        }
      }
      if (best_id >= 0 && best_d < static_cast<float>(landmark_assoc_xy_m_)) {
        tc.track_id = best_id;
      }
    }

    // Build tracked landmarks (stable output from tracker)
    for (const auto & tr : tracks) {
      TrunkCandidate tc;
      tc.centroid.x() = tr.cx;
      tc.centroid.y() = tr.cy;
      tc.centroid.z() = tr.z_base + 0.5f * tr.height;
      tc.z_base = tr.z_base;
      tc.height = tr.height;
      tc.radius = tr.radius;
      tc.cylinder_rmse = tr.cylinder_rmse;
      tc.verticality = 0.95f;
      tc.track_id = tr.id;
      tc.confidence = tr.confidence;
      tracked_landmarks.push_back(tc);
    }

    // Unmatched nonground → obstacles
    for (std::size_t i = 0; i < cloud_nonground->size(); ++i) {
      if (!used[i]) {
        cloud_obstacles->push_back((*cloud_nonground)[i]);
      }
    }
  }

  void process_trunks_column(
    const CloudT::Ptr & cloud_nonground,
    CloudT::Ptr & cloud_trunks,
    CloudT::Ptr & cloud_obstacles,
    std::vector<TrunkCandidate> & trunk_candidates,
    std::vector<TrunkCandidate> & tracked_landmarks,
    ColumnPipelineDebug & col_dbg,
    const builtin_interfaces::msg::Time & stamp)
  {
    forest_3d_perception::ColumnExtractionStats col_stats;
    const auto detections =
      column_extractor_.extract(*cloud_nonground, terrain_grid_, &col_stats);

    col_dbg.n_band = col_stats.n_band_points;
    col_dbg.n_columns = col_stats.n_columns_accepted;
    col_dbg.rej_sparse = col_stats.reject_sparse;
    col_dbg.rej_cylinder = col_stats.reject_cylinder;
    col_dbg.rej_rmse = col_stats.reject_rmse;
    col_dbg.rej_height = col_stats.reject_height;
    col_dbg.rej_radius = col_stats.reject_radius;

    std::vector<bool> used(cloud_nonground->size(), false);
    std::vector<forest_3d_perception::CylinderObservation> cylinders;
    cylinders.reserve(detections.size());
    double sum_rmse = 0.0;

    for (const auto & det : detections) {
      cylinders.push_back(det.cylinder);
      sum_rmse += static_cast<double>(det.cylinder.rmse);

      TrunkCandidate tc;
      tc.centroid.x() = det.cylinder.cx;
      tc.centroid.y() = det.cylinder.cy;
      tc.centroid.z() = det.cylinder.z_base + 0.5f * det.cylinder.height;
      tc.z_base = det.cylinder.z_base;
      tc.height = det.cylinder.height;
      tc.radius = det.cylinder.radius;
      tc.cylinder_rmse = det.cylinder.rmse;
      tc.verticality = 0.95f;
      trunk_candidates.push_back(tc);

      for (std::size_t idx : det.point_indices) {
        if (idx < used.size()) {
          used[idx] = true;
          cloud_trunks->push_back((*cloud_nonground)[idx]);
        }
      }
    }
    if (!detections.empty()) {
      col_dbg.mean_cyl_rmse = sum_rmse / static_cast<double>(detections.size());
    }

    const auto tracks = landmark_tracker_.update(cylinders);
    col_dbg.n_tracks = tracks.size();

    for (auto & tc : trunk_candidates) {
      float best_d = 1e9f;
      int best_id = -1;
      for (const auto & tr : tracks) {
        const float dx = tc.centroid.x() - tr.cx;
        const float dy = tc.centroid.y() - tr.cy;
        const float d = std::hypot(dx, dy);
        if (d < best_d) {
          best_d = d;
          best_id = tr.id;
          tc.confidence = tr.confidence;
        }
      }
      if (best_id >= 0 && best_d < static_cast<float>(landmark_assoc_xy_m_)) {
        tc.track_id = best_id;
      }
    }

    for (const auto & tr : tracks) {
      TrunkCandidate tc;
      tc.centroid.x() = tr.cx;
      tc.centroid.y() = tr.cy;
      tc.centroid.z() = tr.z_base + 0.5f * tr.height;
      tc.z_base = tr.z_base;
      tc.height = tr.height;
      tc.radius = tr.radius;
      tc.cylinder_rmse = tr.cylinder_rmse;
      tc.verticality = 0.95f;
      tc.track_id = tr.id;
      tc.confidence = tr.confidence;
      tracked_landmarks.push_back(tc);
    }

    for (std::size_t i = 0; i < cloud_nonground->size(); ++i) {
      if (!used[i]) {
        cloud_obstacles->push_back((*cloud_nonground)[i]);
      }
    }

    if (publish_ndsm_debug_ && pub_ndsm_debug_) {
      forest_3d_perception::NdsmStats ndsm_stats;
      const auto band = forest_3d_perception::NdsmField::compute_trunk_band(
        *cloud_nonground, terrain_grid_,
        column_extractor_.params.ndsm_min_m,
        column_extractor_.params.ndsm_max_m,
        &ndsm_stats);
      col_dbg.mean_ndsm_h = ndsm_stats.mean_h_trunk_band;

      CloudT::Ptr ndsm_cloud(new CloudT);
      ndsm_cloud->reserve(band.size());
      for (const auto & np : band) {
        ndsm_cloud->push_back((*cloud_nonground)[np.index]);
      }
      publish_cloud(pub_ndsm_debug_, ndsm_cloud, stamp);
    }
  }

  void crop_cloud(const CloudT::Ptr & in, CloudT::Ptr & out) const
  {
    const float r2_min = min_range_ * min_range_;
    const float r2_max = max_range_ * max_range_;
    out->reserve(in->size());
    for (const auto & p : *in) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      float r2 = p.x * p.x + p.y * p.y;
      if (r2 < r2_min || r2 > r2_max) {
        continue;
      }
      if (p.z < min_z_ || p.z > max_z_) {
        continue;
      }
      out->push_back(p);
    }
  }

  /** @return true if plane fit was rejected (centroid Z band). */
  bool segment_ground(
    const CloudT::Ptr & cloud,
    pcl::PointIndices::Ptr & inliers,
    pcl::ModelCoefficients::Ptr & coeffs) const
  {
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_dist_);
    seg.setMaxIterations(ground_max_iter_);

    // Constrain normal to near-vertical (ground plane)
    const double max_slope_rad = ground_max_slope_ * M_PI / 180.0;
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setEpsAngle(max_slope_rad);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeffs);

    if (inliers->indices.empty()) {
      return false;
    }

    // Post-filter: reject plane if centroid Z is too far from expected ground
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, inliers->indices, centroid);
    if (std::abs(centroid.z()) > ground_z_band_) {
      RCLCPP_DEBUG(get_logger(), "Ground plane rejected: centroid z=%.2f outside band", centroid.z());
      inliers->indices.clear();
      return true;
    }
    return false;
  }

  TrunkRejectReason classify_cluster(const CloudT::Ptr & cluster, TrunkCandidate & tc) const
  {
    if (cluster->size() < 5) {
      return TrunkRejectReason::TooFewPoints;
    }

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    tc.height = max_pt.z - min_pt.z;

    if (tc.height < trunk_min_height_) {
      return TrunkRejectReason::TooShort;
    }

    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*cluster, centroid4);
    tc.centroid = centroid4.head<3>();

    double sum_r2 = 0.0;
    for (const auto & p : *cluster) {
      double dx = p.x - tc.centroid.x();
      double dy = p.y - tc.centroid.y();
      sum_r2 += dx * dx + dy * dy;
    }
    tc.radius = std::sqrt(sum_r2 / cluster->size());

    if (tc.radius > trunk_max_radius_) {
      return TrunkRejectReason::TooWide;
    }

    const float dx = max_pt.x - min_pt.x;
    const float dy = max_pt.y - min_pt.y;
    const float dz = max_pt.z - min_pt.z;
    const float diag = std::sqrt(dx * dx + dy * dy + dz * dz);
    tc.verticality = (diag > 0.01f) ? (dz / diag) : 0.0f;

    if (tc.verticality < trunk_min_vert_) {
      return TrunkRejectReason::NotVertical;
    }
    return TrunkRejectReason::Accepted;
  }

  void publish_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const CloudT::Ptr & cloud, const builtin_interfaces::msg::Time & stamp) const
  {
    if (!always_publish_clouds_ && pub->get_subscription_count() == 0) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = processing_frame_;
    msg.header.stamp = stamp;
    pub->publish(msg);
  }

  void publish_intensity_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const builtin_interfaces::msg::Time & stamp) const
  {
    if (!pub || (!always_publish_clouds_ && pub->get_subscription_count() == 0)) {
      return;
    }
    if (!cloud || cloud->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = processing_frame_;
    msg.header.stamp = stamp;
    pub->publish(msg);
  }

  void publish_ground_reference_debug(
    const CloudT::Ptr & cloud_nonground,
    ColumnPipelineDebug & dbg,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_ground_ref_debug_ || ground_method_ != "grid" || !cloud_nonground) {
      return;
    }

    dbg.ground_ref_grid = forest_3d_perception::compute_grid_stats(terrain_grid_);
    forest_3d_perception::GroundRefFrameStats stats;
    const float hmin = static_cast<float>(ndsm_trunk_min_m_);
    const float hmax = static_cast<float>(ndsm_trunk_max_m_);
    for (const auto & p : cloud_nonground->points) {
      const auto d = forest_3d_perception::diagnose_point(terrain_grid_, p.x, p.y, p.z);
      forest_3d_perception::accumulate_ground_ref_stats(&stats, d, hmin, hmax);
    }
    forest_3d_perception::finalize_ground_ref_stats(&stats);
    dbg.ground_ref = stats;

    if (pub_ground_ref_h_) {
      publish_intensity_cloud(
        pub_ground_ref_h_,
        forest_3d_perception::make_height_cloud(*cloud_nonground, terrain_grid_),
        stamp);
    }
    if (pub_ground_ref_zg_) {
      publish_intensity_cloud(
        pub_ground_ref_zg_,
        forest_3d_perception::make_zg_cloud(*cloud_nonground, terrain_grid_),
        stamp);
    }
    if (pub_ground_ref_no_obs_) {
      publish_intensity_cloud(
        pub_ground_ref_no_obs_,
        forest_3d_perception::make_mesh_no_obs_cells_cloud(terrain_grid_),
        stamp);
    }
    if (pub_ground_ref_confidence_) {
      publish_intensity_cloud(
        pub_ground_ref_confidence_,
        forest_3d_perception::make_mesh_confidence_cells_cloud(terrain_grid_),
        stamp);
    }
  }

  void publish_trunk_markers(
    const std::vector<TrunkCandidate> & trunks,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (pub_markers_->get_subscription_count() == 0) {
      return;
    }

    visualization_msgs::msg::MarkerArray ma;

    // Delete previous markers
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.header.frame_id = processing_frame_;
    del.header.stamp = stamp;
    ma.markers.push_back(del);

    for (std::size_t i = 0; i < trunks.size(); ++i) {
      const auto & t = trunks[i];

      visualization_msgs::msg::Marker m;
      m.header.frame_id = processing_frame_;
      m.header.stamp = stamp;
      m.ns = "trunks";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = t.centroid.x();
      m.pose.position.y = t.centroid.y();
      const float z_center =
        (t.z_base > -900.0f) ? (t.z_base + 0.5f * t.height) : t.centroid.z();
      m.pose.position.z = z_center;
      m.scale.x = std::max(0.08f, t.radius * 2.0f);
      m.scale.y = std::max(0.08f, t.radius * 2.0f);
      m.scale.z = std::max(0.15f, t.height);
      m.color.r = 0.55f;
      m.color.g = 0.27f;
      m.color.b = 0.07f;
      m.color.a = 0.7f;
      m.lifetime = rclcpp::Duration::from_seconds(t.track_id >= 0 ? 4.0 : 1.5);
      ma.markers.push_back(m);
    }

    pub_markers_->publish(ma);
  }

  void publish_tracked_landmark_markers(
    const std::vector<TrunkCandidate> & tracks,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (pub_tree_landmarks_->get_subscription_count() == 0) {
      return;
    }

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.header.frame_id = processing_frame_;
    del.header.stamp = stamp;
    ma.markers.push_back(del);

    for (const auto & t : tracks) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = processing_frame_;
      m.header.stamp = stamp;
      m.ns = "tree_landmarks";
      m.id = t.track_id;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = t.centroid.x();
      m.pose.position.y = t.centroid.y();
      m.pose.position.z = t.z_base + 0.5f * t.height;
      m.scale.x = std::max(0.08f, t.radius * 2.0f);
      m.scale.y = std::max(0.08f, t.radius * 2.0f);
      m.scale.z = std::max(0.15f, t.height);
      const float conf = std::clamp(t.confidence, 0.2f, 1.0f);
      m.color.r = 0.45f + 0.2f * conf;
      m.color.g = 0.20f + 0.15f * conf;
      m.color.b = 0.05f;
      m.color.a = 0.55f + 0.35f * conf;
      m.lifetime = rclcpp::Duration::from_seconds(5.0);
      ma.markers.push_back(m);
    }

    pub_tree_landmarks_->publish(ma);
  }

  /** Denser nonground for slice trunks; ground grid unchanged (built from main voxel). */
  CloudT::Ptr build_trunk_nonground_cloud(
    const CloudT::Ptr & cloud_cropped,
    const CloudT::Ptr & cloud_nonground_default) const
  {
    if (trunk_voxel_leaf_m_ <= 0.0 || trunk_voxel_leaf_m_ >= voxel_leaf_ - 1e-6) {
      return cloud_nonground_default;
    }
    CloudT::Ptr cloud_trunk_ds(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_cropped);
    vg.setLeafSize(
      static_cast<float>(trunk_voxel_leaf_m_),
      static_cast<float>(trunk_voxel_leaf_m_),
      static_cast<float>(trunk_voxel_leaf_m_));
    vg.filter(*cloud_trunk_ds);
    if (cloud_trunk_ds->size() < 30) {
      return cloud_nonground_default;
    }
    CloudT::Ptr trunk_nonground(new CloudT);
    if (ground_method_ == "grid" && ground_connectivity_enable_) {
      const auto seg = ground_connectivity_.segment(terrain_grid_, *cloud_trunk_ds);
      trunk_nonground = seg.nonground;
      if (!seg.ground_suspended->empty()) {
        *trunk_nonground += *seg.ground_suspended;
      }
    } else if (ground_method_ == "grid") {
      const auto seg = terrain_grid_.segment_cloud(*cloud_trunk_ds);
      trunk_nonground = seg.nonground;
    } else {
      return cloud_nonground_default;
    }
    return trunk_nonground->size() >= cloud_nonground_default->size() / 4 ?
      trunk_nonground : cloud_nonground_default;
  }

  void publish_terrain_mesh(const builtin_interfaces::msg::Time & stamp)
  {
    if (!pub_terrain_mesh_) {
      return;
    }

    // Delete previous mesh to avoid "ghost" effect when robot moves
    visualization_msgs::msg::Marker del;
    del.header.frame_id = processing_frame_;
    del.header.stamp = stamp;
    del.ns = "terrain_ground_mesh";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETE;
    pub_terrain_mesh_->publish(del);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = processing_frame_;
    m.header.stamp = stamp;
    m.ns = "terrain_ground_mesh";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0.2f;
    m.color.g = 0.65f;
    m.color.b = 0.25f;
    m.color.a = 0.55f;
    m.lifetime = rclcpp::Duration::from_seconds(0.25);

    const float half_x = static_cast<float>(terrain_grid_.size_x_m * 0.5f);
    const float half_y = static_cast<float>(terrain_grid_.size_y_m * 0.5f);
    const float res = static_cast<float>(terrain_grid_.resolution_m);

    auto cell_corner = [&](std::size_t ix, std::size_t iy, float & x, float & y, float & z) {
      x = static_cast<float>(ix) * res - half_x;
      y = static_cast<float>(iy) * res - half_y;
      z = terrain_grid_.height_for_mesh(ix, iy);
    };

    for (std::size_t iy = 0; iy + 1 < terrain_grid_.height; ++iy) {
      for (std::size_t ix = 0; ix + 1 < terrain_grid_.width; ++ix) {
        float x00, y00, z00, x10, y10, z10, x01, y01, z01, x11, y11, z11;
        cell_corner(ix, iy, x00, y00, z00);
        cell_corner(ix + 1, iy, x10, y10, z10);
        cell_corner(ix, iy + 1, x01, y01, z01);
        cell_corner(ix + 1, iy + 1, x11, y11, z11);
        if (std::isnan(z00) || std::isnan(z10) || std::isnan(z01) || std::isnan(z11)) {
          continue;
        }
        geometry_msgs::msg::Point p00, p10, p01, p11;
        p00.x = x00;
        p00.y = y00;
        p00.z = z00;
        p10.x = x10;
        p10.y = y10;
        p10.z = z10;
        p01.x = x01;
        p01.y = y01;
        p01.z = z01;
        p11.x = x11;
        p11.y = y11;
        p11.z = z11;
        m.points.push_back(p00);
        m.points.push_back(p10);
        m.points.push_back(p01);
        m.points.push_back(p10);
        m.points.push_back(p11);
        m.points.push_back(p01);
      }
    }

    pub_terrain_mesh_->publish(m);
  }

  // Parameters
  std::string input_topic_;
  std::string processing_frame_;
  double voxel_leaf_{};
  double trunk_voxel_leaf_m_{0.0};
  double min_range_{};
  double max_range_{};
  double min_z_{};
  double max_z_{};
  double ground_dist_{};
  int ground_max_iter_{};
  double ground_max_slope_{};
  double ground_z_band_{};
  std::string ground_method_;
  double grid_size_x_m_{};
  double grid_size_y_m_{};
  double grid_resolution_m_{};
  double grid_ground_height_thresh_m_{};
  double grid_hole_depth_m_{};
  int grid_inpaint_passes_{};
  double grid_height_percentile_{};
  double grid_smooth_max_step_m_{};
  int grid_smooth_clamp_passes_{};
  int grid_smooth_median_radius_cells_{};
  int grid_ground_neighbor_cells_{};
  std::string terrain_mesh_topic_;
  bool publish_terrain_mesh_{true};
  bool ground_connectivity_enable_{true};
  double connectivity_max_step_m_{};
  double connectivity_seed_radius_m_{};
  bool publish_suspended_debug_{true};
  std::string suspended_topic_;
  forest_3d_perception::TerrainGrid2D terrain_grid_;
  forest_3d_perception::GroundConnectivity ground_connectivity_;
  double cluster_tol_{};
  int min_cluster_{};
  int max_cluster_{};
  double trunk_min_height_{};
  double trunk_max_radius_{};
  double trunk_min_vert_{};
  std::string trunk_method_;
  double ndsm_trunk_min_m_{};
  double ndsm_trunk_max_m_{};
  int column_min_points_per_cell_{};
  int column_min_cells_{};
  int column_min_points_{};
  int column_max_points_{};
  int column_max_columns_{};
  double cylinder_min_height_m_{};
  double cylinder_max_radius_m_{};
  double cylinder_max_rmse_m_{};
  double cylinder_min_inlier_ratio_{};
  double cylinder_inlier_dist_m_{};
  double cylinder_max_slice_height_m_{};
  double slice_height_m_{};
  int slice_max_count_{};
  double slice_cluster_cell_m_{};
  int slice_min_points_per_cluster_{};
  int slice_min_slices_for_trunk_{};
  double slice_assoc_max_xy_m_{};
  int slice_assoc_max_gap_slices_{2};
  double slice_min_continuity_score_{};
  double slice_min_vertical_persistence_{};
  double slice_max_centroid_drift_{};
  double slice_max_radius_cv_{};
  double slice_ground_anchor_max_dz_m_{};
  int slice_max_stems_per_frame_{};
  double landmark_assoc_xy_m_{};
  int landmark_max_misses_{};
  bool publish_ndsm_debug_{false};
  bool publish_slice_pipeline_debug_{false};
  std::string slice_debug_ndsm_topic_;
  std::string slice_debug_clusters_topic_;
  std::string slice_debug_rejected_topic_;
  std::string slice_debug_accepted_topic_;
  std::string ndsm_debug_topic_;
  std::string tree_landmarks_topic_;
  forest_3d_perception::TrunkColumnExtractor column_extractor_;
  forest_3d_perception::TrunkSliceDetector slice_detector_;
  forest_3d_perception::TrunkDetectorEuclidean euclidean_detector_;
  forest_3d_perception::LandmarkTracker landmark_tracker_;
  std::vector<uint8_t> cell_connected_;
  std::string ground_topic_;
  std::string trunks_topic_;
  std::string obstacles_topic_;
  std::string markers_topic_;
  double tf_timeout_{};
  bool publish_debug_stats_{true};
  std::string debug_stats_topic_;
  bool publish_ground_ref_debug_{false};
  std::string ground_ref_h_topic_;
  std::string ground_ref_zg_topic_;
  std::string ground_ref_no_obs_cells_topic_;
  std::string ground_ref_confidence_cells_topic_;
  bool always_publish_clouds_{true};
  int pipeline_log_interval_{25};
  mutable std::size_t pipeline_frame_count_{0};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_suspended_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_trunks_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tree_landmarks_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ndsm_debug_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_slice_ndsm_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_slice_clusters_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_slice_rejected_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_slice_accepted_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_terrain_mesh_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_ref_h_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_ref_zg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_ref_no_obs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_ref_confidence_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar3dSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}

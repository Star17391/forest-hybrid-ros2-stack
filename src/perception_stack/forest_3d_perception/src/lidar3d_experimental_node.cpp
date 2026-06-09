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
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "forest_hybrid_msgs/msg/tree_landmark_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "forest_3d_perception/experimental/cluster_classifier.hpp"
#include "forest_3d_perception/experimental/experimental_pipeline.hpp"
#include "forest_3d_perception/experimental/object_on_ground_filter.hpp"
#include "forest_3d_perception/experimental/pipeline_debug.hpp"
#include "forest_3d_perception/experimental/stem_band_clustering.hpp"
#include "forest_3d_perception/experimental/stem_point_filter.hpp"
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

forest_3d_perception::experimental::StemPointFilterParams load_stem_filter_params(
  rclcpp::Node & node)
{
  forest_3d_perception::experimental::StemPointFilterParams p;
  p.enabled = node.get_parameter("stem_filter.enabled").as_bool();
  p.neighbor_radius_m =
    static_cast<float>(node.get_parameter("stem_filter.neighbor_radius_m").as_double());
  p.min_neighbors = node.get_parameter("stem_filter.min_neighbors").as_int();
  p.linearity_min =
    static_cast<float>(node.get_parameter("stem_filter.linearity_min").as_double());
  p.verticality_floor =
    static_cast<float>(node.get_parameter("stem_filter.verticality_floor").as_double());
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
  p.rock_max_height_m =
    static_cast<float>(node.get_parameter("classify.rock_max_height_m").as_double());
  p.rock_max_aspect =
    static_cast<float>(node.get_parameter("classify.rock_max_aspect").as_double());
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
    pipeline_.apply_params();
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

    if (debug_publish_stages_) {
      pub_dbg_voxel_ = create_publisher<sensor_msgs::msg::PointCloud2>(dbg_voxel_topic_, qos);
      pub_dbg_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(dbg_ground_topic_, qos);
      pub_dbg_non_ground_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(dbg_non_ground_topic_, qos);
      pub_dbg_clusters_ = create_publisher<sensor_msgs::msg::PointCloud2>(dbg_clusters_topic_, qos);
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

    // Sprint 3.5 — Option 1: per-point linearity split (OFF by default; kept for
    // comparison). Fragments trees, so region growing above supersedes it.
    declare_parameter<bool>("stem_filter.enabled", true);
    declare_parameter<double>("stem_filter.neighbor_radius_m", 0.25);
    declare_parameter<int>("stem_filter.min_neighbors", 5);
    declare_parameter<double>("stem_filter.linearity_min", 0.60);
    declare_parameter<double>("stem_filter.verticality_floor", 0.30);

    // Sprint 3 (Option B): slice-based classification — TRUNK / ROCK / SHRUB /
    // OBSTACLE. Trunk = vertical stem core (shape, not width); OBSTACLE is the
    // only catch-all.
    declare_parameter<double>("classify.slice_height_m", 0.20);
    declare_parameter<int>("classify.slice_min_pts", 2);
    declare_parameter<double>("classify.trunk_radius_grow_factor", 2.2);
    declare_parameter<double>("classify.trunk_radius_abs_margin_m", 0.10);
    declare_parameter<double>("classify.trunk_center_jump_m", 0.30);
    declare_parameter<double>("classify.trunk_core_min_height_m", 0.80);
    declare_parameter<double>("classify.rock_max_height_m", 0.60);
    declare_parameter<double>("classify.rock_max_aspect", 1.20);
    declare_parameter<double>("classify.shrub_max_height_m", 1.50);
    declare_parameter<int>("classify.min_points", 4);

    reload_pipeline_params();
  }

  void reload_pipeline_params()
  {
    pipeline_.params.sprint =
      forest_3d_perception::experimental::PipelineSprint::GroundClustering;
    pipeline_.params.sprint1.csf = load_csf_params(*this);
    pipeline_.params.sprint1.clustering = load_clustering_params(*this);
    pipeline_.apply_params();
    stem_clusterer_.params = load_stem_params(*this);
    object_filter_.params = load_object_filter_params(*this);
    stem_filter_.params = load_stem_filter_params(*this);
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
          n.rfind("stem.", 0) == 0 || n.rfind("stem_filter.", 0) == 0 ||
          n.rfind("region_grow.", 0) == 0 ||
          n.rfind("csf_post.", 0) == 0 || n.rfind("classify.", 0) == 0)
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
    const std::vector<forest_3d_perception::experimental::ClassifiedCluster> & labels,
    const rclcpp::Time & stamp,
    const std::string & frame_id) const
  {
    visualization_msgs::msg::MarkerArray arr;
    // Clear stale markers from the previous frame before redrawing.
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
      const auto & lab = labels[i];
      const float cx = lab.feat.centroid_x;
      const float cy = lab.feat.centroid_y;
      const float cz = lab.feat.centroid_z;

      float r;
      float g;
      float b;
      class_color(lab.cls, r, g, b);

      // Sphere at the cluster centroid, colored by class.
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

      // Text label above the cluster: class name + confidence.
      visualization_msgs::msg::Marker t;
      t.header.stamp = stamp;
      t.header.frame_id = frame_id;
      t.ns = "experimental_class_labels";
      t.id = mid++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose.position.x = cx;
      t.pose.position.y = cy;
      t.pose.position.z = cz + 0.5f * lab.feat.height_span + 0.3f;
      t.scale.z = 0.3;
      t.color.r = r;
      t.color.g = g;
      t.color.b = b;
      t.color.a = 1.0f;
      char buf[48];
      std::snprintf(
        buf, sizeof(buf), "%s %.0f%%",
        forest_3d_perception::experimental::cluster_class_string(lab.cls),
        lab.confidence * 100.0f);
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
      "band=%zu clusters=%zu csf_ground%%=%.1f in_frame=%s",
      forest_3d_perception::experimental::exit_status_string(f.status),
      f.debug_stage, f.n_raw, f.n_crop, f.n_voxel, f.n_ground, f.n_non_ground,
      n_band_last_, f.n_clusters, f.csf_ground_pct, f.input_frame.c_str());
    if (f.status != forest_3d_perception::experimental::PipelineExitStatus::Ok) {
      RCLCPP_WARN(
        get_logger(), "Pipeline exit before full publish: %s",
        forest_3d_perception::experimental::exit_status_string(f.status));
    }
  }

  void publish_labeled_clusters(
    const CloudT::Ptr & non_ground,
    const std::vector<forest_3d_perception::experimental::PointCluster> & clusters,
    const std::vector<forest_3d_perception::experimental::ClassifiedCluster> & labels,
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
      publish_cluster_markers(clusters, labels, stamp, processing_frame_);
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
    const std::vector<forest_3d_perception::experimental::ClassifiedCluster> & labels,
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
        const float id = semantic_id(labels[i].cls);
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

  /**
   * Layer-contract output #2: per-frame tree landmarks (TRUNK clusters only) as
   * parametric primitives — base (x,y,z), diameter, height, confidence. Consumed
   * by the map/SLAM layer (which does tracking, fusion and confidence). Stateless.
   */
  void publish_tree_landmarks(
    const std::vector<forest_3d_perception::experimental::PointCluster> & clusters,
    const std::vector<forest_3d_perception::experimental::ClassifiedCluster> & labels,
    const rclcpp::Time & stamp) const
  {
    if (!pub_tree_landmarks_) {
      return;
    }
    forest_hybrid_msgs::msg::TreeLandmarkArray arr;
    arr.header.stamp = stamp;
    arr.header.frame_id = processing_frame_;
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      if (labels[i].cls != forest_3d_perception::experimental::ClusterClass::Trunk) {
        continue;
      }
      const auto & f = labels[i].feat;
      float zmin = std::numeric_limits<float>::max();
      if (clusters[i].cloud) {
        for (const auto & p : clusters[i].cloud->points) {
          zmin = std::min(zmin, p.z);
        }
      }
      forest_hybrid_msgs::msg::TreeLandmark t;
      t.base.x = f.centroid_x;
      t.base.y = f.centroid_y;
      t.base.z = (zmin == std::numeric_limits<float>::max()) ? f.centroid_z : zmin;
      t.diameter = 2.0f * f.trunk_ref_radius;
      t.height = f.height_span;
      t.confidence = labels[i].confidence;
      arr.trees.push_back(t);
    }
    pub_tree_landmarks_->publish(arr);
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
      auto r = pipeline_.sprint1.clusterer.cluster(*non_ground);
      clusters = std::move(r.clusters);
      cluster_ref_cloud = non_ground;
    } else {
      /* Stage 2 or full: CSF */
      const auto csf_out = pipeline_.sprint1.csf_segmenter.segment(*voxel);
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
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "REGION_GROW working=%zu seeds=%zu grown=%zu clusters=%zu "
            "(seed_max_hag=%.2f radius=%.2f z_scale=%.2f)",
            rg.n_working, rg.n_seeds, rg.n_grown, rg.clusters.size(),
            region_grower_.params.seed_max_hag_m, region_grower_.params.growth_radius_m,
            region_grower_.params.growth_z_scale);
          clusters = std::move(rg.clusters);
        } else {
          /* Sprint 3.5 — Option 1 (off by default): band + linearity SUBTRACTION.
             band → split into set A (linear/trunks) and set B (rest: shrubs+rocks);
             cluster each disjoint set. Kept for comparison; fragments trees. */
          const auto band = stem_clusterer_.extract_band(*ground, *non_ground);
          band_cloud = band.cloud;
          n_band_last_ = band.indices.size();

          const auto mask = stem_filter_.linear_mask(*band.cloud);
          std::vector<std::size_t> set_trunk;
          std::vector<std::size_t> set_rest;
          set_trunk.reserve(band.indices.size());
          set_rest.reserve(band.indices.size());
          for (std::size_t k = 0; k < band.indices.size(); ++k) {
            if (k < mask.size() && mask[k]) {
              set_trunk.push_back(band.indices[k]);
            } else {
              set_rest.push_back(band.indices[k]);
            }
          }

          auto clusters_trunk = stem_clusterer_.cluster_band_subset(*non_ground, set_trunk, 0);
          auto clusters_rest = stem_clusterer_.cluster_band_subset(
            *non_ground, set_rest, static_cast<int>(clusters_trunk.size()));

          stem_candidates->reserve(set_trunk.size());
          for (std::size_t oi : set_trunk) {
            stem_candidates->push_back(non_ground->points[oi]);
          }
          stem_candidates->width = static_cast<std::uint32_t>(stem_candidates->size());
          stem_candidates->height = 1;
          stem_candidates->is_dense = true;

          clusters = std::move(clusters_trunk);
          clusters.insert(
            clusters.end(),
            std::make_move_iterator(clusters_rest.begin()),
            std::make_move_iterator(clusters_rest.end()));
        }
      }
      // Cluster indices reference the non-ground cloud in both paths.
      cluster_ref_cloud = non_ground;
    }

    funnel.n_clusters = clusters.size();
    funnel.cluster_summaries =
      forest_3d_perception::experimental::summarize_clusters(clusters);

    // Sprint 3: geometric classification (trunk / rock / shrub) per cluster.
    const auto labels = classifier_.classify_all(clusters);

    // Debug: dump per-cluster features so misclassifications can be traced
    // (e.g. trunks coming out short -> ROCK means region growing fragmented them).
    if (debug_log_interval_ > 0 && !labels.empty()) {
      std::size_t n_trunk = 0;
      std::size_t n_rock = 0;
      std::size_t n_shrub = 0;
      std::size_t n_obstacle = 0;
      for (const auto & l : labels) {
        using CC = forest_3d_perception::experimental::ClusterClass;
        if (l.cls == CC::Trunk) { ++n_trunk; }
        else if (l.cls == CC::Rock) { ++n_rock; }
        else if (l.cls == CC::Shrub) { ++n_shrub; }
        else if (l.cls == CC::Obstacle) { ++n_obstacle; }
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CLASSIFY n=%zu trunk=%zu rock=%zu shrub=%zu obstacle=%zu",
        labels.size(), n_trunk, n_rock, n_shrub, n_obstacle);
      const std::size_t dump = std::min<std::size_t>(labels.size(), 6);
      for (std::size_t i = 0; i < dump; ++i) {
        const auto & f = labels[i].feat;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "  cl[%zu] %s n=%d h_span=%.2f horiz=%.2f core=%.2f ref_r=%.2f",
          i, forest_3d_perception::experimental::cluster_class_string(labels[i].cls),
          f.n_points, f.height_span, f.horizontal_size, f.trunk_core_height, f.trunk_ref_radius);
      }
    }

    publish_cloud(pub_ground_, ground, stamp, processing_frame_);
    publish_cloud(pub_non_ground_, non_ground, stamp, processing_frame_);
    publish_cloud(pub_stem_candidates_, stem_candidates, stamp, processing_frame_);
    publish_cloud(pub_stem_band_, band_cloud, stamp, processing_frame_);
    publish_labeled_clusters(
      cluster_ref_cloud ? cluster_ref_cloud : non_ground, clusters, labels, stamp);

    // Layer-contract outputs for the map/SLAM layer (base_link, stateless).
    publish_semantic_points(
      ground, cluster_ref_cloud ? cluster_ref_cloud : non_ground, clusters, labels, stamp);
    publish_tree_landmarks(clusters, labels, stamp);

    funnel.status = PS::Ok;
    publish_funnel(funnel);
    log_funnel_throttled(funnel);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  forest_3d_perception::experimental::ExperimentalPipeline pipeline_;
  forest_3d_perception::experimental::StemBandClusterer stem_clusterer_;
  forest_3d_perception::experimental::ObjectOnGroundFilter object_filter_;
  forest_3d_perception::experimental::StemPointFilter stem_filter_;
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

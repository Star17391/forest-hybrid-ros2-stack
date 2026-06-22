#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "forest_tree_slam/tracker.hpp"

using forest_tree_slam::LandmarkTracker;
using forest_tree_slam::TrackerParams;
using forest_tree_slam::TreeDetection;

namespace
{
std::vector<Eigen::Vector3d> trunk_arc_points(
  double cx, double cy, double r, double z0, double z1, int n_per_ring)
{
  std::vector<Eigen::Vector3d> pts;
  const int rings = 8;
  for (int ring = 0; ring < rings; ++ring) {
    const double z = z0 + (z1 - z0) * static_cast<double>(ring) / static_cast<double>(rings - 1);
    for (int i = 0; i < n_per_ring; ++i) {
      const double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n_per_ring);
      pts.emplace_back(cx + r * std::cos(t), cy + r * std::sin(t), z);
    }
  }
  return pts;
}

TreeDetection ambiguous_trunk_detection()
{
  TreeDetection d;
  d.x = 5.0;
  d.y = 0.0;
  d.diameter = 0.40;
  d.confidence = 0.9F;
  d.class_scores = {0.40F, 0.35F, 0.25F};
  d.has_stem_inliers = true;
  return d;
}
}  // namespace

TEST(MultiviewClassScore, AggregatedArcBoostsTrunkPosterior)
{
  TrackerParams params;
  params.promote_min_obs = 1;
  params.multiview.refit_min_points = 12;
  params.multiview.coverage_bins = 8;
  params.multiview_class_min_coverage = 0.20;
  params.multiview_class_coverage_step = 0.10;
  // Isola o mecanismo de acumulação de CLASSE: desliga o gate (D) de pose
  // corrigida (que noutro contexto também gateia esta via — ver tracker.cpp).
  params.multiview_gate_min_obs = 0;
  params.multiview_gate_max_pos_var = 100.0;
  LandmarkTracker tracker(params);

  const TreeDetection det = ambiguous_trunk_detection();
  const auto report = tracker.update({det}, 0.0, Eigen::Vector2d::Zero());
  ASSERT_EQ(report.births.size(), 1u);
  const auto uid = report.births.front();

  const Eigen::Vector3d posterior_before =
    LandmarkTracker::class_posterior(tracker.tracks().front());

  const std::vector<Eigen::Vector3d> arc = trunk_arc_points(5.0, 0.0, 0.20, 0.5, 2.0, 16);
  const std::vector<Eigen::Vector2d> views = {
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(0.0, -4.0),
    Eigen::Vector2d(-4.0, 0.0),
    Eigen::Vector2d(0.0, 4.0),
  };
  for (const auto & view : views) {
    tracker.ingest_multiview_inliers(uid, arc, view, det);
  }

  const Eigen::Vector3d posterior_after =
    LandmarkTracker::class_posterior(tracker.tracks().front());

  EXPECT_GT(posterior_after[0], posterior_before[0]);
  EXPECT_GT(posterior_after[0], posterior_after[1]);
}

TEST(MultiviewClassScore, RockDetectionDoesNotAccumulateMultiview)
{
  TrackerParams params;
  params.multiview.refit_min_points = 8;
  LandmarkTracker tracker(params);

  TreeDetection rock;
  rock.x = 4.0;
  rock.y = 1.0;
  rock.diameter = 0.8;
  rock.confidence = 0.9F;
  rock.class_scores = {0.10F, 0.80F, 0.10F};
  rock.has_stem_inliers = true;

  const auto report = tracker.update({rock}, 0.0, Eigen::Vector2d::Zero());
  ASSERT_EQ(report.births.size(), 1u);
  const auto uid = report.births.front();
  const double cov_before = tracker.tracks().front().last_multiview_class_coverage_;

  const std::vector<Eigen::Vector3d> arc = trunk_arc_points(4.0, 1.0, 0.20, 0.2, 0.5, 12);
  tracker.ingest_multiview_inliers(uid, arc, Eigen::Vector2d::Zero(), rock);

  EXPECT_EQ(tracker.tracks().front().last_multiview_class_coverage_, cov_before);
  EXPECT_EQ(tracker.tracks().front().multiview_buffer.n_voxels(), 0u);
}

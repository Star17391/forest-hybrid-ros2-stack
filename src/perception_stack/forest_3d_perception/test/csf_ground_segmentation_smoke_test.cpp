/**
 * Offline regression for the experimental CSF ground segmenter.
 *
 * Root-cause guard: Gazebo LiDAR emits non-finite (NaN/Inf) returns for rays
 * that hit nothing. Those must never reach CSF — `int(NaN/step)` is INT_MIN at
 * runtime, so CSF's `getParticle()` reads far out of bounds and the process
 * dies with SIGSEGV (exit -11) right after "post handle".
 *
 * This test feeds clouds that DO contain NaN/Inf and asserts the segmenter
 * stays alive and excludes the garbage points.
 */

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/experimental/csf_ground_segmentation.hpp"

namespace
{

pcl::PointCloud<pcl::PointXYZ> make_robot_like_cloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // Ground patch (z ~ 0)
  for (float x = -8.0f; x <= 8.0f; x += 0.4f) {
    for (float y = -8.0f; y <= 8.0f; y += 0.4f) {
      const float r = std::sqrt(x * x + y * y);
      if (r < 0.35f || r > 14.0f) {
        continue;
      }
      cloud.push_back(pcl::PointXYZ{x, y, 0.02f * ((x + y) * 0.01f)});
    }
  }
  // Trunks / clutter above ground
  for (float x = -6.0f; x <= 6.0f; x += 1.2f) {
    for (float y = -6.0f; y <= 6.0f; y += 1.2f) {
      if (std::sqrt(x * x + y * y) < 1.5f) {
        continue;
      }
      for (float z = 0.5f; z <= 3.5f; z += 0.25f) {
        cloud.push_back(pcl::PointXYZ{x + 0.05f, y - 0.03f, z});
      }
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ> make_sim_like_cloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float x = -14.5f; x <= 14.5f; x += 0.35f) {
    for (float y = -14.5f; y <= 14.5f; y += 0.35f) {
      const float r = std::sqrt(x * x + y * y);
      if (r < 0.35f || r > 14.5f) {
        continue;
      }
      cloud.push_back(pcl::PointXYZ{x, y, 0.05f * std::sin(x * 0.2f)});
      if (r > 2.0f && (static_cast<int>(x * 10) + static_cast<int>(y * 10)) % 17 == 0) {
        for (float z = 0.6f; z <= 3.0f; z += 0.35f) {
          cloud.push_back(pcl::PointXYZ{x, y, z});
        }
      }
    }
  }
  return cloud;
}

/** Sprinkle non-finite returns like a real Gazebo LiDAR (no-return rays). */
void inject_non_finite(pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::size_t base = cloud.size();
  for (std::size_t i = 0; i < base / 10 + 5; ++i) {
    cloud.push_back(pcl::PointXYZ{nan, nan, nan});
    cloud.push_back(pcl::PointXYZ{inf, inf, inf});
    cloud.push_back(pcl::PointXYZ{1.0f, nan, 0.5f});
    cloud.push_back(pcl::PointXYZ{inf, 2.0f, -inf});
  }
}

}  // namespace

int main()
{
  using forest_3d_perception::experimental::CsfGroundSegmentation;
  using forest_3d_perception::experimental::CsfParams;

  const auto cloud = make_robot_like_cloud();
  if (cloud.size() < 100u) {
    std::cerr << "FAIL: synthetic cloud too small (" << cloud.size() << ")\n";
    return 1;
  }

  CsfGroundSegmentation segmenter;
  segmenter.params = CsfParams{};
  segmenter.params.slope_smooth = true;
  segmenter.params.cloth_resolution = 0.5f;
  segmenter.params.iterations = 120;
  segmenter.params.class_threshold = 0.5f;

  auto sim_cloud = make_sim_like_cloud();
  if (sim_cloud.size() < 500u) {
    std::cerr << "FAIL: sim-like cloud too small (" << sim_cloud.size() << ")\n";
    return 1;
  }

  // The crux: clouds carrying NaN/Inf, exactly like the Gazebo LiDAR.
  auto robot_bad = make_robot_like_cloud();
  inject_non_finite(robot_bad);
  auto sim_bad = sim_cloud;
  inject_non_finite(sim_bad);

  const pcl::PointCloud<pcl::PointXYZ> clouds[] = {cloud, sim_cloud, robot_bad, sim_bad};
  const char * labels[] = {"robot_clean", "sim_clean", "robot_with_NaN_Inf", "sim_with_NaN_Inf"};
  for (std::size_t c = 0; c < 4; ++c) {
    for (int trial = 0; trial < 2; ++trial) {
      const auto out = segmenter.segment(clouds[c]);
      if (out.n_ground == 0u && out.n_non_ground == 0u) {
        std::cerr << "FAIL: " << labels[c] << " trial " << trial
                  << " produced no ground/non-ground\n";
        return 1;
      }
      // Non-finite points must never appear in outputs.
      for (const auto & p : out.ground->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          std::cerr << "FAIL: " << labels[c] << " ground contains non-finite point\n";
          return 1;
        }
      }
      for (const auto & p : out.non_ground->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          std::cerr << "FAIL: " << labels[c] << " non_ground contains non-finite point\n";
          return 1;
        }
      }
      std::cout << labels[c] << " trial " << trial << ": in=" << out.n_input
                << " ground=" << out.n_ground << " non_ground=" << out.n_non_ground << "\n";
    }
  }

  std::cout << "OK: CSF survived NaN/Inf input and produced finite outputs\n";
  return 0;
}

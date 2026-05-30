/**
 * @file landmark_tracker.hpp
 * @brief Temporal association of cylinder trunk observations (Sprint 6).
 */

#ifndef FOREST_3D_PERCEPTION__LANDMARK_TRACKER_HPP_
#define FOREST_3D_PERCEPTION__LANDMARK_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "forest_3d_perception/cylinder_fit.hpp"

namespace forest_3d_perception
{

struct TrunkTrack
{
  int id{0};
  float cx{0.0f};
  float cy{0.0f};
  float z_base{0.0f};
  float height{0.0f};
  float radius{0.0f};
  float confidence{0.0f};
  float cylinder_rmse{0.0f};
  int age{0};
  int misses{0};
  CylinderObservation last_obs;
};

struct LandmarkTrackerParams
{
  double assoc_max_xy_m{0.85};
  double assoc_max_radius_m{0.22};
  double birth_min_inlier_ratio{0.30};
  double birth_max_rmse_m{0.20};
  int max_misses{20};
  float confidence_alpha{0.20f};
  double merge_xy_m{0.45};
  double merge_radius_m{0.18};
  std::size_t max_tracks{48};
};

class LandmarkTracker
{
public:
  LandmarkTrackerParams params;
  std::vector<TrunkTrack> tracks;

  std::vector<TrunkTrack> update(const std::vector<CylinderObservation> & detections)
  {
    for (auto & t : tracks) {
      ++t.misses;
    }

    std::vector<bool> det_used(detections.size(), false);
    std::vector<bool> track_matched(tracks.size(), false);

    for (std::size_t ti = 0; ti < tracks.size(); ++ti) {
      auto & track = tracks[ti];
      float best_cost = std::numeric_limits<float>::max();
      int best_di = -1;

      for (std::size_t di = 0; di < detections.size(); ++di) {
        if (det_used[di] || !detections[di].valid || !observation_is_finite(detections[di])) {
          continue;
        }
        const auto & d = detections[di];
        const float dx = track.cx - d.cx;
        const float dy = track.cy - d.cy;
        const float dr = std::abs(track.radius - d.radius);
        if (std::hypot(dx, dy) > params.assoc_max_xy_m || dr > params.assoc_max_radius_m) {
          continue;
        }
        const float cost = std::hypot(dx, dy) + 0.5f * dr;
        if (cost < best_cost) {
          best_cost = cost;
          best_di = static_cast<int>(di);
        }
      }

      if (best_di >= 0) {
        fuse_track(track, detections[static_cast<std::size_t>(best_di)]);
        det_used[static_cast<std::size_t>(best_di)] = true;
        track_matched[ti] = true;
      }
    }

    for (std::size_t di = 0; di < detections.size(); ++di) {
      if (det_used[di] || !detections[di].valid || !observation_is_finite(detections[di])) {
        continue;
      }
      if (tracks.size() >= params.max_tracks) {
        break;
      }
      const auto & d = detections[di];
      if (d.inlier_ratio < static_cast<float>(params.birth_min_inlier_ratio) ||
        d.rmse > static_cast<float>(params.birth_max_rmse_m))
      {
        continue;
      }
      TrunkTrack born;
      born.id = next_id_++;
      born.cx = d.cx;
      born.cy = d.cy;
      born.z_base = d.z_base;
      born.height = d.height;
      born.radius = d.radius;
      born.cylinder_rmse = d.rmse;
      born.confidence = d.inlier_ratio;
      born.age = 1;
      born.misses = 0;
      born.last_obs = d;
      tracks.push_back(born);
    }

    std::vector<TrunkTrack> active;
    active.reserve(tracks.size());
    for (std::size_t ti = 0; ti < tracks.size(); ++ti) {
      auto & t = tracks[ti];
      if (ti < track_matched.size() && track_matched[ti]) {
        t.misses = 0;
        ++t.age;
      }
      if (t.misses <= params.max_misses && observation_is_finite(t.last_obs)) {
        active.push_back(t);
      }
    }
    tracks.swap(active);
    merge_duplicates();
    return tracks;
  }

private:
  int next_id_{1};

  void fuse_track(TrunkTrack & track, const CylinderObservation & d)
  {
    if (!observation_is_finite(d)) {
      return;
    }
    const float a = params.confidence_alpha;
    track.cx = (1.0f - a) * track.cx + a * d.cx;
    track.cy = (1.0f - a) * track.cy + a * d.cy;
    track.z_base = (1.0f - a) * track.z_base + a * d.z_base;
    track.height = (1.0f - a) * track.height + a * d.height;
    track.radius = (1.0f - a) * track.radius + a * d.radius;
    track.cylinder_rmse = (1.0f - a) * track.cylinder_rmse + a * d.rmse;
    track.confidence = std::min(1.0f, track.confidence + a * d.inlier_ratio);
    track.last_obs = d;
  }

  void merge_duplicates()
  {
    if (tracks.size() < 2) {
      return;
    }
    std::vector<bool> drop(tracks.size(), false);
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      if (drop[i]) {
        continue;
      }
      for (std::size_t j = i + 1; j < tracks.size(); ++j) {
        if (drop[j]) {
          continue;
        }
        const float dx = tracks[i].cx - tracks[j].cx;
        const float dy = tracks[i].cy - tracks[j].cy;
        const float dr = std::abs(tracks[i].radius - tracks[j].radius);
        if (std::hypot(dx, dy) >= params.merge_xy_m || dr >= params.merge_radius_m) {
          continue;
        }
        if (tracks[i].age >= tracks[j].age) {
          fuse_track(tracks[i], tracks[j].last_obs);
          drop[j] = true;
        } else {
          fuse_track(tracks[j], tracks[i].last_obs);
          drop[i] = true;
          break;
        }
      }
    }
    std::vector<TrunkTrack> merged;
    merged.reserve(tracks.size());
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      if (!drop[i]) {
        merged.push_back(tracks[i]);
      }
    }
    tracks.swap(merged);
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__LANDMARK_TRACKER_HPP_

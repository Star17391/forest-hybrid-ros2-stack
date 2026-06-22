/**
 * @file cylinder_fit.hpp
 * @brief Vertical-axis cylinder fit for trunk columns (centroid XY + median radius).
 *
 * NOT iterative RANSAC: fixed-axis cylinder with XY centroid, median radial distance,
 * and post-hoc inlier ratio / RMSE gate. See fit_vertical_cylinder().
 */

#ifndef FOREST_3D_PERCEPTION__CYLINDER_FIT_HPP_
#define FOREST_3D_PERCEPTION__CYLINDER_FIT_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <random>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace forest_3d_perception
{

struct CylinderObservation
{
  float cx{0.0f};
  float cy{0.0f};
  float z_base{0.0f};
  float height{0.0f};
  float radius{0.0f};
  float rmse{0.0f};
  float inlier_ratio{0.0f};
  std::size_t n_points{0};
  // Diagnostics for the stem-band selection.
  float ref_radius{0.0f};       // lower-quartile slice radius (the stem reference)
  std::size_t n_band{0};        // points used in the DBH fit (the stem band)
  bool used_fallback{false};    // true = band too sparse, fell back to whole cluster
  // Angular coverage of the visible arc, as sagitta/chord (≈ tan(arc/4)), measured
  // from the fit points and INDEPENDENT of the (uncertain) estimated radius. This
  // is what actually drives DBH uncertainty: a partial arc (low coverage) makes the
  // radius ill-conditioned even when the RMSE is tiny. The consumer must inflate the
  // DBH sigma when this is low — see node's diameter_stddev. 1.0 = full circle.
  float arc_coverage{0.0f};
  bool valid{false};
};

enum class CylinderReject
{
  Accepted,
  TooFewPoints,
  TooShort,
  TooWide,
  HighRmse,
  LowInliers,
};

inline bool observation_is_finite(const CylinderObservation & o)
{
  return std::isfinite(o.cx) && std::isfinite(o.cy) && std::isfinite(o.z_base) &&
         std::isfinite(o.height) && std::isfinite(o.radius) && std::isfinite(o.rmse) &&
         o.height > 0.0f && o.radius > 0.0f;
}

/**
 * Algebraic circle fit (Kåsa). Estimates centre (and radius) of the circle that
 * best fits a set of XY points by solving the 3x3 normal system of
 *   x² + y² + A·x + B·y + C = 0   →   centre = (-A/2, -B/2), r = sqrt(A²/4+B²/4-C).
 *
 * Why this matters for trunks: a ground LiDAR sees only the SENSOR-FACING arc of
 * a trunk (partial occlusion). The simple XY mean of those points is biased
 * toward the sensor by ~radius and inflates the apparent radius. The algebraic
 * circle fit recovers the true centre from the arc, so DBH and base are unbiased.
 *
 * Returns false (caller falls back to the mean) when the system is near-singular
 * (collinear points) or the result is non-finite.
 */
inline bool fit_circle_kasa(
  const std::vector<float> & xs, const std::vector<float> & ys,
  double & cx_out, double & cy_out, double & r_out)
{
  const std::size_t n = xs.size();
  if (n < 4) {
    return false;
  }
  // Work centred on the points' mean for numerical conditioning.
  double mx = 0.0, my = 0.0;
  for (std::size_t i = 0; i < n; ++i) { mx += xs[i]; my += ys[i]; }
  mx /= static_cast<double>(n);
  my /= static_cast<double>(n);

  double Suu = 0.0, Suv = 0.0, Svv = 0.0, Suuu = 0.0, Svvv = 0.0, Suvv = 0.0, Svuu = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double u = xs[i] - mx;
    const double v = ys[i] - my;
    const double uu = u * u, vv = v * v;
    Suu += uu; Svv += vv; Suv += u * v;
    Suuu += uu * u; Svvv += vv * v;
    Suvv += u * vv; Svuu += v * uu;
  }
  // Solve [Suu Suv; Suv Svv] [uc; vc] = 0.5 [Suuu+Suvv; Svvv+Svuu].
  const double det = Suu * Svv - Suv * Suv;
  if (std::abs(det) < 1e-12) {
    return false;  // collinear / degenerate
  }
  const double bx = 0.5 * (Suuu + Suvv);
  const double by = 0.5 * (Svvv + Svuu);
  const double uc = (bx * Svv - by * Suv) / det;
  const double vc = (by * Suu - bx * Suv) / det;
  const double r2 = uc * uc + vc * vc + (Suu + Svv) / static_cast<double>(n);
  if (!(r2 > 0.0) || !std::isfinite(uc) || !std::isfinite(vc)) {
    return false;
  }
  cx_out = uc + mx;
  cy_out = vc + my;
  r_out = std::sqrt(r2);
  return std::isfinite(cx_out) && std::isfinite(cy_out) && std::isfinite(r_out);
}

/**
 * Algebraic circle fit (Taubin 1991). Same role as Kåsa — a fast closed-form
 * seed for the geometric refinement — but statistically less biased: on a
 * partial arc Kåsa pulls the radius noticeably (the ~-13% DBH bias we measured),
 * while Taubin is near the bias-optimal among algebraic fits (Al-Sharadqah &
 * Chernov, 2009). Drop-in replacement: centre on the points' mean, build the
 * normalised moments, find the smallest root of the Taubin characteristic
 * polynomial by Newton's method, recover (cx,cy,r). Falls back to Kåsa if the
 * geometry is degenerate.
 */
inline bool fit_circle_taubin(
  const std::vector<float> & xs, const std::vector<float> & ys,
  double & cx_out, double & cy_out, double & r_out)
{
  const std::size_t n = xs.size();
  if (n < 4) {
    return false;
  }
  double mx = 0.0, my = 0.0;
  for (std::size_t i = 0; i < n; ++i) { mx += xs[i]; my += ys[i]; }
  mx /= static_cast<double>(n);
  my /= static_cast<double>(n);

  double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double u = xs[i] - mx;
    const double v = ys[i] - my;
    const double z = u * u + v * v;
    Mxx += u * u; Myy += v * v; Mxy += u * v;
    Mxz += u * z; Myz += v * z; Mzz += z * z;
  }
  const double inv = 1.0 / static_cast<double>(n);
  Mxx *= inv; Myy *= inv; Mxy *= inv; Mxz *= inv; Myz *= inv; Mzz *= inv;
  const double Mz = Mxx + Myy;
  const double Cov_xy = Mxx * Myy - Mxy * Mxy;
  const double Var_z = Mzz - Mz * Mz;

  // Characteristic polynomial P(x)=A0+A1 x+A2 x^2+A3 x^3 (Taubin); smallest root.
  const double A3 = 4.0 * Mz;
  const double A2 = -3.0 * Mz * Mz - Mzz;
  const double A1 = Var_z * Mz + 4.0 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
  const double A0 = Mxz * (Mxz * Myy - Myz * Mxy) +
                    Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
  const double A22 = A2 + A2;
  const double A33 = A3 + A3 + A3;

  double x = 0.0, y = A0;
  for (int it = 0; it < 99; ++it) {
    const double Dy = A1 + x * (A22 + A33 * x);
    if (std::abs(Dy) < 1e-18) { break; }
    const double xnew = x - y / Dy;
    if (xnew == x || !std::isfinite(xnew)) { break; }
    const double ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
    if (std::abs(ynew) >= std::abs(y)) { break; }
    x = xnew; y = ynew;
  }

  const double det = x * x - x * Mz + Cov_xy;
  if (std::abs(det) < 1e-12) {
    return fit_circle_kasa(xs, ys, cx_out, cy_out, r_out);
  }
  const double xc = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
  const double yc = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;
  const double r2 = xc * xc + yc * yc + Mz;
  if (!(r2 > 0.0) || !std::isfinite(xc) || !std::isfinite(yc)) {
    return fit_circle_kasa(xs, ys, cx_out, cy_out, r_out);
  }
  cx_out = xc + mx;
  cy_out = yc + my;
  r_out = std::sqrt(r2);
  return std::isfinite(cx_out) && std::isfinite(cy_out) && std::isfinite(r_out);
}

/** Exact circle through 3 points (circumcircle). false if (near-)collinear. */
inline bool circumcircle(
  double ax, double ay, double bx, double by, double cx, double cy,
  double & ox, double & oy, double & r)
{
  const double d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (std::abs(d) < 1e-12) {
    return false;
  }
  const double a2 = ax * ax + ay * ay;
  const double b2 = bx * bx + by * by;
  const double c2 = cx * cx + cy * cy;
  ox = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
  oy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
  r = std::hypot(ax - ox, ay - oy);
  return std::isfinite(ox) && std::isfinite(oy) && std::isfinite(r);
}

/**
 * RANSAC circle fit (robust to outliers — branches, occlusion noise, leaves).
 * Samples 3-point circumcircles, scores by inlier count (|dist−r| ≤ inlier_dist),
 * then refines the best model with a Taubin fit on its inliers. Falls back to a
 * plain Taubin fit if too few points or no good model. Deterministic (fixed seed)
 * so the DBH is reproducible frame to frame.
 */
inline bool fit_circle_ransac(
  const std::vector<float> & xs, const std::vector<float> & ys,
  double inlier_dist, double & cx_out, double & cy_out, double & r_out,
  int iters = 48, unsigned seed = 1234u)
{
  const std::size_t n = xs.size();
  if (n < 6) {
    return fit_circle_taubin(xs, ys, cx_out, cy_out, r_out);
  }
  std::mt19937 rng(seed);
  std::uniform_int_distribution<std::size_t> pick(0, n - 1);
  long best_inliers = -1;
  double bcx = 0.0, bcy = 0.0, br = 0.0;
  for (int it = 0; it < iters; ++it) {
    const std::size_t a = pick(rng), b = pick(rng), c = pick(rng);
    if (a == b || b == c || a == c) {
      continue;
    }
    double ox, oy, rr;
    if (!circumcircle(xs[a], ys[a], xs[b], ys[b], xs[c], ys[c], ox, oy, rr)) {
      continue;
    }
    long inl = 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (std::abs(std::hypot(xs[i] - ox, ys[i] - oy) - rr) <= inlier_dist) {
        ++inl;
      }
    }
    if (inl > best_inliers) {
      best_inliers = inl;
      bcx = ox; bcy = oy; br = rr;
    }
  }
  if (best_inliers < 3) {
    return fit_circle_taubin(xs, ys, cx_out, cy_out, r_out);
  }
  // Refine on the best model's inliers.
  std::vector<float> ix, iy;
  ix.reserve(n);
  iy.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (std::abs(std::hypot(xs[i] - bcx, ys[i] - bcy) - br) <= inlier_dist) {
      ix.push_back(xs[i]);
      iy.push_back(ys[i]);
    }
  }
  if (ix.size() >= 4 && fit_circle_taubin(ix, iy, cx_out, cy_out, r_out)) {
    return true;
  }
  cx_out = bcx; cy_out = bcy; r_out = br;
  return std::isfinite(br) && br > 0.0;
}

/**
 * Geometric circle refinement (Landau 1987). The algebraic Kåsa/Taubin fits
 * minimise an algebraic error and OVER-estimate the radius on partial arcs — a
 * trunk seen only from the sensor-facing side fits to a much larger circle. Landau's
 * iteration minimises the true geometric error Σ(dist−r)² and pulls the centre/radius
 * back to the real trunk. Starts from the algebraic estimate (cx,cy).
 */
inline void refine_circle_landau(
  const std::vector<float> & xs, const std::vector<float> & ys,
  double & cx, double & cy, double & r, int iters = 12)
{
  const std::size_t n = xs.size();
  if (n < 3) {
    return;
  }
  for (int it = 0; it < iters; ++it) {
    double mx = 0.0, my = 0.0, mr = 0.0, sux = 0.0, suy = 0.0;
    std::size_t cnt = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const double dx = xs[i] - cx, dy = ys[i] - cy;
      const double d = std::hypot(dx, dy);
      if (d < 1e-9) {
        continue;
      }
      mx += xs[i]; my += ys[i]; mr += d;
      sux += dx / d; suy += dy / d;
      ++cnt;
    }
    if (cnt == 0) {
      return;
    }
    const double inv = 1.0 / static_cast<double>(cnt);
    mr *= inv;
    const double ncx = mx * inv - mr * (sux * inv);
    const double ncy = my * inv - mr * (suy * inv);
    const bool converged = std::hypot(ncx - cx, ncy - cy) < 1e-6;
    cx = ncx; cy = ncy;
    if (converged) {
      break;
    }
  }
  double mr = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    mr += std::hypot(xs[i] - cx, ys[i] - cy);
  }
  r = mr / static_cast<double>(n);
}

/**
 * Angular coverage of a point arc as sagitta/chord (≈ tan(arc/4)), in [0, ~0.5].
 * chord = the two most distant points; sagitta = a high percentile of the
 * perpendicular deviations to that chord (the median resists noise better than the
 * max). Crucially this is INDEPENDENT of the fitted radius, so it stays valid even
 * when the radius is wrong on a short arc — unlike chord/(2r), which collapses
 * because r is itself underestimated. Returns 0 for degenerate inputs.
 */
inline float arc_coverage_sagitta(const std::vector<float> & xs, const std::vector<float> & ys)
{
  const std::size_t n = xs.size();
  if (n < 3) {
    return 0.0f;
  }
  // Farthest pair = chord (O(n²), but the band is small).
  std::size_t ia = 0, ib = 0;
  double best = -1.0;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const double d = std::hypot(xs[i] - xs[j], ys[i] - ys[j]);
      if (d > best) { best = d; ia = i; ib = j; }
    }
  }
  const double L = best;
  if (L < 1e-6) {
    return 0.0f;
  }
  const double nx = -(ys[ib] - ys[ia]) / L;  // unit normal to the chord
  const double ny = (xs[ib] - xs[ia]) / L;
  std::vector<double> dev;
  dev.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    dev.push_back(std::abs((xs[i] - xs[ia]) * nx + (ys[i] - ys[ia]) * ny));
  }
  std::sort(dev.begin(), dev.end());
  const double sag = dev[static_cast<std::size_t>(0.75 * (dev.size() - 1))];  // robust sagitta
  return static_cast<float>(sag / L);
}

/**
 * Connected components of a slice's points by XY single-link distance (union-find).
 * The trunk and an adjacent branch fall into different components when separated by
 * more than link_d; this lets the slice-walk pick the trunk lobe and drop the branch.
 */
inline std::vector<std::vector<std::size_t>> slice_components(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std::vector<std::size_t> & ids, float link_d)
{
  std::vector<std::vector<std::size_t>> comps;
  const std::size_t n = ids.size();
  if (n == 0) {
    return comps;
  }
  std::vector<std::size_t> parent(n);
  for (std::size_t i = 0; i < n; ++i) { parent[i] = i; }
  std::function<std::size_t(std::size_t)> find = [&](std::size_t a) {
    while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
    return a;
  };
  const float d2 = link_d * link_d;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const auto & pi = cloud.points[ids[i]];
      const auto & pj = cloud.points[ids[j]];
      const float dx = pi.x - pj.x, dy = pi.y - pj.y;
      if (dx * dx + dy * dy <= d2) {
        const std::size_t ri = find(i), rj = find(j);
        if (ri != rj) { parent[ri] = rj; }
      }
    }
  }
  std::map<std::size_t, std::vector<std::size_t>> m;
  for (std::size_t i = 0; i < n; ++i) { m[find(i)].push_back(ids[i]); }
  for (auto & kv : m) { comps.push_back(std::move(kv.second)); }
  return comps;
}

/** Median XY of a set of point indices (robust slice centre). */
inline void median_xy(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<std::size_t> & ids,
  float & mx, float & my)
{
  std::vector<float> xs, ys;
  xs.reserve(ids.size());
  ys.reserve(ids.size());
  for (std::size_t id : ids) { xs.push_back(cloud.points[id].x); ys.push_back(cloud.points[id].y); }
  std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
  std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
  mx = xs[xs.size() / 2];
  my = ys[ys.size() / 2];
}

inline CylinderReject fit_vertical_cylinder(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std::vector<std::size_t> & indices,
  CylinderObservation & out,
  double min_height_m,
  double max_radius_m,
  double max_rmse_m,
  double min_inlier_ratio,
  double inlier_dist_m,
  double max_trunk_slice_height_m = 2.5,
  double dbh_band_low_m = 0.3,
  double dbh_band_high_m = 2.5,
  double stem_grow_factor = 1.8,
  double stem_axis_jump_m = 0.20,
  std::vector<std::size_t> * out_band_indices = nullptr)
{
  if (indices.size() < 5) {
    return CylinderReject::TooFewPoints;
  }

  std::vector<float> zs;
  zs.reserve(indices.size());
  for (std::size_t idx : indices) {
    zs.push_back(cloud.points[idx].z);
  }
  std::sort(zs.begin(), zs.end());

  // AXIS-ANCHORED TRUNK CORE (robust to canopy-dominated clusters).
  //   The earlier z_base = 10th-percentile-of-z is corrupted whenever the cluster is
  //   mostly canopy — an up-tilted LiDAR sees far more crown than stem, so the 10th
  //   percentile lands INSIDE the crown, the whole DBH band shifts up into the
  //   canopy, and the DBH explodes (measured: a 0.66 m trunk read as 3.05 m). The
  //   crown can no longer corrupt anything once we ANCHOR on the trunk axis:
  //     1. Seed the axis from the lowest clean slice (near the ground only the stem
  //        exists — the crown is above), via a robust circle fit.
  //     2. Refit the foot z_base from the MINIMUM z of points NEAR that axis (the
  //        crown, being far from the thin axis, cannot pull it up).
  //     3. Grow a vertical core upward keeping ONLY points inside a radial tube
  //        around the (lean-tracked) axis; the crown is excluded by construction
  //        because it sits far from the axis. The run ends where the stem ends.
  const float z_lo0 = zs.front();

  // --- 1. Seed the trunk axis from the lowest band. ---
  auto collect_xy = [&](float zlo, float zhi, std::vector<float> & sx,
                        std::vector<float> & sy) {
    for (std::size_t idx : indices) {
      const auto & p = cloud.points[idx];
      if (p.z >= zlo && p.z <= zhi) {
        sx.push_back(p.x);
        sy.push_back(p.y);
      }
    }
  };
  std::vector<float> seed_x, seed_y;
  collect_xy(z_lo0 + 0.10f, z_lo0 + 0.90f, seed_x, seed_y);
  if (seed_x.size() < 4) {
    seed_x.clear();
    seed_y.clear();
    collect_xy(z_lo0, z_lo0 + 1.50f, seed_x, seed_y);
  }
  // A ground LiDAR sees only the sensor-facing ARC of the stem (here arc_coverage is
  // ~0.1, i.e. ~30°). A circle fit on so thin an arc throws the centre/radius all over
  // the place, so we do NOT trust it for the anchor. Instead anchor on the near
  // SURFACE — the median XY of the seed points — which is stable up a vertical stem,
  // and size the radial tube from the seed's own lateral spread. The crown, metres
  // away in XY, never enters the tube. The true centre/radius are recovered later by
  // the RANSAC+Landau fit on the gathered arc (Landau removes the partial-arc bias).
  double axc = 0.0, ayc = 0.0, r_seed = 0.1;
  if (!seed_x.empty()) {
    std::vector<float> tx = seed_x, ty = seed_y;
    std::sort(tx.begin(), tx.end());
    std::sort(ty.begin(), ty.end());
    axc = tx[tx.size() / 2];
    ayc = ty[ty.size() / 2];
    std::vector<float> rr;
    rr.reserve(seed_x.size());
    for (std::size_t i = 0; i < seed_x.size(); ++i) {
      rr.push_back(std::hypot(seed_x[i] - axc, seed_y[i] - ayc));
    }
    std::sort(rr.begin(), rr.end());
    r_seed = rr[3 * rr.size() / 4];  // 75th pct lateral spread of the near arc
  }
  r_seed = std::clamp(r_seed, 0.05, 0.40);

  // --- 2. Foot z_base from the min-z of NEAR-AXIS points (crown can't corrupt it). ---
  // Radial tube ~ one stem diameter around the near-surface anchor: wide enough to
  // keep the whole stem arc as the anchor tracks up, tight enough to drop the crown.
  const float gate_r = std::clamp(
    static_cast<float>(stem_grow_factor * 2.0 * r_seed) + 0.10f, 0.22f, 0.50f);
  float z_base = std::numeric_limits<float>::max();
  for (std::size_t idx : indices) {
    const auto & p = cloud.points[idx];
    if (std::hypot(p.x - axc, p.y - ayc) <= gate_r) {
      z_base = std::min(z_base, p.z);
    }
  }
  if (!std::isfinite(z_base) || z_base > z_lo0 + 1.0f) {
    z_base = z_lo0;
  }
  // Breast-height ceiling for the sparse-stem fallback band (see §4).
  constexpr float kBhHi = 1.3f;
  (void)max_trunk_slice_height_m;
  (void)dbh_band_low_m;
  (void)dbh_band_high_m;
  (void)stem_axis_jump_m;

  // Sparse-stem fallback indices: points near the seed axis, BELOW the breast-height
  // ceiling. Two guards that matter: (1) RADIALLY GATED (not the whole cluster) so a
  // distant stem hit by only a handful of points does not re-admit the crown — the
  // measured 8 m leak; (2) z-CAPPED at breast height so a low crown (Tree5, foliage
  // from ~1.5 m) cannot enter the fallback either. The fallback is the lower stem.
  std::vector<std::size_t> fit_indices;
  fit_indices.reserve(indices.size());
  for (std::size_t idx : indices) {
    const auto & p = cloud.points[idx];
    if (p.z <= z_base + kBhHi && std::hypot(p.x - axc, p.y - ayc) <= gate_r) {
      fit_indices.push_back(idx);
    }
  }

  // --- 3. Slice the whole column above the foot; DP-track the trunk thread. ---
  constexpr double kSliceH = 0.15;
  constexpr float kLinkD = 0.18f;
  const float z_top_all = zs.back();
  const int nb = std::clamp(
    static_cast<int>(std::ceil((z_top_all - z_base) / kSliceH)), 1, 400);
  std::vector<std::vector<std::size_t>> bins(static_cast<std::size_t>(nb));
  for (std::size_t idx : indices) {
    const float z = cloud.points[idx].z;
    if (z >= z_base) {
      int b = std::clamp(static_cast<int>((z - z_base) / kSliceH), 0, nb - 1);
      bins[static_cast<std::size_t>(b)].push_back(idx);
    }
  }
  struct SliceComp { std::vector<std::size_t> ids; float cx; float cy; int n; };
  std::vector<int> slc_b;                    // bin index of each non-empty slice
  std::vector<std::vector<SliceComp>> slc;   // its connected components
  for (int b = 0; b < nb; ++b) {
    if (bins[static_cast<std::size_t>(b)].empty()) { continue; }
    auto cs = slice_components(cloud, bins[static_cast<std::size_t>(b)], kLinkD);
    std::vector<SliceComp> cv;
    cv.reserve(cs.size());
    for (auto & c : cs) {
      SliceComp sc;
      median_xy(cloud, c, sc.cx, sc.cy);
      sc.n = static_cast<int>(c.size());
      sc.ids = std::move(c);
      cv.push_back(std::move(sc));
    }
    slc_b.push_back(b);
    slc.push_back(std::move(cv));
  }

  // Viterbi DP: the smoothest continuous thread from the (trunk-dominated) base slice.
  // A detour onto a branch lobe and back costs more lateral movement than staying on
  // the near-vertical stem, so the optimal path follows the trunk and ignores branches
  // — unlike a greedy walk, a single wrong local pick cannot derail the whole column.
  const std::size_t K = slc.size();
  std::vector<int> chosen(K, -1);
  if (K > 0) {
    constexpr double kINF = 1e18;
    std::vector<std::vector<double>> dp(K);
    std::vector<std::vector<int>> bk(K);
    for (std::size_t k = 0; k < K; ++k) {
      dp[k].assign(slc[k].size(), kINF);
      bk[k].assign(slc[k].size(), -1);
    }
    int j0 = 0;
    for (std::size_t j = 1; j < slc[0].size(); ++j) {
      if (slc[0][j].n > slc[0][static_cast<std::size_t>(j0)].n) { j0 = static_cast<int>(j); }
    }
    dp[0][static_cast<std::size_t>(j0)] = 0.0;
    for (std::size_t k = 1; k < K; ++k) {
      const int gap = slc_b[k] - slc_b[k - 1];
      if (gap > 4) { continue; }                 // big vertical break → thread ends here
      const double gate = std::min(0.30, 0.16 + 0.45 * gap);
      for (std::size_t j = 0; j < slc[k].size(); ++j) {
        for (std::size_t i = 0; i < slc[k - 1].size(); ++i) {
          if (dp[k - 1][i] >= kINF) { continue; }
          const double d = std::hypot(slc[k][j].cx - slc[k - 1][i].cx,
                                      slc[k][j].cy - slc[k - 1][i].cy);
          if (d <= gate && dp[k - 1][i] + d < dp[k][j]) {
            dp[k][j] = dp[k - 1][i] + d;
            bk[k][j] = static_cast<int>(i);
          }
        }
      }
    }
    int end_k = -1;
    for (int k = static_cast<int>(K) - 1; k >= 0 && end_k < 0; --k) {
      for (double v : dp[static_cast<std::size_t>(k)]) { if (v < kINF) { end_k = k; break; } }
    }
    if (end_k >= 0) {
      int j = 0;
      double bestv = kINF;
      for (std::size_t jj = 0; jj < dp[static_cast<std::size_t>(end_k)].size(); ++jj) {
        if (dp[static_cast<std::size_t>(end_k)][jj] < bestv) {
          bestv = dp[static_cast<std::size_t>(end_k)][jj];
          j = static_cast<int>(jj);
        }
      }
      for (int k = end_k; k >= 0 && j >= 0; --k) {
        chosen[static_cast<std::size_t>(k)] = j;
        j = bk[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)];
      }
    }
  }

  // Per-slice thread centre (carried across gaps), clean radius and full extent.
  std::vector<float> sl_z(K), sl_cx(K), sl_cy(K), sl_r(K), sl_ext(K);
  std::vector<int> sl_on(K, 0);
  float last_cx = static_cast<float>(axc), last_cy = static_cast<float>(ayc), last_r = 0.1f;
  for (std::size_t k = 0; k < K; ++k) {
    sl_z[k] = z_base + (static_cast<float>(slc_b[k]) + 0.5f) * static_cast<float>(kSliceH);
    if (chosen[k] >= 0) {
      const SliceComp & c = slc[k][static_cast<std::size_t>(chosen[k])];
      last_cx = c.cx; last_cy = c.cy;
      std::vector<float> rr;
      rr.reserve(c.ids.size());
      for (std::size_t id : c.ids) {
        rr.push_back(std::hypot(cloud.points[id].x - c.cx, cloud.points[id].y - c.cy));
      }
      std::sort(rr.begin(), rr.end());
      last_r = std::max(rr[static_cast<std::size_t>(0.40 * (rr.size() - 1))], 0.02f);
      sl_on[k] = 1;
    }
    sl_cx[k] = last_cx; sl_cy[k] = last_cy; sl_r[k] = last_r;
    std::vector<float> ra;
    const auto & bp = bins[static_cast<std::size_t>(slc_b[k])];
    ra.reserve(bp.size());
    for (std::size_t id : bp) {
      ra.push_back(std::hypot(cloud.points[id].x - last_cx, cloud.points[id].y - last_cy));
    }
    std::sort(ra.begin(), ra.end());
    sl_ext[k] = ra.empty() ? 0.0f : ra[static_cast<std::size_t>(0.95 * (ra.size() - 1))];
  }

  // --- Crown base: first slice where the extent opens up in a SUSTAINED way. ---
  // Reference stem radius = median of the low clean slices; the crown begins where the
  // 95th-pct extent exceeds max(0.5 m, 3·r_ref) for ≥ kSustain consecutive slices (an
  // isolated wide slice — one branch — does not trigger it). The fuste is cut BELOW it.
  float r_ref = 0.15f;
  {
    std::vector<float> low;
    const std::size_t nlow = std::max<std::size_t>(3, K / 3);
    for (std::size_t k = 0; k < K && k < nlow; ++k) {
      if (sl_on[k]) { low.push_back(sl_r[k]); }
    }
    if (!low.empty()) {
      std::sort(low.begin(), low.end());
      r_ref = low[low.size() / 2];
    }
  }
  const float crown_thr = std::max(0.5f, 3.0f * r_ref);
  constexpr int kSustain = 3;
  float z_copa = std::numeric_limits<float>::max();
  int run0 = -1;
  for (std::size_t k = 0; k < K; ++k) {
    if (sl_ext[k] > crown_thr) {
      if (run0 < 0) { run0 = static_cast<int>(k); }
      if (static_cast<int>(k) - run0 + 1 >= kSustain) {
        z_copa = sl_z[static_cast<std::size_t>(run0)];
        break;
      }
    } else {
      run0 = -1;
    }
  }

  // --- 4. Column below the crown; TRUE-AXIS circle fit + radial trim = the fuste. ---
  // Recenter every column point by its own slice centre so the partial arcs of a
  // curved/leaning stem align; one circle fit then recovers the centroid→axis offset
  // (dx,dy) and the true radius r_trunk. A branch sits beyond r_trunk from the axis and
  // is trimmed by its WHOLE body, not just the tip.
  std::vector<std::size_t> col_ids;
  std::vector<float> rel_x, rel_y;
  float foot_cx = static_cast<float>(axc), foot_cy = static_cast<float>(ayc);
  bool foot_set = false;
  for (std::size_t k = 0; k < K; ++k) {
    if (chosen[k] < 0 || sl_z[k] >= z_copa) { continue; }
    if (!foot_set) { foot_cx = sl_cx[k]; foot_cy = sl_cy[k]; foot_set = true; }
    for (std::size_t id : slc[k][static_cast<std::size_t>(chosen[k])].ids) {
      col_ids.push_back(id);
      rel_x.push_back(cloud.points[id].x - sl_cx[k]);
      rel_y.push_back(cloud.points[id].y - sl_cy[k]);
    }
  }

  std::vector<std::size_t> dbh_indices;
  float cx = foot_cx, cy = foot_cy, radius = r_ref;
  std::vector<float> resid;          // |dist_to_axis − r_trunk| for rmse/inliers
  bool used_fb = false;
  if (col_ids.size() >= 5) {
    double dx = 0.0, dy = 0.0, rt = 0.0;
    if (!fit_circle_taubin(rel_x, rel_y, dx, dy, rt)) { dx = 0.0; dy = 0.0; rt = r_ref; }
    refine_circle_landau(rel_x, rel_y, dx, dy, rt);
    {  // one refit on the kept (true-axis inlier) points
      std::vector<float> kx, ky;
      const double lim0 = 1.35 * rt + 0.03;
      for (std::size_t i = 0; i < rel_x.size(); ++i) {
        if (std::hypot(rel_x[i] - dx, rel_y[i] - dy) <= lim0) {
          kx.push_back(rel_x[i]); ky.push_back(rel_y[i]);
        }
      }
      if (kx.size() >= 5) {
        double ndx = dx, ndy = dy, nrt = rt;
        if (fit_circle_taubin(kx, ky, ndx, ndy, nrt)) {
          refine_circle_landau(kx, ky, ndx, ndy, nrt);
          dx = ndx; dy = ndy; rt = nrt;
        }
      }
    }
    const double lim = 1.35 * rt + 0.03;
    for (std::size_t i = 0; i < col_ids.size(); ++i) {
      const double d = std::hypot(rel_x[i] - dx, rel_y[i] - dy);
      if (d <= lim) {
        dbh_indices.push_back(col_ids[i]);
        resid.push_back(static_cast<float>(std::abs(d - rt)));
      }
    }
    radius = static_cast<float>(rt);
    cx = foot_cx + static_cast<float>(dx);
    cy = foot_cy + static_cast<float>(dy);
  }
  if (dbh_indices.size() < 5) {
    // Fallback: too few thread points → gated lower-stem world fit (legacy path).
    used_fb = true;
    dbh_indices = fit_indices;
    if (dbh_indices.size() >= 5) {
      std::vector<float> fx, fy;
      fx.reserve(dbh_indices.size());
      fy.reserve(dbh_indices.size());
      for (std::size_t id : dbh_indices) { fx.push_back(cloud.points[id].x); fy.push_back(cloud.points[id].y); }
      double rc_cx = 0, rc_cy = 0, rc_r = 0;
      if (fit_circle_ransac(fx, fy, inlier_dist_m, rc_cx, rc_cy, rc_r) && rc_r <= max_radius_m * 1.5) {
        refine_circle_landau(fx, fy, rc_cx, rc_cy, rc_r);
        cx = static_cast<float>(rc_cx); cy = static_cast<float>(rc_cy); radius = static_cast<float>(rc_r);
      } else {
        median_xy(cloud, dbh_indices, cx, cy);
      }
      resid.clear();
      resid.reserve(dbh_indices.size());
      for (std::size_t id : dbh_indices) {
        resid.push_back(std::abs(std::hypot(cloud.points[id].x - cx, cloud.points[id].y - cy) - radius));
      }
    }
  }

  if (out_band_indices) { *out_band_indices = dbh_indices; }
  const std::size_t n_band_stem = dbh_indices.size();
  if (dbh_indices.size() < 5) {
    out.ref_radius = static_cast<float>(r_seed);
    out.n_band = n_band_stem;
    out.used_fallback = used_fb;
    out.z_base = z_base;
    out.valid = false;
    return CylinderReject::TooFewPoints;
  }

  // Geometry + diagnostics (filled BEFORE the quality gates, as in the original: a
  // rejected fit still carries the stem-band centre so the consumer never falls back
  // to the whole-cloud centroid).
  std::vector<float> fit_xs, fit_ys;
  fit_xs.reserve(dbh_indices.size());
  fit_ys.reserve(dbh_indices.size());
  float z_stem_hi = -std::numeric_limits<float>::max();
  for (std::size_t idx : dbh_indices) {
    fit_xs.push_back(cloud.points[idx].x);
    fit_ys.push_back(cloud.points[idx].y);
    z_stem_hi = std::max(z_stem_hi, cloud.points[idx].z);
  }
  const float height = z_stem_hi - z_base;
  out.cx = cx;
  out.cy = cy;
  out.z_base = z_base;
  out.height = height;
  out.n_points = indices.size();
  out.ref_radius = static_cast<float>(r_seed);
  out.n_band = n_band_stem;
  out.used_fallback = used_fb;
  out.arc_coverage = arc_coverage_sagitta(fit_xs, fit_ys);
  out.radius = radius;

  if (height < static_cast<float>(min_height_m)) {
    out.valid = false;
    return CylinderReject::TooShort;
  }
  if (radius > static_cast<float>(max_radius_m)) {
    out.valid = false;
    return CylinderReject::TooWide;
  }

  double sum_err2 = 0.0;
  std::size_t inliers = 0;
  for (float e : resid) {
    sum_err2 += static_cast<double>(e) * e;
    if (e <= inlier_dist_m) { ++inliers; }
  }
  const float rmse = resid.empty() ? 0.0f :
    static_cast<float>(std::sqrt(sum_err2 / static_cast<double>(resid.size())));
  const float inlier_ratio = resid.empty() ? 0.0f :
    static_cast<float>(inliers) / static_cast<float>(resid.size());
  out.rmse = rmse;
  out.inlier_ratio = inlier_ratio;
  if (rmse > static_cast<float>(max_rmse_m)) {
    out.valid = false;
    return CylinderReject::HighRmse;
  }
  if (inlier_ratio < static_cast<float>(min_inlier_ratio)) {
    out.valid = false;
    return CylinderReject::LowInliers;
  }

  out.valid = observation_is_finite(out);
  return out.valid ? CylinderReject::Accepted : CylinderReject::HighRmse;
}

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__CYLINDER_FIT_HPP_

#include "forest_tree_slam/multiview_dbh.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace forest_tree_slam
{
namespace
{
bool fit_circle_kasa(
  const std::vector<float> & xs, const std::vector<float> & ys, double & cx_out, double & cy_out,
  double & r_out)
{
  const std::size_t n = xs.size();
  if (n < 4) {
    return false;
  }
  double mx = 0.0, my = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= static_cast<double>(n);
  my /= static_cast<double>(n);

  double Suu = 0.0, Suv = 0.0, Svv = 0.0, Suuu = 0.0, Svvv = 0.0, Suvv = 0.0, Svuu = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double u = xs[i] - mx;
    const double v = ys[i] - my;
    const double uu = u * u, vv = v * v;
    Suu += uu;
    Svv += vv;
    Suv += u * v;
    Suuu += uu * u;
    Svvv += vv * v;
    Suvv += u * vv;
    Svuu += v * uu;
  }
  const double det = Suu * Svv - Suv * Suv;
  if (std::abs(det) < 1e-12) {
    return false;
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
}  // namespace

bool fit_circle_taubin_xy(
  const std::vector<float> & xs, const std::vector<float> & ys, double & cx_out, double & cy_out,
  double & r_out)
{
  const std::size_t n = xs.size();
  if (n < 4) {
    return false;
  }
  double mx = 0.0, my = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= static_cast<double>(n);
  my /= static_cast<double>(n);

  double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double u = xs[i] - mx;
    const double v = ys[i] - my;
    const double z = u * u + v * v;
    Mxx += u * u;
    Myy += v * v;
    Mxy += u * v;
    Mxz += u * z;
    Myz += v * z;
    Mzz += z * z;
  }
  const double inv = 1.0 / static_cast<double>(n);
  Mxx *= inv;
  Myy *= inv;
  Mxy *= inv;
  Mxz *= inv;
  Myz *= inv;
  Mzz *= inv;
  const double Mz = Mxx + Myy;
  const double Cov_xy = Mxx * Myy - Mxy * Mxy;
  const double Var_z = Mzz - Mz * Mz;

  const double A3 = 4.0 * Mz;
  const double A2 = -3.0 * Mz * Mz - Mzz;
  const double A1 = Var_z * Mz + 4.0 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
  const double A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
  const double A22 = A2 + A2;
  const double A33 = A3 + A3 + A3;

  double x = 0.0, y = A0;
  for (int it = 0; it < 99; ++it) {
    const double Dy = A1 + x * (A22 + A33 * x);
    if (std::abs(Dy) < 1e-18) {
      break;
    }
    const double xnew = x - y / Dy;
    if (xnew == x || !std::isfinite(xnew)) {
      break;
    }
    const double ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
    if (std::abs(ynew) >= std::abs(y)) {
      break;
    }
    x = xnew;
    y = ynew;
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

float arc_coverage_from_xy(
  const std::vector<float> & xs, const std::vector<float> & ys, double cx, double cy)
{
  if (xs.size() < 4) {
    return 0.0F;
  }
  std::size_t ia = 0, ib = 1;
  double best_d2 = -1.0;
  for (std::size_t i = 0; i < xs.size(); ++i) {
    for (std::size_t j = i + 1; j < xs.size(); ++j) {
      const double dx = xs[i] - xs[j];
      const double dy = ys[i] - ys[j];
      const double d2 = dx * dx + dy * dy;
      if (d2 > best_d2) {
        best_d2 = d2;
        ia = i;
        ib = j;
      }
    }
  }
  if (best_d2 < 1e-8) {
    return 0.0F;
  }
  const double L = std::sqrt(best_d2);
  const double nx = -(ys[ib] - ys[ia]) / L;
  const double ny = (xs[ib] - xs[ia]) / L;
  std::vector<double> dev;
  dev.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) {
    dev.push_back(std::abs((xs[i] - xs[ia]) * nx + (ys[i] - ys[ia]) * ny));
  }
  std::sort(dev.begin(), dev.end());
  const double sag = dev[static_cast<std::size_t>(0.75 * (dev.size() - 1))];
  return static_cast<float>(std::clamp(sag / L, 0.0, 1.0));
}

MultiviewPointBuffer::MultiviewPointBuffer(MultiviewDbhParams params)
: params_(params)
{
}

double MultiviewPointBuffer::coverage_ratio() const
{
  const int bins = std::max(1, params_.coverage_bins);
  int filled = 0;
  for (int b = 0; b < bins && b < 64; ++b) {
    if ((coverage_bits_ & (1ULL << static_cast<unsigned>(b))) != 0) {
      ++filled;
    }
  }
  return static_cast<double>(filled) / static_cast<double>(bins);
}

std::uint64_t MultiviewPointBuffer::voxel_key(const Eigen::Vector3d & p) const
{
  const double inv = 1.0 / std::max(params_.voxel_size_m, 1e-4);
  const std::int64_t ix = static_cast<std::int64_t>(std::floor(p.x() * inv));
  const std::int64_t iy = static_cast<std::int64_t>(std::floor(p.y() * inv));
  const std::int64_t iz = static_cast<std::int64_t>(std::floor(p.z() * inv));
  const auto mix = [](std::uint64_t h, std::int64_t v) {
      return h ^ (static_cast<std::uint64_t>(v) * 0x9E3779B97F4A7C15ULL);
    };
  std::uint64_t h = 0;
  h = mix(h, ix);
  h = mix(h, iy);
  h = mix(h, iz);
  return h;
}

int MultiviewPointBuffer::view_coverage_bin(
  const Eigen::Vector2d & landmark_xy, const Eigen::Vector2d & view_xy) const
{
  constexpr double kTwoPi = 2.0 * M_PI;
  const double bearing = std::atan2(view_xy.y() - landmark_xy.y(), view_xy.x() - landmark_xy.x());
  double b = std::fmod(bearing + kTwoPi, kTwoPi);
  const int bins = std::max(1, params_.coverage_bins);
  int bin = static_cast<int>(b / kTwoPi * static_cast<double>(bins));
  if (bin >= bins) {
    bin = bins - 1;
  }
  return bin;
}

bool MultiviewPointBuffer::insert_frame(
  const std::vector<Eigen::Vector3d> & points_map, const Eigen::Vector2d & landmark_xy,
  const Eigen::Vector2d & view_xy)
{
  if (saturated_ || points_map.empty()) {
    return false;
  }

  bool added_voxel = false;
  for (const auto & p : points_map) {
    const std::uint64_t key = voxel_key(p);
    if (voxels_.find(key) == voxels_.end()) {
      voxels_.emplace(key, p);
      added_voxel = true;
    }
  }

  const int bin = view_coverage_bin(landmark_xy, view_xy);
  const std::uint64_t bit = 1ULL << static_cast<unsigned>(bin % 64);
  bool new_bin = (coverage_bits_ & bit) == 0;
  if (new_bin) {
    coverage_bits_ |= bit;
    ++n_inlier_frames_;
  }

  return added_voxel || new_bin;
}

DbhRefitResult MultiviewPointBuffer::refit(
  double prior_cx, double prior_cy, double prior_r, bool has_prior) const
{
  DbhRefitResult out;
  if (voxels_.size() < params_.refit_min_points) {
    return out;
  }

  std::vector<float> xs, ys;
  xs.reserve(voxels_.size());
  ys.reserve(voxels_.size());
  for (const auto & kv : voxels_) {
    xs.push_back(static_cast<float>(kv.second.x()));
    ys.push_back(static_cast<float>(kv.second.y()));
  }

  const double trim = std::max(params_.trim_tol_m, 1.5 * params_.voxel_size_m);
  // Recolhe os pontos a <= trim de um círculo (cx,cy,cr).
  const auto inliers_to = [&](double ccx, double ccy, double cr, std::vector<float> & ix,
      std::vector<float> & iy) {
      ix.clear();
      iy.clear();
      for (std::size_t i = 0; i < xs.size(); ++i) {
        if (std::abs(std::hypot(xs[i] - ccx, ys[i] - ccy) - cr) <= trim) {
          ix.push_back(xs[i]);
          iy.push_back(ys[i]);
        }
      }
    };

  // Conjunto de trabalho inicial: ANCORADO no prior (rejeita o anel deslocado
  // sem deixar o least-squares ser puxado por ele); senão, todos os pontos.
  std::vector<float> ix, iy;
  if (has_prior && prior_r > 0.0) {
    inliers_to(prior_cx, prior_cy, prior_r, ix, iy);
  }
  if (ix.size() < params_.refit_min_points) {
    ix = xs;
    iy = ys;
  }

  double cx = 0.0, cy = 0.0, r = 0.0;
  if (!fit_circle_taubin_xy(ix, iy, cx, cy, r)) {
    return out;
  }
  // Refino IRLS: re-seleciona inliers de TODOS os pontos face ao círculo atual e
  // reajusta (recupera bons pontos que o prior pudesse ter excluído por erro).
  for (int it = 0; it < 2; ++it) {
    inliers_to(cx, cy, r, ix, iy);
    if (ix.size() < params_.refit_min_points) {
      break;
    }
    double ncx = 0.0, ncy = 0.0, nr = 0.0;
    if (!fit_circle_taubin_xy(ix, iy, ncx, ncy, nr)) {
      break;
    }
    cx = ncx;
    cy = ncy;
    r = nr;
  }
  if (r <= 0.0 || r > params_.refit_max_radius_m) {
    return out;
  }

  // rmse/arco SÓ nos inliers finais (outliers não pertencem ao tronco).
  inliers_to(cx, cy, r, ix, iy);
  if (ix.size() < params_.refit_min_points) {
    return out;
  }
  double err2 = 0.0;
  for (std::size_t i = 0; i < ix.size(); ++i) {
    const double e = std::hypot(ix[i] - cx, iy[i] - cy) - r;
    err2 += e * e;
  }

  out.valid = true;
  out.cx = cx;
  out.cy = cy;
  out.radius = r;
  out.arc_coverage = arc_coverage_from_xy(ix, iy, cx, cy);
  out.rmse = std::sqrt(err2 / static_cast<double>(ix.size()));
  return out;
}

void MultiviewPointBuffer::saturate()
{
  // Pára de ingerir, mas MANTÉM os voxels: a referência acumulada continua disponível
  // via points() (visualização no RViz; o DBH refinado já está guardado no track).
  saturated_ = true;
}

void MultiviewPointBuffer::reset()
{
  saturated_ = false;
  voxels_.clear();
  coverage_bits_ = 0;
  n_inlier_frames_ = 0;
}

std::vector<Eigen::Vector3d> MultiviewPointBuffer::points() const
{
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(voxels_.size());
  for (const auto & kv : voxels_) {
    pts.push_back(kv.second);
  }
  return pts;
}

bool MultiviewPointBuffer::should_saturate(double diameter_var) const
{
  return !saturated_ && coverage_ratio() >= params_.saturation_coverage &&
         n_inlier_frames_ >= params_.min_inlier_frames &&
         diameter_var <= params_.saturation_max_diameter_var;
}

}  // namespace forest_tree_slam

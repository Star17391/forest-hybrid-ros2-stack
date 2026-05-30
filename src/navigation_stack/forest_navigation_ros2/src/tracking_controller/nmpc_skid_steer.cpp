#include "forest_navigation_ros2/tracking_controller/nmpc_skid_steer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#ifdef FOREST_NAV_ENABLE_NMPC
extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_skid_steer_kin.h"
}
#endif

namespace forest_navigation_ros2
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double yaw_from_quat(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double normalize_angle(double a)
{
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a < -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

double distance_xy(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double path_length(const nav_msgs::msg::Path & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    len += distance_xy(path.poses[i - 1].pose.position, path.poses[i].pose.position);
  }
  return len;
}

double remaining_path_length(const nav_msgs::msg::Path & path, size_t from_idx)
{
  double len = 0.0;
  for (size_t i = from_idx + 1; i < path.poses.size(); ++i) {
    len += distance_xy(path.poses[i - 1].pose.position, path.poses[i].pose.position);
  }
  return len;
}

size_t advance_along_path(
  const nav_msgs::msg::Path & path,
  size_t start_idx,
  double distance_m)
{
  if (path.poses.empty()) {
    return 0;
  }
  size_t idx = std::min(start_idx, path.poses.size() - 1);
  double remaining = distance_m;
  while (remaining > 0.0 && idx + 1 < path.poses.size()) {
    const double seg = distance_xy(
      path.poses[idx].pose.position,
      path.poses[idx + 1].pose.position);
    if (seg <= 1e-9) {
      ++idx;
      continue;
    }
    if (remaining <= seg) {
      return idx + 1;
    }
    remaining -= seg;
    ++idx;
  }
  return path.poses.size() - 1;
}

size_t closest_index(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path,
  size_t start_idx)
{
  size_t best = std::min(start_idx, path.poses.size() - 1);
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = start_idx; i < path.poses.size(); ++i) {
    const double d = distance_xy(pose.pose.position, path.poses[i].pose.position);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}
}  // namespace

#ifdef FOREST_NAV_ENABLE_NMPC
struct NmpcSkidSteerController::Impl
{
  skid_steer_kin_solver_capsule * capsule{nullptr};
  ocp_nlp_config * nlp_config{nullptr};
  ocp_nlp_dims * nlp_dims{nullptr};
  ocp_nlp_in * nlp_in{nullptr};
  ocp_nlp_out * nlp_out{nullptr};
  int N{SKID_STEER_KIN_N};
  double dt{SKID_STEER_KIN_N > 0 ? 2.0 / SKID_STEER_KIN_N : 0.08};
};
#else
struct NmpcSkidSteerController::Impl
{};
#endif

NmpcSkidSteerController::NmpcSkidSteerController(const Params & params)
: impl_(std::make_unique<Impl>()),
  params_(params),
  fallback_(
    1.2, params.max_linear_vel, params.max_angular_vel,
    params.goal_tolerance_m, params.goal_heading_tolerance_rad)
{
#ifdef FOREST_NAV_ENABLE_NMPC
  impl_->capsule = skid_steer_kin_acados_create_capsule();
  if (!impl_->capsule) {
    last_error_ = "acados create_capsule failed";
    return;
  }
  const int status = skid_steer_kin_acados_create(impl_->capsule);
  if (status != 0) {
    last_error_ = "acados create failed status=" + std::to_string(status);
    return;
  }
  impl_->nlp_config = skid_steer_kin_acados_get_nlp_config(impl_->capsule);
  impl_->nlp_dims = skid_steer_kin_acados_get_nlp_dims(impl_->capsule);
  impl_->nlp_in = skid_steer_kin_acados_get_nlp_in(impl_->capsule);
  impl_->nlp_out = skid_steer_kin_acados_get_nlp_out(impl_->capsule);
  impl_->N = SKID_STEER_KIN_N;
  impl_->dt = params_.horizon_tf / static_cast<double>(impl_->N);
#else
  last_error_ = "NMPC not compiled (run scripts/generate_nmpc_solver.sh && rebuild)";
#endif
}

NmpcSkidSteerController::~NmpcSkidSteerController()
{
#ifdef FOREST_NAV_ENABLE_NMPC
  if (impl_ && impl_->capsule) {
    skid_steer_kin_acados_free(impl_->capsule);
    skid_steer_kin_acados_free_capsule(impl_->capsule);
  }
#endif
}

bool NmpcSkidSteerController::is_ready() const
{
#ifdef FOREST_NAV_ENABLE_NMPC
  return impl_ && impl_->capsule && impl_->nlp_in && last_error_.empty();
#else
  return false;
#endif
}

std::string NmpcSkidSteerController::last_error() const
{
  return last_error_;
}

void NmpcSkidSteerController::reset()
{
  last_closest_idx_ = 0;
  have_prev_u_ = false;
  prev_v_ = 0.0;
  prev_w_ = 0.0;
  fallback_.reset();
#ifdef FOREST_NAV_ENABLE_NMPC
  if (is_ready()) {
    skid_steer_kin_acados_reset(impl_->capsule, 1);
  }
#endif
}

PursuitCommand NmpcSkidSteerController::compute(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path)
{
  PursuitCommand metrics = fallback_.compute(pose, path);
  if (path.poses.size() < 2) {
    return metrics;
  }

#ifndef FOREST_NAV_ENABLE_NMPC
  metrics = fallback_.compute(pose, path);
  return metrics;
#else
  if (!is_ready()) {
    return fallback_.compute(pose, path);
  }

  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  const double psi = yaw_from_quat(
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);

  last_closest_idx_ = closest_index(pose, path, last_closest_idx_);

  const double x0[SKID_STEER_KIN_NX] = {px, py, psi};
  ocp_nlp_constraints_model_set(
    impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, impl_->nlp_out,
    0, "lbx", const_cast<double *>(x0));
  ocp_nlp_constraints_model_set(
    impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, impl_->nlp_out,
    0, "ubx", const_cast<double *>(x0));

  const double v_ref_nom = std::min(
    params_.max_linear_vel,
    std::max(0.15, metrics.remaining_path_m / std::max(params_.horizon_tf, 0.5)));

  for (int k = 0; k < impl_->N; ++k) {
    const double horizon_dist = v_ref_nom * impl_->dt * static_cast<double>(k + 1);
    const size_t idx = advance_along_path(path, last_closest_idx_, horizon_dist);
    const auto & ref_pose = path.poses[idx];
    const double ref_psi = yaw_from_quat(
      ref_pose.pose.orientation.x, ref_pose.pose.orientation.y,
      ref_pose.pose.orientation.z, ref_pose.pose.orientation.w);
    double p[SKID_STEER_KIN_NP] = {
      ref_pose.pose.position.x,
      ref_pose.pose.position.y,
      ref_psi,
      v_ref_nom,
    };
    skid_steer_kin_acados_update_params(impl_->capsule, k, p, SKID_STEER_KIN_NP);
  }

  const auto t0 = std::chrono::steady_clock::now();
  last_acados_status_ = skid_steer_kin_acados_solve(impl_->capsule);
  last_solve_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  if (last_acados_status_ != 0) {
    last_error_ = "acados solve status=" + std::to_string(last_acados_status_);
    return fallback_.compute(pose, path);
  }
  last_error_.clear();

  double u0[SKID_STEER_KIN_NU];
  ocp_nlp_out_get(
    impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, 0, "u", u0);

  metrics.linear_x = std::clamp(u0[0], 0.0, params_.max_linear_vel);
  metrics.angular_z = std::clamp(u0[1], -params_.max_angular_vel, params_.max_angular_vel);

  if (metrics.goal_reached) {
    metrics.linear_x = 0.0;
    metrics.angular_z = 0.0;
  }

  prev_v_ = metrics.linear_x;
  prev_w_ = metrics.angular_z;
  have_prev_u_ = true;
  return metrics;
#endif
}

}  // namespace forest_navigation_ros2

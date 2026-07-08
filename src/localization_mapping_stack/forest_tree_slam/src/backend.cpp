#include "forest_tree_slam/backend.hpp"

#include <cmath>
#include <stdexcept>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace forest_tree_slam
{

namespace
{
gtsam::Pose2 to_gtsam(const Pose2 & p) {return gtsam::Pose2(p.x, p.y, p.theta);}

Pose2 from_gtsam(const gtsam::Pose2 & p) {return Pose2{p.x(), p.y(), p.theta()};}

gtsam::noiseModel::Diagonal::shared_ptr diag3(const Eigen::Vector3d & sigma)
{
  return gtsam::noiseModel::Diagonal::Sigmas(sigma);
}

gtsam::noiseModel::Diagonal::shared_ptr diag2(const Eigen::Vector2d & sigma)
{
  return gtsam::noiseModel::Diagonal::Sigmas(sigma);
}
}  // namespace

TreeSlamBackend::TreeSlamBackend(BackendParams params)
: params_(params)
{
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_ = gtsam::ISAM2(isam_params);
}

gtsam::Key TreeSlamBackend::landmark_key(LandmarkUid uid) const
{
  return gtsam::Symbol('l', uid);
}

gtsam::SharedNoiseModel TreeSlamBackend::robustify(const gtsam::SharedNoiseModel & base) const
{
  if (!params_.use_robust_kernels) {
    return base;
  }
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(params_.robust_huber_k), base);
}

void TreeSlamBackend::initialize(const Pose2 & initial_pose)
{
  if (initialized_) {
    throw std::logic_error("TreeSlamBackend::initialize chamado mais que uma vez");
  }
  new_factors_.addPrior(pose_key(0), to_gtsam(initial_pose), diag3(params_.prior_pose_sigma));
  new_values_.insert(pose_key(0), to_gtsam(initial_pose));
  n_keyframes_ = 1;
  initialized_ = true;
  dirty_ = true;
}

bool TreeSlamBackend::should_open_keyframe(const Pose2 & accumulated_delta) const
{
  const double dist = std::hypot(accumulated_delta.x, accumulated_delta.y);
  return dist >= params_.keyframe_distance_m ||
         std::abs(accumulated_delta.theta) >= params_.keyframe_angle_rad;
}

std::size_t TreeSlamBackend::add_odom_keyframe(
  const Pose2 & relative_pose, const std::optional<Eigen::Vector3d> & sigma)
{
  if (!initialized_) {
    throw std::logic_error("TreeSlamBackend::add_odom_keyframe antes de initialize()");
  }
  const std::size_t prev = n_keyframes_ - 1;
  const std::size_t next = n_keyframes_;

  const gtsam::Pose2 delta = to_gtsam(relative_pose);
  const auto noise = diag3(sigma.value_or(params_.default_odom_sigma));
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(pose_key(prev), pose_key(next), delta,
      noise));

  const gtsam::Pose2 prev_pose = current_estimate_.exists(pose_key(prev)) ?
    current_estimate_.at<gtsam::Pose2>(pose_key(prev)) :
    new_values_.at<gtsam::Pose2>(pose_key(prev));
  new_values_.insert(pose_key(next), prev_pose.compose(delta));

  n_keyframes_++;
  dirty_ = true;
  return next;
}

std::size_t TreeSlamBackend::add_aerial_hop_edge(
  const Pose2 & relative_pose, const Eigen::Vector3d & sigma)
{
  // Mesma mecânica que uma keyframe de odom — a diferença é semântica (cov
  // grande, vem do ArduPilot/EKF, não da roda) e está documentada no design
  // §6: "o voo inteiro vira UMA aresta SE2 entre a keyframe pré-takeoff e a
  // pós-aterragem".
  return add_odom_keyframe(relative_pose, sigma);
}

void TreeSlamBackend::add_tree_observation(
  LandmarkUid uid, std::size_t keyframe_index, double bearing_rad, double range_m,
  const Eigen::Vector2d & initial_guess_world,
  std::optional<double> bearing_sigma_rad, std::optional<double> range_sigma_m)
{
  const gtsam::Key lkey = landmark_key(uid);
  const bool is_new = landmark_keys_.find(uid) == landmark_keys_.end();

  const auto noise = robustify(
    gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector2(
        bearing_sigma_rad.value_or(params_.default_bearing_sigma_rad),
        range_sigma_m.value_or(params_.default_range_sigma_m))));

  new_factors_.add(
    gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
      pose_key(keyframe_index), lkey, gtsam::Rot2::fromAngle(bearing_rad), range_m, noise));

  if (is_new) {
    landmark_keys_[uid] = lkey;
    if (!current_estimate_.exists(lkey) && !new_values_.exists(lkey)) {
      new_values_.insert(lkey, gtsam::Point2(initial_guess_world));
    }
  }
  dirty_ = true;
}

void TreeSlamBackend::add_relocalization_factor(
  LandmarkUid uid, std::size_t keyframe_index, double bearing_rad, double range_m)
{
  // O landmark já existe no mapa (é um re-match do TreeLoc); usa a posição
  // conhecida como "initial_guess" — só relevante se por algum motivo ainda
  // não tivesse sido inserido (não deve acontecer no caminho de loop closure).
  const Eigen::Vector2d guess =
    has_landmark(uid) ? landmark_position(uid) : Eigen::Vector2d::Zero();
  add_tree_observation(uid, keyframe_index, bearing_rad, range_m, guess);
}

void TreeSlamBackend::add_landmark_position_prior(
  LandmarkUid uid, const Eigen::Vector2d & xy_world, const Eigen::Vector2d & sigma)
{
  if (landmark_keys_.find(uid) == landmark_keys_.end()) {
    return;  // só landmarks já inseridos no grafo
  }
  const auto noise = robustify(
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(sigma.x(), sigma.y())));
  new_factors_.addPrior(landmark_key(uid), gtsam::Point2(xy_world.x(), xy_world.y()), noise);
  dirty_ = true;
}

void TreeSlamBackend::add_constellation_distance(
  LandmarkUid uid_a, LandmarkUid uid_b, double distance_m)
{
  if (landmark_keys_.find(uid_a) == landmark_keys_.end() ||
    landmark_keys_.find(uid_b) == landmark_keys_.end())
  {
    return;  // só liga troncos já conhecidos do grafo
  }
  const auto noise = robustify(
    gtsam::noiseModel::Isotropic::Sigma(1, params_.constellation_distance_sigma_m));
  new_factors_.add(
    gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>(
      landmark_key(uid_a), landmark_key(uid_b), distance_m, noise));
  dirty_ = true;
}

void TreeSlamBackend::add_weak_gps_prior(
  std::size_t keyframe_index, const Eigen::Vector2d & xy_world, const Eigen::Vector2d & sigma)
{
  // Prior de pose completo com sigma de theta enorme: ancora xy sem impor
  // heading (GPS não dá orientação). Evita precisar de um factor dedicado.
  const gtsam::Pose2 anchor(xy_world.x(), xy_world.y(), 0.0);
  const Eigen::Vector3d sigma3(sigma.x(), sigma.y(), 1.0e3);
  new_factors_.addPrior(pose_key(keyframe_index), anchor, diag3(sigma3));
  dirty_ = true;
}

bool TreeSlamBackend::optimize()
{
  if (!dirty_) {
    return false;
  }
  isam_.update(new_factors_, new_values_);
  // iSAM2 só relineariza parcialmente por update(); chamadas extra sem
  // fatores novos deixam-no convergir mais perto do óptimo do grafo atual
  // (relevante quando o "initial guess" de um landmark novo está longe da
  // verdade — caso comum em birth de tracks).
  for (int i = 0; i < 3; ++i) {
    isam_.update();
  }
  current_estimate_ = isam_.calculateEstimate();
  new_factors_.resize(0);
  new_values_.clear();
  dirty_ = false;
  return true;
}

Pose2 TreeSlamBackend::keyframe_pose(std::size_t index) const
{
  return from_gtsam(current_estimate_.at<gtsam::Pose2>(pose_key(index)));
}

Eigen::Vector2d TreeSlamBackend::landmark_position(LandmarkUid uid) const
{
  const auto it = landmark_keys_.find(uid);
  if (it == landmark_keys_.end()) {
    throw std::out_of_range("landmark_position: uid desconhecido");
  }
  const gtsam::Point2 p = current_estimate_.at<gtsam::Point2>(it->second);
  return Eigen::Vector2d(p.x(), p.y());
}

Eigen::Matrix3d TreeSlamBackend::landmark_covariance(LandmarkUid uid) const
{
  const auto it = landmark_keys_.find(uid);
  if (it == landmark_keys_.end()) {
    throw std::out_of_range("landmark_covariance: uid desconhecido");
  }
  Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
  try {
    gtsam::Marginals marginals(isam_.getFactorsUnsafe(), current_estimate_);
    out.topLeftCorner<2, 2>() = marginals.marginalCovariance(it->second);
  } catch (const std::exception &) {
    // Marginais podem falhar logo após update (grafo ainda não fatorizado
    // de forma adequada); devolve zeros — o chamador trata como "desconhecido".
  }
  return out;
}

std::vector<LandmarkUid> TreeSlamBackend::all_landmark_uids() const
{
  std::vector<LandmarkUid> out;
  out.reserve(landmark_keys_.size());
  for (const auto & kv : landmark_keys_) {
    out.push_back(kv.first);
  }
  return out;
}

}  // namespace forest_tree_slam

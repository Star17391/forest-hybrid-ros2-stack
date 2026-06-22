#pragma once

// Geometria SE(2) da cola do Tree-SLAM, ROS-agnóstica e header-only para ser
// partilhada pelo nó (tree_slam_node.cpp) e pelos testes. Centraliza aqui o
// que antes estava duplicado em ambos — em particular o cálculo do par
// (bearing, range) de uma observação, que tem de ser feito SEMPRE no frame da
// keyframe à qual o BearingRangeFactor vai ser ancorado (ver
// FOREST_TREE_SLAM_DESIGN.md §5.2 e o bug histórico documentado em
// test/test_node_glue.cpp).

#include <cmath>

#include <Eigen/Core>

#include "forest_tree_slam/types.hpp"

namespace forest_tree_slam
{

// Normaliza um ângulo para [-pi, pi[.
inline double wrap_angle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// Composição SE(2): pose `a` seguida do deslocamento local `b`.
inline Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  return Pose2{a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap_angle(a.theta + b.theta)};
}

// delta tal que compose(a, delta) == b  (b "menos" a, em SE(2)).
inline Pose2 between(const Pose2 & a, const Pose2 & b)
{
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  const double dx = b.x - a.x, dy = b.y - a.y;
  return Pose2{c * dx + s * dy, -s * dx + c * dy, wrap_angle(b.theta - a.theta)};
}

// Transforma um ponto (px, py) expresso no frame local de `pose` para o frame
// global. Usado para passar deteções de tronco de base_link -> mundo.
inline Eigen::Vector2d transform_point(const Pose2 & pose, double px, double py)
{
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  return Eigen::Vector2d(pose.x + c * px - s * py, pose.y + s * px + c * py);
}

struct BearingRange
{
  double bearing{0.0};  // rad, [-pi, pi[
  double range{0.0};    // m
};

// (bearing, range) de um ponto-mundo (wx, wy) TAL COMO VISTO de `ref_pose`.
// `ref_pose` TEM de ser a pose da keyframe à qual o BearingRangeFactor será
// ancorado — senão o fator fica inconsistente com a sua própria medição (o
// erro dominante de tracking corrigido nesta camada, ver test_node_glue.cpp).
inline BearingRange bearing_range_from(const Pose2 & ref_pose, double wx, double wy)
{
  const double dx = wx - ref_pose.x;
  const double dy = wy - ref_pose.y;
  return BearingRange{wrap_angle(std::atan2(dy, dx) - ref_pose.theta), std::hypot(dx, dy)};
}

// Interpolação de uma pose SE(2) no instante `t` entre (t0,p0) e (t1,p1).
// Translação linear; orientação pelo caminho angular mais curto. Para `t` fora
// de [t0,t1] devolve o extremo mais próximo (clamp), evitando extrapolação.
inline Pose2 interpolate_pose(
  const Pose2 & p0, double t0, const Pose2 & p1, double t1, double t)
{
  if (t1 <= t0) {
    return p1;  // intervalo degenerado: usa a amostra mais recente
  }
  double u = (t - t0) / (t1 - t0);
  if (u <= 0.0) {return p0;}
  if (u >= 1.0) {return p1;}
  const double dtheta = wrap_angle(p1.theta - p0.theta);
  return Pose2{
    p0.x + u * (p1.x - p0.x),
    p0.y + u * (p1.y - p0.y),
    wrap_angle(p0.theta + u * dtheta)};
}

}  // namespace forest_tree_slam

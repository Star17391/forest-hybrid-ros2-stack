#include "nav2_dstar_lite_planner/dstar_lite.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_dstar_lite_planner
{

namespace
{
constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr double kDiag = 1.4142135623730951;
}  // namespace

void DStarLite::reset(int width, int height)
{
  width_ = std::max(1, width);
  height_ = std::max(1, height);
  const int n = width_ * height_;
  costs_.assign(n, 0);
  g_.assign(n, kInf);
  rhs_.assign(n, kInf);
  while (!open_.empty()) {
    open_.pop();
  }
  changed_.clear();
  goal_idx_ = -1;
  start_idx_ = -1;
  km_ = 0.0;
  initialized_ = false;
}

bool DStarLite::in_bounds(int x, int y) const
{
  return x >= 0 && y >= 0 && x < width_ && y < height_;
}

int DStarLite::index(int x, int y) const
{
  return y * width_ + x;
}

void DStarLite::set_goal(int gx, int gy)
{
  if (!in_bounds(gx, gy)) {
    return;
  }
  const int new_goal = index(gx, gy);
  if (new_goal != goal_idx_) {
    goal_idx_ = new_goal;
    initialized_ = false;
  }
}

void DStarLite::set_start(int sx, int sy)
{
  if (!in_bounds(sx, sy)) {
    return;
  }
  start_idx_ = index(sx, sy);
}

void DStarLite::set_cell_cost(int x, int y, unsigned char cost)
{
  if (!in_bounds(x, y)) {
    return;
  }
  const int idx = index(x, y);
  if (costs_[idx] != cost) {
    costs_[idx] = cost;
    changed_.insert(idx);
  }
}

void DStarLite::mark_changed(int x, int y)
{
  if (in_bounds(x, y)) {
    changed_.insert(index(x, y));
  }
}

double DStarLite::cost_at(int x, int y) const
{
  const unsigned char c = costs_[index(x, y)];
  if (c >= kLethal) {
    return kInf;
  }
  return 1.0 + static_cast<double>(c) / 252.0;
}

double DStarLite::heuristic(int x, int y) const
{
  if (start_idx_ < 0) {
    return 0.0;
  }
  const int sx = start_idx_ % width_;
  const int sy = start_idx_ / width_;
  return std::hypot(static_cast<double>(x - sx), static_cast<double>(y - sy));
}

double DStarLite::g_at(int x, int y) const
{
  if (!in_bounds(x, y)) {
    return kInf;
  }
  return g_[index(x, y)];
}

DStarLite::Key DStarLite::calculate_key(int idx) const
{
  const int x = idx % width_;
  const int y = idx / width_;
  const double min_g_rhs = std::min(g_[idx], rhs_[idx]);
  return {min_g_rhs + heuristic(x, y) + km_, min_g_rhs};
}

double DStarLite::edge_cost(int from_idx, int to_idx) const
{
  const int fx = from_idx % width_;
  const int fy = from_idx / width_;
  const int tx = to_idx % width_;
  const int ty = to_idx / width_;
  const double step = (fx != tx && fy != ty) ? kDiag : 1.0;
  const double c_to = cost_at(tx, ty);
  if (!std::isfinite(c_to)) {
    return kInf;
  }
  return step * c_to;
}

void DStarLite::update_vertex(int idx)
{
  if (idx == goal_idx_) {
    return;
  }

  const int x = idx % width_;
  const int y = idx / width_;
  double min_rhs = kInf;
  for (int d = 0; d < 8; ++d) {
    const int nx = x + kDx[d];
    const int ny = y + kDy[d];
    if (!in_bounds(nx, ny)) {
      continue;
    }
    const int nidx = index(nx, ny);
    const double c = edge_cost(idx, nidx);
    if (!std::isfinite(c)) {
      continue;
    }
    min_rhs = std::min(min_rhs, g_[nidx] + c);
  }
  rhs_[idx] = min_rhs;

  // Remove stale queue entries lazily (standard D* Lite practice).
  if (g_[idx] != rhs_[idx]) {
    open_.push({calculate_key(idx), idx});
  }
}

void DStarLite::initialize()
{
  const int n = width_ * height_;
  std::fill(g_.begin(), g_.end(), kInf);
  std::fill(rhs_.begin(), rhs_.end(), kInf);
  while (!open_.empty()) {
    open_.pop();
  }
  km_ = 0.0;

  if (goal_idx_ < 0 || start_idx_ < 0) {
    return;
  }

  rhs_[goal_idx_] = 0.0;
  open_.push({calculate_key(goal_idx_), goal_idx_});
  initialized_ = true;
}

void DStarLite::apply_cost_changes()
{
  if (!initialized_) {
    return;
  }
  for (const int idx : changed_) {
    update_vertex(idx);
    const int x = idx % width_;
    const int y = idx / width_;
    for (int d = 0; d < 8; ++d) {
      const int nx = x + kDx[d];
      const int ny = y + kDy[d];
      if (!in_bounds(nx, ny)) {
        continue;
      }
      update_vertex(index(nx, ny));
    }
  }
  changed_.clear();
}

void DStarLite::compute_shortest_path_inner()
{
  if (!initialized_ || goal_idx_ < 0 || start_idx_ < 0) {
    return;
  }

  const Key start_key = calculate_key(start_idx_);
  while (!open_.empty()) {
    const QueueEntry top = open_.top();
    if (top.key < start_key && rhs_[start_idx_] == g_[start_idx_]) {
      break;
    }

    open_.pop();
    const Key k_old = top.key;
    const int u = top.idx;
    const Key k_new = calculate_key(u);
    if (k_old < k_new) {
      open_.push({k_new, u});
      continue;
    }

    if (g_[u] > rhs_[u]) {
      g_[u] = rhs_[u];
      const int ux = u % width_;
      const int uy = u / width_;
      for (int d = 0; d < 8; ++d) {
        const int nx = ux + kDx[d];
        const int ny = uy + kDy[d];
        if (!in_bounds(nx, ny)) {
          continue;
        }
        update_vertex(index(nx, ny));
      }
    } else {
      g_[u] = kInf;
      const int ux = u % width_;
      const int uy = u / width_;
      update_vertex(u);
      for (int d = 0; d < 8; ++d) {
        const int nx = ux + kDx[d];
        const int ny = uy + kDy[d];
        if (!in_bounds(nx, ny)) {
          continue;
        }
        update_vertex(index(nx, ny));
      }
    }
  }
}

bool DStarLite::compute_shortest_path(std::function<bool()> cancel_checker)
{
  if (goal_idx_ < 0 || start_idx_ < 0) {
    return false;
  }

  if (!initialized_) {
    initialize();
  } else {
    apply_cost_changes();
    if (start_idx_ >= 0) {
      const int sx = start_idx_ % width_;
      const int sy = start_idx_ / width_;
      km_ += heuristic(sx, sy);
    }
  }

  compute_shortest_path_inner();

  if (cancel_checker && cancel_checker()) {
    return false;
  }

  return std::isfinite(g_[start_idx_]);
}

bool DStarLite::extract_path(std::vector<Cell> & path) const
{
  path.clear();
  if (start_idx_ < 0 || goal_idx_ < 0 || !std::isfinite(g_[start_idx_])) {
    return false;
  }

  int cur = start_idx_;
  path.push_back({cur % width_, cur / width_});
  std::vector<int> visited;
  visited.push_back(cur);

  while (cur != goal_idx_ && static_cast<int>(path.size()) < width_ * height_) {
    const int cx = cur % width_;
    const int cy = cur / width_;
    int best = -1;
    double best_cost = kInf;
    for (int d = 0; d < 8; ++d) {
      const int nx = cx + kDx[d];
      const int ny = cy + kDy[d];
      if (!in_bounds(nx, ny)) {
        continue;
      }
      const int nidx = index(nx, ny);
      const double c = edge_cost(cur, nidx);
      if (!std::isfinite(c)) {
        continue;
      }
      const double total = g_[nidx] + c;
      if (total < best_cost) {
        best_cost = total;
        best = nidx;
      }
    }
    if (best < 0 || best == cur) {
      return false;
    }
    if (std::find(visited.begin(), visited.end(), best) != visited.end()) {
      return false;
    }
    cur = best;
    visited.push_back(cur);
    path.push_back({cur % width_, cur / width_});
  }

  return cur == goal_idx_;
}

}  // namespace nav2_dstar_lite_planner

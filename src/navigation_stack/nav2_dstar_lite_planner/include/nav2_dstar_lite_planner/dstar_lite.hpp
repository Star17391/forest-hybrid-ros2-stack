// D* Lite grid search (Koenig & Likhachev, 2002) — adapted for Nav2 costmaps.
#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_HPP_

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

namespace nav2_dstar_lite_planner
{

class DStarLite
{
public:
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  static constexpr unsigned char kLethal = 254;

  struct Cell
  {
    int x{0};
    int y{0};
  };

  void reset(int width, int height);

  void set_goal(int gx, int gy);
  void set_start(int sx, int sy);

  void set_cell_cost(int x, int y, unsigned char cost);
  void mark_changed(int x, int y);

  bool compute_shortest_path(std::function<bool()> cancel_checker);
  bool extract_path(std::vector<Cell> & path) const;

  double g_at(int x, int y) const;
  bool in_bounds(int x, int y) const;
  int index(int x, int y) const;

private:
  struct Key
  {
    double k1{0.0};
    double k2{0.0};
    bool operator>(const Key & o) const
    {
      if (k1 != o.k1) {
        return k1 > o.k1;
      }
      return k2 > o.k2;
    }
    bool operator<(const Key & o) const
    {
      if (k1 != o.k1) {
        return k1 < o.k1;
      }
      return k2 < o.k2;
    }
  };

  struct QueueEntry
  {
    Key key;
    int idx{0};
    bool operator>(const QueueEntry & o) const { return key > o.key; }
  };

  double heuristic(int x, int y) const;
  double edge_cost(int from_idx, int to_idx) const;
  double cost_at(int x, int y) const;
  Key calculate_key(int idx) const;
  void update_vertex(int idx);
  void compute_shortest_path_inner();
  void initialize();
  void apply_cost_changes();

  int width_{0};
  int height_{0};
  int goal_idx_{-1};
  int start_idx_{-1};
  double km_{0.0};

  std::vector<unsigned char> costs_;
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_;
  std::unordered_set<int> changed_;
  bool initialized_{false};
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_HPP_

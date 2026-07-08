// Teste ISOLADO do core D* Lite (sem ROS/costmap). Exit 0 = passou.
//
// Valida o MECANISMO do bug encontrado + a correção do allow_unknown:
//   - kLethal=254; NO_INFORMATION(255) >= kLethal → o core trata 255 como LETAL.
//   - Logo o planner TEM de converter 255: allow_unknown=true → LIVRE(0); senão
//     → letal. Sem isto, um costmap por-povoar (todo 255) fica todo bloqueado e o
//     planeamento falha ("no path found") mesmo em espaço aberto.
//
// Casos:
//   [aberto]     grelha toda livre → encontra caminho (sanidade).
//   [unknown-ok] grelha toda 255, convertida com allow_unknown=true (→0) → caminho. (A CORREÇÃO)
//   [unknown-no] grelha toda 255, convertida com allow_unknown=false (→letal) → SEM caminho. (conservador)
//   [parede]     coluna letal com abertura → caminho; sem abertura → SEM caminho.
#include <cstdio>
#include <functional>
#include <vector>

#include "nav2_dstar_lite_planner/dstar_lite.hpp"

using nav2_dstar_lite_planner::DStarLite;

namespace
{
// Espelha a conversão do planner (dstar_lite_planner.cpp sync_costmap).
unsigned char conv(unsigned char c, bool allow_unknown)
{
  if (c == 255) {
    return allow_unknown ? 0 : DStarLite::kLethal;
  }
  return c;
}

// Planeia numa grelha w×h com custos `cells` (já convertidos), start→goal.
bool plan(int w, int h, const std::vector<unsigned char> & cells,
  int sx, int sy, int gx, int gy)
{
  DStarLite d;
  d.reset(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      d.set_cell_cost(x, y, cells[static_cast<std::size_t>(y) * w + x]);
    }
  }
  d.set_goal(gx, gy);
  d.set_start(sx, sy);
  return d.compute_shortest_path([]() {return false;});
}

std::vector<unsigned char> fill(int w, int h, unsigned char v)
{
  return std::vector<unsigned char>(static_cast<std::size_t>(w) * h, v);
}
}  // namespace

int main()
{
  int fail = 0;
  const int W = 12, H = 12;

  // [aberto] tudo livre → caminho.
  if (!plan(W, H, fill(W, H, 0), 0, 0, W - 1, H - 1)) {
    std::printf("[FALHA aberto] grelha livre devia ter caminho\n"); ++fail;
  } else {
    std::printf("[ok aberto] grelha livre → caminho\n");
  }

  // [extract] o caminho extraído começa no START, acaba no GOAL e tem vários pontos
  // (um caminho degenerado faria o controlador pensar que já chegou → não anda).
  {
    DStarLite d; d.reset(W, H);
    for (int y = 0; y < H; ++y) {for (int x = 0; x < W; ++x) {d.set_cell_cost(x, y, 0);}}
    d.set_goal(W - 1, H - 1);
    d.set_start(0, 0);
    d.compute_shortest_path([]() {return false;});
    std::vector<DStarLite::Cell> path;
    const bool ok = d.extract_path(path);
    const bool good = ok && path.size() > 2 &&
      path.front().x == 0 && path.front().y == 0 &&
      path.back().x == W - 1 && path.back().y == H - 1;
    if (!good) {
      std::printf("[FALHA extract] caminho degenerado: ok=%d n=%zu", ok, path.size());
      if (!path.empty()) {
        std::printf(" front=(%d,%d) back=(%d,%d)", path.front().x, path.front().y,
          path.back().x, path.back().y);
      }
      std::printf(" (esperado front=(0,0) back=(%d,%d))\n", W - 1, H - 1); ++fail;
    } else {
      std::printf("[ok extract] caminho %zu pts de (0,0) a (%d,%d)\n", path.size(), W - 1, H - 1);
    }
  }

  // [unknown-ok] tudo 255 convertido com allow_unknown=true → caminho (A CORREÇÃO).
  {
    auto c = fill(W, H, 255);
    for (auto & v : c) {v = conv(v, true);}
    if (!plan(W, H, c, 0, 0, W - 1, H - 1)) {
      std::printf("[FALHA unknown-ok] 255+allow_unknown devia ser traversável\n"); ++fail;
    } else {
      std::printf("[ok unknown-ok] 255 com allow_unknown → LIVRE → caminho (fix)\n");
    }
  }

  // [unknown-no] tudo 255 convertido com allow_unknown=false → letal → SEM caminho.
  {
    auto c = fill(W, H, 255);
    for (auto & v : c) {v = conv(v, false);}
    if (plan(W, H, c, 0, 0, W - 1, H - 1)) {
      std::printf("[FALHA unknown-no] 255 sem allow_unknown devia ser letal (sem caminho)\n"); ++fail;
    } else {
      std::printf("[ok unknown-no] 255 sem allow_unknown → letal → sem caminho\n");
    }
  }

  // [parede] coluna letal em x=6 com abertura em y=0 → caminho.
  {
    auto c = fill(W, H, 0);
    for (int y = 1; y < H; ++y) {c[static_cast<std::size_t>(y) * W + 6] = DStarLite::kLethal;}
    if (!plan(W, H, c, 0, 5, W - 1, 5)) {
      std::printf("[FALHA parede-gap] devia contornar pela abertura\n"); ++fail;
    } else {
      std::printf("[ok parede-gap] contorna a parede pela abertura\n");
    }
    // sem abertura → sem caminho.
    auto c2 = fill(W, H, 0);
    for (int y = 0; y < H; ++y) {c2[static_cast<std::size_t>(y) * W + 6] = DStarLite::kLethal;}
    if (plan(W, H, c2, 0, 5, W - 1, 5)) {
      std::printf("[FALHA parede-cheia] parede completa não devia ter caminho\n"); ++fail;
    } else {
      std::printf("[ok parede-cheia] parede completa → sem caminho\n");
    }
  }

  std::printf(fail == 0 ? "PASSOU\n" : "FALHOU (%d)\n", fail);
  return fail == 0 ? 0 : 1;
}

#pragma once

#include <cmath>
#include <limits>
#include <vector>

namespace forest_tree_slam
{

// Hungarian algorithm (Jonker-Volgenant com potenciais, O(n^2 * m)) para
// atribuição retangular que MINIMIZA o custo total. n_rows <= n_cols.
// `cost[i][j]` pode ser std::numeric_limits<double>::infinity() para
// proibir o par (i,j) — usado pelo gate de Mahalanobis do tracker.
//
// Devolve, por linha i, o índice de coluna atribuído (ou -1 se nenhuma
// atribuição viável existir para essa linha — todas as colunas estavam a
// infinito). Linhas/colunas "fictícias" não existem: o chamador decide o
// que fazer com não-associados (birth/death), tipicamente acrescentando
// linhas ou colunas de custo infinito para limitar o gate, não para
// forçar atribuição.
inline std::vector<int> hungarian_assign(const std::vector<std::vector<double>> & cost)
{
  const int n_rows = static_cast<int>(cost.size());
  if (n_rows == 0) {
    return {};
  }
  const int n_cols = static_cast<int>(cost.front().size());
  if (n_cols == 0) {
    return std::vector<int>(n_rows, -1);
  }

  constexpr double kInf = std::numeric_limits<double>::infinity();
  // Algoritmo assume n_rows <= n_cols; se não for o caso, transpõe-se a
  // matriz e inverte-se a leitura do resultado no final.
  const bool transposed = n_rows > n_cols;
  const int n = transposed ? n_cols : n_rows;       // linhas do problema interno
  const int m = transposed ? n_rows : n_cols;       // colunas do problema interno

  auto at = [&](int i, int j) -> double {
      return transposed ? cost[j][i] : cost[i][j];
    };

  // Substitui infinitos por um valor finito grande mas seguro para a
  // aritmética do algoritmo (que faz somas/subtrações de potenciais).
  double finite_max = 0.0;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      const double c = at(i, j);
      if (std::isfinite(c) && c > finite_max) {
        finite_max = c;
      }
    }
  }
  const double big = finite_max * 4.0 + 1.0e6;

  // Implementação clássica O(n^2*m) com potenciais (cp-algorithms).
  std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
  std::vector<int> p(m + 1, 0);     // p[j] = linha (1-indexada) atualmente atribuída à coluna j
  std::vector<int> way(m + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(m + 1, kInf);
    std::vector<bool> used(m + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = kInf;
      int j1 = -1;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) {
          continue;
        }
        double raw = at(i0 - 1, j - 1);
        if (!std::isfinite(raw)) {
          raw = big;
        }
        const double cur = raw - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  // result_internal[row 0-indexed] = col 0-indexed assignment, ou -1.
  std::vector<int> result_internal(n, -1);
  for (int j = 1; j <= m; ++j) {
    if (p[j] != 0) {
      result_internal[p[j] - 1] = j - 1;
    }
  }

  // Filtra atribuições que caíram num par originalmente infinito (o `big`
  // é só um truque numérico do algoritmo — não é uma atribuição válida).
  for (int i = 0; i < n; ++i) {
    const int j = result_internal[i];
    if (j >= 0 && !std::isfinite(at(i, j))) {
      result_internal[i] = -1;
    }
  }

  if (!transposed) {
    return result_internal;
  }

  // transposto: result_internal está indexado por coluna original (m=n_rows
  // do problema externo? não: aqui n=n_cols_orig, m=n_rows_orig). Inverte.
  std::vector<int> result(n_rows, -1);
  for (int j_internal = 0; j_internal < n; ++j_internal) {
    const int i_internal = result_internal[j_internal];
    if (i_internal >= 0) {
      result[i_internal] = j_internal;
    }
  }
  return result;
}

}  // namespace forest_tree_slam

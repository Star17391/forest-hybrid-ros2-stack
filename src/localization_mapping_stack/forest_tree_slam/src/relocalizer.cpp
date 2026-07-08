#include "forest_tree_slam/relocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>

namespace forest_tree_slam
{

namespace
{
double dist2d(const LandmarkPoint & a, const LandmarkPoint & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Kabsch 2D: melhor rotação+translação tal que dst_i ~= R*src_i + t.
// Devolve Pose2{tx, ty, theta}. Pré-condição: src.size()==dst.size()>=2.
Pose2 align_se2(const std::vector<Eigen::Vector2d> & src, const std::vector<Eigen::Vector2d> & dst)
{
  const std::size_t n = src.size();
  Eigen::Vector2d centroid_src = Eigen::Vector2d::Zero();
  Eigen::Vector2d centroid_dst = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    centroid_src += src[i];
    centroid_dst += dst[i];
  }
  centroid_src /= static_cast<double>(n);
  centroid_dst /= static_cast<double>(n);

  Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    h += (src[i] - centroid_src) * (dst[i] - centroid_dst).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d r = svd.matrixV() * svd.matrixU().transpose();
  if (r.determinant() < 0.0) {
    Eigen::Matrix2d v = svd.matrixV();
    v.col(1) *= -1.0;
    r = v * svd.matrixU().transpose();
  }
  const Eigen::Vector2d t = centroid_dst - r * centroid_src;
  return Pose2{t.x(), t.y(), std::atan2(r(1, 0), r(0, 0))};
}

Eigen::Vector2d transform_point(const Pose2 & p, const Eigen::Vector2d & v)
{
  const double c = std::cos(p.theta);
  const double s = std::sin(p.theta);
  return Eigen::Vector2d(c * v.x() - s * v.y() + p.x, s * v.x() + c * v.y() + p.y);
}
}  // namespace

TreeLocRelocalizer::Descriptor TreeLocRelocalizer::compute_tdh(
  const std::vector<LandmarkPoint> & cluster, const std::vector<std::size_t> & members) const
{
  Descriptor hist(
    static_cast<std::size_t>(params_.n_radial_bins) *
    static_cast<std::size_t>(params_.n_diameter_bins), 0.0);

  double total = 0.0;
  for (const auto i : members) {
    for (const auto j : members) {
      if (i == j) {
        continue;
      }
      const double d = dist2d(cluster[i], cluster[j]);
      const double dbh = cluster[j].diameter;
      int rb = static_cast<int>(d / params_.radial_bin_max_m * params_.n_radial_bins);
      int db = static_cast<int>(dbh / params_.diameter_bin_max_m * params_.n_diameter_bins);
      rb = std::clamp(rb, 0, params_.n_radial_bins - 1);
      db = std::clamp(db, 0, params_.n_diameter_bins - 1);
      hist[static_cast<std::size_t>(rb * params_.n_diameter_bins + db)] += 1.0;
      total += 1.0;
    }
  }
  if (total > 0.0) {
    for (auto & v : hist) {
      v /= total;
    }
  }
  return hist;
}

double TreeLocRelocalizer::chi_square_distance(const Descriptor & a, const Descriptor & b) const
{
  double d = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double denom = a[i] + b[i];
    if (denom > 1e-12) {
      const double diff = a[i] - b[i];
      d += diff * diff / denom;
    }
  }
  return d;
}

std::vector<TreeLocRelocalizer::Triangle> TreeLocRelocalizer::build_triangles(
  const std::vector<LandmarkPoint> & points, const std::vector<std::size_t> & subset) const
{
  std::vector<Triangle> triangles;
  const std::size_t n = subset.size();
  if (n < 3) {
    return triangles;
  }
  for (std::size_t a = 0; a < n; ++a) {
    for (std::size_t b = a + 1; b < n; ++b) {
      for (std::size_t c = b + 1; c < n; ++c) {
        const std::size_t ia = subset[a], ib = subset[b], ic = subset[c];
        // lado oposto a cada vértice = distância entre os OUTROS dois.
        const double side_a = dist2d(points[ib], points[ic]);  // oposto a ia
        const double side_b = dist2d(points[ia], points[ic]);  // oposto a ib
        const double side_c = dist2d(points[ia], points[ib]);  // oposto a ic
        if (side_a < params_.min_triangle_side_m || side_b < params_.min_triangle_side_m ||
          side_c < params_.min_triangle_side_m)
        {
          continue;  // triângulo degenerado (troncos quase coincidentes)
        }
        std::array<std::pair<double, std::size_t>, 3> v = {
          std::make_pair(side_a, ia), std::make_pair(side_b, ib), std::make_pair(side_c, ic)};
        std::sort(v.begin(), v.end(), [](const auto & x, const auto & y) {
            return x.first < y.first;
        });
        Triangle tri;
        for (int k = 0; k < 3; ++k) {
          tri.sides[static_cast<std::size_t>(k)] = v[static_cast<std::size_t>(k)].first;
          tri.indices[static_cast<std::size_t>(k)] = v[static_cast<std::size_t>(k)].second;
        }
        triangles.push_back(tri);
      }
    }
  }
  return triangles;
}

RelocalizationResult TreeLocRelocalizer::relocalize(
  const std::vector<LandmarkPoint> & query, const std::vector<LandmarkPoint> & map) const
{
  RelocalizationResult result;
  if (static_cast<int>(query.size()) < params_.min_correspondences || map.size() < 3) {
    return result;
  }

  // --- Coarse: TDH da cena query vs vizinhanças candidatas no mapa --------
  std::vector<std::size_t> query_all(query.size());
  std::iota(query_all.begin(), query_all.end(), 0);
  const Descriptor query_tdh = compute_tdh(query, query_all);

  // O raio de vizinhança usado para recortar candidatos no mapa tem de
  // corresponder à EXTENSÃO REAL da cena query, não ao alcance do histograma
  // (`radial_bin_max_m`, que só define a largura dos bins). Usar um raio fixo
  // grande faria todos os candidatos numa grelha uniforme parecerem iguais
  // (sem poder discriminativo) e desalinhados em escala com a query.
  Eigen::Vector2d query_centroid = Eigen::Vector2d::Zero();
  for (const auto i : query_all) {
    query_centroid += Eigen::Vector2d(query[i].x, query[i].y);
  }
  query_centroid /= static_cast<double>(query_all.size());
  double scene_radius = 2.0;
  for (const auto i : query_all) {
    scene_radius = std::max(
      scene_radius, (Eigen::Vector2d(query[i].x, query[i].y) - query_centroid).norm());
  }
  scene_radius *= 1.3;  // margem para tolerar jitter/erro de deteção

  struct Candidate
  {
    std::vector<std::size_t> members;
    double chi2;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(map.size());
  for (std::size_t c = 0; c < map.size(); ++c) {
    std::vector<std::size_t> members;
    for (std::size_t m = 0; m < map.size(); ++m) {
      if (dist2d(map[c], map[m]) <= scene_radius) {
        members.push_back(m);
      }
    }
    if (members.size() < 3) {
      continue;
    }
    const Descriptor d = compute_tdh(map, members);
    candidates.push_back({members, chi_square_distance(query_tdh, d)});
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      return a.chi2 < b.chi2;
  });
  const std::size_t n_coarse =
    std::min(static_cast<std::size_t>(std::max(0, params_.top_n_coarse)), candidates.size());

  // --- Fine: triângulos query vs cada candidato -> hipóteses de transformação.
  // Cada PAR de triângulos compatível (3 lados dentro da tolerância) é
  // internamente consistente por construção, mas em floresta densa há
  // constelações ambíguas: vários pares podem coincidir por acaso fora da
  // região certa. Em vez de votar correspondências individuais (que mistura
  // pares certos com errados antes de verificar geometria — falha observada
  // empiricamente com troncos próximos), cada par gera uma HIPÓTESE de
  // transformação SE2 (3 pontos -> 3 pontos) e só se mantém a hipótese com
  // mais apoio geométrico de TODA a query (consenso estilo RANSAC; é a
  // "supressão de outliers" do TreeLoc++ aplicada à origem da ambiguidade,
  // não só ao resultado final).
  const auto query_triangles = build_triangles(query, query_all);

  Pose2 best_transform{};
  std::vector<ReloCorrespondence> best_inliers;
  // 2.ª melhor hipótese DISTINTA (guarda nº5): se outra transformação diferente
  // tiver apoio quase igual, o match é ambíguo e não se aceita.
  std::size_t second_best_size = 0;
  const auto same_cluster = [this](const Pose2 & a, const Pose2 & b) {
      const double dt = std::hypot(a.x - b.x, a.y - b.y);
      double dth = std::abs(a.theta - b.theta);
      dth = std::min(dth, 2.0 * M_PI - dth);
      return dt <= params_.distinct_transform_translation_m &&
             dth <= params_.distinct_transform_rotation_rad;
    };

  std::size_t hypotheses_evaluated = 0;
  constexpr std::size_t kMaxHypotheses = 2000;  // limite de custo (engenharia, não tese)
  for (std::size_t k = 0; k < n_coarse; ++k) {
    const auto map_triangles = build_triangles(map, candidates[k].members);
    for (const auto & tq : query_triangles) {
      for (const auto & tm : map_triangles) {
        if (hypotheses_evaluated >= kMaxHypotheses) {
          break;
        }
        bool ok = true;
        for (int s = 0; s < 3; ++s) {
          if (std::abs(
            tq.sides[static_cast<std::size_t>(s)] - tm.sides[static_cast<std::size_t>(s)]) >
            params_.triangle_side_tolerance_m)
          {
            ok = false;
            break;
          }
        }
        if (!ok) {
          continue;
        }
        ++hypotheses_evaluated;

        std::vector<Eigen::Vector2d> hyp_src(3), hyp_dst(3);
        for (int s = 0; s < 3; ++s) {
          hyp_src[static_cast<std::size_t>(s)] = Eigen::Vector2d(
            query[tq.indices[static_cast<std::size_t>(s)]].x,
            query[tq.indices[static_cast<std::size_t>(s)]].y);
          hyp_dst[static_cast<std::size_t>(s)] = Eigen::Vector2d(
            map[tm.indices[static_cast<std::size_t>(s)]].x,
              map[tm.indices[static_cast<std::size_t>(s)]].y);
        }
        const Pose2 hyp = align_se2(hyp_src, hyp_dst);

        // Consenso: quantos pontos da query, transformados por esta hipótese,
        // caem perto de ALGUM landmark do mapa com DBH compatível?
        // UNICIDADE: cada landmark do mapa só pode ser reclamado por UMA
        // deteção (greedy por ordem da query) — sem isto, duas deteções a
        // colar no mesmo landmark inflacionavam os inliers e hipóteses
        // ambíguas passavam o gate.
        std::vector<ReloCorrespondence> inliers;
        std::set<LandmarkUid> claimed;
        for (std::size_t qi = 0; qi < query.size(); ++qi) {
          const Eigen::Vector2d predicted =
            transform_point(hyp, Eigen::Vector2d(query[qi].x, query[qi].y));
          double best_d = std::numeric_limits<double>::infinity();
          LandmarkUid best_uid = 0;
          for (const auto & m : map) {
            if (claimed.count(m.uid) > 0) {
              continue;
            }
            const double d = (predicted - Eigen::Vector2d(m.x, m.y)).norm();
            if (d < best_d) {
              best_d = d;
              best_uid = m.uid;
            }
          }
          if (best_d <= params_.planar_residual_threshold_m) {
            const auto map_it = std::find_if(
              map.begin(), map.end(), [&](const LandmarkPoint & p) {return p.uid == best_uid;});
            const double diam_residual = std::abs(map_it->diameter - query[qi].diameter);
            if (diam_residual <= params_.diameter_residual_threshold_m) {
              inliers.push_back({qi, best_uid});
              claimed.insert(best_uid);
            }
          }
        }
        if (inliers.size() > best_inliers.size()) {
          // A anterior melhor passa a 2.ª melhor SE for de um cluster distinto
          // (a mesma transformação a melhorar não conta como ambiguidade).
          if (!best_inliers.empty() && !same_cluster(hyp, best_transform)) {
            second_best_size = std::max(second_best_size, best_inliers.size());
          }
          best_inliers = inliers;
          best_transform = hyp;
        } else if (!inliers.empty() && !same_cluster(hyp, best_transform)) {
          second_best_size = std::max(second_best_size, inliers.size());
        }
      }
      if (hypotheses_evaluated >= kMaxHypotheses) {
        break;
      }
    }
    if (hypotheses_evaluated >= kMaxHypotheses) {
      break;
    }
  }

  last_best_inliers_ = best_inliers;
  if (static_cast<int>(best_inliers.size()) < params_.min_correspondences) {
    return result;
  }

  // Refinamento final: SVD com TODOS os inliers da melhor hipótese.
  std::vector<Eigen::Vector2d> final_src, final_dst;
  for (const auto & c : best_inliers) {
    final_src.push_back(Eigen::Vector2d(query[c.query_index].x, query[c.query_index].y));
    const auto map_it = std::find_if(
      map.begin(), map.end(), [&](const LandmarkPoint & p) {return p.uid == c.map_uid;});
    final_dst.push_back(Eigen::Vector2d(map_it->x, map_it->y));
  }
  const Pose2 refined = align_se2(final_src, final_dst);

  double residual_sum = 0.0;
  for (std::size_t i = 0; i < final_src.size(); ++i) {
    residual_sum += (transform_point(refined, final_src[i]) - final_dst[i]).norm();
  }
  result.mean_residual_m = residual_sum / static_cast<double>(final_src.size());
  result.overlap_ratio =
    static_cast<double>(best_inliers.size()) / static_cast<double>(query.size());

  if (result.overlap_ratio < params_.min_overlap_ratio ||
    static_cast<int>(best_inliers.size()) < params_.min_correspondences)
  {
    return result;
  }

  // Guarda nº5: margem clara ao 2.º melhor cluster distinto. Sem margem, o
  // match é ambíguo (floresta auto-semelhante) → recusar; aceitar errado é
  // pior do que falhar (o fator errado puxa o grafo e arrasta o terreno).
  if (second_best_size > 0 &&
    best_inliers.size() <
    second_best_size + static_cast<std::size_t>(std::max(0, params_.accept_margin_inliers)))
  {
    return result;
  }

  result.accepted = true;
  result.map_to_query_transform = refined;
  result.correspondences = best_inliers;
  return result;
}

}  // namespace forest_tree_slam

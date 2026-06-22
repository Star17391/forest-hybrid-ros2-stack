/**
 * @file stem_band_clustering.hpp
 * @brief Stem-band 2D clustering of non-ground points (experimental).
 *
 * Replaces naive Euclidean 3D clustering over the whole non-ground cloud, which
 * merges touching canopies and mixes trunk+canopy+undergrowth into one blob.
 *
 * Robotic-forestry consensus (TreeSLAM, 3DFin, DigiForest): cluster only the
 * TRUNK BAND in 2D, not the whole tree in 3D.
 *
 *   ground (CSF)      → CsfGroundGrid (min-Z per cell)
 *   non-ground (CSF)  → nDSM height above ground
 *                     → keep band [band_min, band_max]   (drops canopy that glues trees)
 *                     → 2D XY Euclidean clustering         (each trunk = compact core)
 *
 * Output reuses PointCluster (id, point_indices into non-ground, cloud) so the
 * node's existing marker/labeled-cloud publishers work unchanged.
 *
 * References (local): docs/perception/references/2024_arxiv_treeslam_summary.md,
 *   2025_isprs_3dfin_pipeline_notes.md, FORESTRY_CLUSTERING_LITERATURE.md §4.4.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_BAND_CLUSTERING_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_BAND_CLUSTERING_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "forest_3d_perception/experimental/csf_ground_grid.hpp"
#include "forest_3d_perception/experimental/euclidean_clustering.hpp"

namespace forest_3d_perception::experimental
{

struct StemBandParams
{
  float ground_grid_resolution_m{0.20f};
  float band_min_m{0.30f};
  float band_max_m{3.00f};
  float cluster_tolerance_m{0.20f};
  int cluster_min_pts{6};
  int cluster_max_pts{3000};
  // Filtro de colunas verticais (SÓ caminho SEM solo). Sem o corte de banda
  // relativa ao solo, a copa fica nos pontos sem-solo e, ao colapsar em 2D, as
  // copas de árvores vizinhas fazem PONTE num blob enorme. Este filtro mantém só
  // pontos cuja vizinhança LOCAL é vertical e linear (superfície de fuste) e
  // larga a copa/folhas (disperso/horizontal), ANTES do clustering 2D.
  float vcol_radius_m{0.40f};       // raio da vizinhança local p/ PCA
  int vcol_min_neighbors{4};        // mínimo de vizinhos p/ PCA fiável
  float vcol_min_verticality{0.60f};// |eixo dominante · z| ≥ isto → coluna vertical
  float vcol_min_linearity{0.25f};  // (l0-l1)/l0 ≥ isto → anisotrópico (não disperso)
};

struct StemBandResult
{
  std::vector<PointCluster> clusters;       // point_indices reference the non-ground cloud
  std::size_t n_non_ground_in{0};
  std::size_t n_band_points{0};             // points surviving the nDSM band
  std::size_t n_ground_cells{0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr band_cloud;  // optional debug: the band points (3D)
};

/**
 * Points surviving the nDSM trunk band. `indices` reference the non-ground
 * cloud; `cloud` holds the matching 3D points (aligned 1:1 with `indices`) so
 * callers can run per-point analysis (e.g. linearity) and split the band before
 * clustering.
 */
struct BandExtraction
{
  std::vector<std::size_t> indices;                // into the non-ground cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;       // 3D band points, aligned with indices
};

class StemBandClusterer
{
public:
  StemBandParams params;

  /**
   * Step 1 — extract the nDSM trunk band: non-ground points whose height above
   * the CSF ground is within [band_min, band_max]. Returns indices into the
   * non-ground cloud plus the matching 3D points, so callers can analyse and
   * split the band before clustering.
   */
  BandExtraction extract_band(
    const pcl::PointCloud<pcl::PointXYZ> & ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud) const
  {
    BandExtraction band;
    band.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    CsfGroundGrid grid;
    grid.resolution_m = params.ground_grid_resolution_m;
    grid.build(ground_cloud);
    if (grid.empty() || non_ground_cloud.empty()) {
      return band;
    }

    band.indices.reserve(non_ground_cloud.size());
    for (std::size_t i = 0; i < non_ground_cloud.size(); ++i) {
      const auto & p = non_ground_cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        continue;
      }
      const float h = grid.height_above_ground(p.x, p.y, p.z);
      if (std::isnan(h) || h < params.band_min_m || h > params.band_max_m) {
        continue;
      }
      band.indices.push_back(i);
      band.cloud->push_back(p);
    }
    band.cloud->width = static_cast<std::uint32_t>(band.cloud->size());
    band.cloud->height = 1;
    band.cloud->is_dense = true;
    return band;
  }

  /**
   * Step 1b (caminho SEM solo) — o COMPLEMENTO de extract_band: pontos não-solo
   * SEM referência de solo por baixo (HAG = NaN), que o extract_band larga em
   * silêncio. Com o LiDAR inclinado, atrás/ao lado do robô não há retornos de
   * solo, logo troncos reais ficam com HAG=NaN e desaparecem — é o que faz o
   * SLAM ir a LOST numa travessia. Estes pontos seguem para o caminho de deteção
   * por ESTRUTURA (verticalidade/forma, sem nDSM); a base fica indefinida (cota
   * incerta), mas o raio/DBH não depende dela. Índices para a nuvem não-solo.
   *
   * NÃO altera extract_band (caminho com solo intocado): é puramente aditivo.
   */
  BandExtraction extract_no_ground(
    const pcl::PointCloud<pcl::PointXYZ> & ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud) const
  {
    BandExtraction out;
    out.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (non_ground_cloud.empty()) {
      return out;
    }
    CsfGroundGrid grid;
    grid.resolution_m = params.ground_grid_resolution_m;
    grid.build(ground_cloud);
    const bool no_grid = grid.empty();  // zero solo em lado nenhum -> tudo é "sem solo"
    out.indices.reserve(non_ground_cloud.size());
    for (std::size_t i = 0; i < non_ground_cloud.size(); ++i) {
      const auto & p = non_ground_cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        continue;
      }
      const bool no_ground =
        no_grid || std::isnan(grid.height_above_ground(p.x, p.y, p.z));
      if (no_ground) {
        out.indices.push_back(i);
        out.cloud->push_back(p);
      }
    }
    out.cloud->width = static_cast<std::uint32_t>(out.cloud->size());
    out.cloud->height = 1;
    out.cloud->is_dense = true;
    return out;
  }

  /**
   * Step 1c (caminho SEM solo) — filtro de COLUNAS VERTICAIS. Substitui o corte
   * de banda (que precisa de solo) por um critério puramente GEOMÉTRICO: por cada
   * ponto candidato, PCA da vizinhança local (raio `vcol_radius_m`); mantém só os
   * pontos cujo eixo dominante é VERTICAL (|v0·z| ≥ vcol_min_verticality) e cuja
   * vizinhança é LINEAR/anisotrópica (vcol_min_linearity) — i.e. superfície de
   * fuste. Copa e folhagem (dispersas, sem eixo vertical dominante) são largadas,
   * o que impede a ponte 2D entre copas. Grid hash O(N). `subset` e o retorno são
   * índices para a nuvem não-solo. NÃO toca no caminho com solo.
   */
  std::vector<std::size_t> filter_vertical_columns(
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud,
    const std::vector<std::size_t> & subset) const
  {
    std::vector<std::size_t> out;
    const float r = std::max(params.vcol_radius_m, 0.02f);
    if (subset.size() < static_cast<std::size_t>(params.vcol_min_neighbors)) {
      return out;
    }
    using Key = std::array<int, 3>;
    auto cell = [r](const pcl::PointXYZ & p) -> Key {
      return {static_cast<int>(std::floor(p.x / r)),
              static_cast<int>(std::floor(p.y / r)),
              static_cast<int>(std::floor(p.z / r))};
    };
    std::map<Key, std::vector<std::size_t>> grid;
    for (std::size_t orig : subset) {
      grid[cell(non_ground_cloud.points[orig])].push_back(orig);
    }
    out.reserve(subset.size());
    for (std::size_t orig : subset) {
      const auto & p = non_ground_cloud.points[orig];
      const Key k0 = cell(p);
      std::vector<std::size_t> nb;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            auto it = grid.find({k0[0] + dx, k0[1] + dy, k0[2] + dz});
            if (it == grid.end()) {
              continue;
            }
            for (std::size_t j : it->second) {
              const auto & q = non_ground_cloud.points[j];
              const float d = std::hypot(std::hypot(q.x - p.x, q.y - p.y), q.z - p.z);
              if (d <= r) {
                nb.push_back(j);
              }
            }
          }
        }
      }
      if (static_cast<int>(nb.size()) < params.vcol_min_neighbors) {
        continue;
      }
      double mx = 0, my = 0, mz = 0;
      for (std::size_t j : nb) {
        mx += non_ground_cloud.points[j].x;
        my += non_ground_cloud.points[j].y;
        mz += non_ground_cloud.points[j].z;
      }
      const double in = 1.0 / static_cast<double>(nb.size());
      mx *= in; my *= in; mz *= in;
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (std::size_t j : nb) {
        const double ex = non_ground_cloud.points[j].x - mx;
        const double ey = non_ground_cloud.points[j].y - my;
        const double ez = non_ground_cloud.points[j].z - mz;
        cov(0, 0) += ex * ex; cov(0, 1) += ex * ey; cov(0, 2) += ex * ez;
        cov(1, 1) += ey * ey; cov(1, 2) += ey * ez; cov(2, 2) += ez * ez;
      }
      cov(1, 0) = cov(0, 1); cov(2, 0) = cov(0, 2); cov(2, 1) = cov(1, 2);
      cov *= in;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      if (es.info() != Eigen::Success) {
        continue;
      }
      const Eigen::Vector3d ev = es.eigenvalues();  // ascendente: ev(0)≤ev(1)≤ev(2)
      const double l0 = std::max(ev(2), 1e-9);
      const double l1 = std::max(ev(1), 0.0);
      const float vert = static_cast<float>(std::abs(es.eigenvectors().col(2).normalized().z()));
      const float lin = static_cast<float>((l0 - l1) / l0);
      if (vert >= params.vcol_min_verticality && lin >= params.vcol_min_linearity) {
        out.push_back(orig);
      }
    }
    return out;
  }

  /**
   * Step 2 — 2D XY Euclidean clustering of a SUBSET of band points (`subset`
   * holds indices into the non-ground cloud). Z is collapsed so proximity is
   * purely horizontal. Cluster ids start at `id_offset` so two disjoint subsets
   * (e.g. trunk set and the rest) get globally-unique ids when merged.
   */
  std::vector<PointCluster> cluster_band_subset(
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud,
    const std::vector<std::size_t> & subset,
    int id_offset) const
  {
    std::vector<PointCluster> clusters;
    if (subset.size() < static_cast<std::size_t>(params.cluster_min_pts)) {
      return clusters;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr flat(new pcl::PointCloud<pcl::PointXYZ>);
    flat->reserve(subset.size());
    for (std::size_t bi : subset) {
      pcl::PointXYZ p2d;
      p2d.x = non_ground_cloud.points[bi].x;
      p2d.y = non_ground_cloud.points[bi].y;
      p2d.z = 0.0f;
      flat->push_back(p2d);
    }
    flat->width = static_cast<std::uint32_t>(flat->size());
    flat->height = 1;
    flat->is_dense = true;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(flat);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(static_cast<double>(params.cluster_tolerance_m));
    ec.setMinClusterSize(params.cluster_min_pts);
    ec.setMaxClusterSize(params.cluster_max_pts);
    ec.setSearchMethod(tree);
    ec.setInputCloud(flat);
    ec.extract(cluster_indices);

    int id = id_offset;
    clusters.reserve(cluster_indices.size());
    for (const auto & ci : cluster_indices) {
      PointCluster c;
      c.id = id++;
      c.point_indices.reserve(ci.indices.size());
      c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      c.cloud->reserve(ci.indices.size());
      for (int fi : ci.indices) {
        const std::size_t orig = subset[static_cast<std::size_t>(fi)];
        c.point_indices.push_back(orig);
        c.cloud->push_back(non_ground_cloud.points[orig]);
      }
      c.cloud->width = static_cast<std::uint32_t>(c.cloud->size());
      c.cloud->height = 1;
      c.cloud->is_dense = true;
      clusters.push_back(std::move(c));
    }
    return clusters;
  }

  /** Convenience: extract the full band and cluster it in one shot. */
  StemBandResult cluster(
    const pcl::PointCloud<pcl::PointXYZ> & ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud) const
  {
    StemBandResult out;
    out.n_non_ground_in = non_ground_cloud.size();

    const BandExtraction band = extract_band(ground_cloud, non_ground_cloud);
    out.band_cloud = band.cloud;
    out.n_band_points = band.indices.size();
    out.clusters = cluster_band_subset(non_ground_cloud, band.indices, 0);
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_BAND_CLUSTERING_HPP_

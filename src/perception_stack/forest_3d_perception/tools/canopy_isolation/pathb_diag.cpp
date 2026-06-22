// Diagnóstico ISOLADO do caminho B (deteção SEM solo).
//
// Corre o caminho B REAL sobre nuvens ground/non_ground capturadas do mundo
// surround: extract_no_ground -> cluster_band_subset -> ClusterClassifier ->
// fit_vertical_cylinder. Despeja, por cluster, a geometria, a classe e o DBH —
// para CONFIRMAR (ou refutar) a hipótese de que a COPA contamina os clusters
// sem solo (sem corte de banda relativa ao solo).
//
// Uso: pathb_diag <ground.xyz> <nonground.xyz>
// Build: g++ -O2 -std=c++17 -I include -I /usr/include/pcl-1.14 -I /usr/include/eigen3
//        -lpcl_common -lpcl_search -lpcl_kdtree -lpcl_segmentation

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <vector>

#include <Eigen/Dense>

#include "forest_3d_perception/experimental/stem_band_clustering.hpp"
#include "forest_3d_perception/experimental/cluster_classifier.hpp"
#include "forest_3d_perception/cylinder_fit.hpp"

using namespace forest_3d_perception;
using namespace forest_3d_perception::experimental;

// PROTÓTIPO do filtro de colunas verticais (independente do solo):
// por cada ponto candidato, PCA da vizinhança local (raio r); mantém só pontos
// cujo eixo dominante é VERTICAL e a vizinhança é LINEAR (superfície de tronco).
// Copa/folhas (disperso, horizontal) é largada -> sem ponte 2D entre copas.
static std::vector<std::size_t> filter_vertical_columns(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std::vector<std::size_t> & subset,
  float r, int min_nb, float min_vert, float min_lin)
{
  using Key = std::array<int, 3>;
  std::map<Key, std::vector<std::size_t>> grid;  // voxel (lado r) -> índices em subset
  auto cell = [r](const pcl::PointXYZ & p) -> Key {
    return {int(std::floor(p.x / r)), int(std::floor(p.y / r)), int(std::floor(p.z / r))};
  };
  for (std::size_t k = 0; k < subset.size(); ++k) {
    grid[cell(cloud.points[subset[k]])].push_back(subset[k]);
  }
  std::vector<std::size_t> out;
  out.reserve(subset.size());
  for (std::size_t orig : subset) {
    const auto & p = cloud.points[orig];
    const Key k0 = cell(p);
    std::vector<std::size_t> nb;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          auto it = grid.find({k0[0] + dx, k0[1] + dy, k0[2] + dz});
          if (it != grid.end())
            for (std::size_t j : it->second)
              if (std::hypot(std::hypot(cloud.points[j].x - p.x, cloud.points[j].y - p.y),
                             cloud.points[j].z - p.z) <= r)
                nb.push_back(j);
        }
    if (static_cast<int>(nb.size()) < min_nb) continue;
    double mx = 0, my = 0, mz = 0;
    for (std::size_t j : nb) { mx += cloud.points[j].x; my += cloud.points[j].y; mz += cloud.points[j].z; }
    const double in = 1.0 / nb.size(); mx *= in; my *= in; mz *= in;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (std::size_t j : nb) {
      const double ex = cloud.points[j].x - mx, ey = cloud.points[j].y - my, ez = cloud.points[j].z - mz;
      cov(0, 0) += ex * ex; cov(0, 1) += ex * ey; cov(0, 2) += ex * ez;
      cov(1, 1) += ey * ey; cov(1, 2) += ey * ez; cov(2, 2) += ez * ez;
    }
    cov(1, 0) = cov(0, 1); cov(2, 0) = cov(0, 2); cov(2, 1) = cov(1, 2); cov *= in;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    if (es.info() != Eigen::Success) continue;
    const Eigen::Vector3d ev = es.eigenvalues();           // ascendente: ev(0)<=ev(1)<=ev(2)
    const double l0 = std::max(ev(2), 1e-9), l1 = std::max(ev(1), 0.0);
    const float vert = std::abs(es.eigenvectors().col(2).normalized().z());
    const float lin = static_cast<float>((l0 - l1) / l0);
    if (vert >= min_vert && lin >= min_lin) out.push_back(orig);
  }
  return out;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr load(const char * path)
{
  auto c = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::ifstream f(path);
  float x, y, z;
  while (f >> x >> y >> z) { c->push_back({x, y, z}); }
  c->width = c->size(); c->height = 1; c->is_dense = true;
  return c;
}

int main(int argc, char ** argv)
{
  if (argc < 3) { std::fprintf(stderr, "uso: %s ground.xyz nonground.xyz\n", argv[0]); return 2; }
  auto ground = load(argv[1]);
  auto non_ground = load(argv[2]);
  std::printf("ground=%zu  non_ground=%zu\n", ground->size(), non_ground->size());

  StemBandClusterer clus;  // params por omissão (iguais ao node)
  const BandExtraction ng = clus.extract_no_ground(*ground, *non_ground);
  std::printf("extract_no_ground -> %zu pontos sem solo\n", ng.indices.size());

  // Filtro de colunas verticais (parametrizável por env p/ afinar).
  const float R = std::getenv("VR") ? std::atof(std::getenv("VR")) : 0.25f;
  const int MN = std::getenv("VN") ? std::atoi(std::getenv("VN")) : 5;
  const float MV = std::getenv("VV") ? std::atof(std::getenv("VV")) : 0.70f;
  const float ML = std::getenv("VL") ? std::atof(std::getenv("VL")) : 0.30f;
  const bool use_filter = !std::getenv("RAW");
  std::vector<std::size_t> idx_for_cluster = ng.indices;
  if (use_filter) {
    idx_for_cluster = filter_vertical_columns(*non_ground, ng.indices, R, MN, MV, ML);
    std::printf("filtro vert (r=%.2f nb>=%d vert>=%.2f lin>=%.2f) -> %zu pontos\n",
                R, MN, MV, ML, idx_for_cluster.size());
  } else {
    std::printf("(SEM filtro -- caminho B atual)\n");
  }

  const std::vector<PointCluster> clusters =
    clus.cluster_band_subset(*non_ground, idx_for_cluster, 0);
  std::printf("cluster_band_subset -> %zu clusters\n\n", clusters.size());

  // Gate de emissão DEDICADO ao path B (não usa o scorer suave do path A).
  const float G_CORE = std::getenv("GCORE") ? std::atof(std::getenv("GCORE")) : 1.50f;
  const float G_DBHLO = 0.05f, G_DBHHI = std::getenv("GDBH") ? std::atof(std::getenv("GDBH")) : 0.60f;
  const float G_HSIZE = std::getenv("GHS") ? std::atof(std::getenv("GHS")) : 0.80f;
  int n_emit = 0;

  ClusterClassifier classifier;
  std::printf("%3s %5s %6s %6s %6s %6s %6s %7s %6s  %-8s  %6s %5s  %s\n",
              "id", "n", "az", "dist", "hspan", "hsize", "vert", "linear",
              "core", "classe", "DBH", "fit", "EMIT");
  for (const auto & c : clusters) {
    const ScoredCluster s = classifier.score_cluster(c, 0.0f);
    const auto & f = s.feat;
    const float az = std::atan2(f.centroid_y, f.centroid_x) * 180.0f / M_PI;
    const float dist = std::hypot(f.centroid_x, f.centroid_y);
    const ClusterClass cl = cluster_class_from_scores(s.class_scores);

    // cilindro
    std::vector<std::size_t> idx(c.cloud->size());
    for (std::size_t i = 0; i < idx.size(); ++i) { idx[i] = i; }
    CylinderObservation obs;
    const CylinderReject rej = fit_vertical_cylinder(
      *c.cloud, idx, obs, 0.3, 0.6, 0.08, 0.5, 0.05);
    const float dbh = (rej == CylinderReject::Accepted) ? 2.0f * obs.radius : -1.0f;
    const char * rejs = (rej == CylinderReject::Accepted) ? "ok" :
      (rej == CylinderReject::TooWide ? "wide" :
      (rej == CylinderReject::TooShort ? "short" :
      (rej == CylinderReject::HighRmse ? "rmse" :
      (rej == CylinderReject::LowInliers ? "inl" : "few"))));

    const bool emit = (rej == CylinderReject::Accepted) &&
                      (f.trunk_core_height >= G_CORE) &&
                      (dbh >= G_DBHLO && dbh <= G_DBHHI) &&
                      (f.horizontal_size <= G_HSIZE) && (f.verticality >= 0.70f) && (f.linearity >= 0.60f);
    if (emit) ++n_emit;
    std::printf("%3d %5d %6.0f %6.1f %6.2f %6.2f %6.2f %7.2f %6.2f  %-8s  %6.3f %5s  %s "
                " [T%.2f R%.2f O%.2f]\n",
                c.id, f.n_points, az, dist, f.height_span, f.horizontal_size,
                f.verticality, f.linearity, f.trunk_core_height,
                cluster_class_string(cl), dbh, rejs, emit ? "TRUNK*" : "-",
                s.class_scores[0], s.class_scores[1], s.class_scores[2]);
    (void)cl;
  }
  std::printf("\n== EMITIDOS como TRUNK (gate path B: core>=%.2f, DBH[%.2f,%.2f], hsize<=%.2f): %d ==\n",
              G_CORE, G_DBHLO, G_DBHHI, G_HSIZE, n_emit);
  return 0;
}

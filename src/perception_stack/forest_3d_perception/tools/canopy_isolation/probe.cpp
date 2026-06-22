// Probe da isolação de tronco: corre o fit_vertical_cylinder REAL (cylinder_fit.hpp)
// sobre uma nuvem rotulada (x y z label; 0=tronco 1=copa 2=solo) e mede quantos
// pontos de COPA contaminam a banda usada para o DBH.
//
// Alimenta todos os pontos NÃO-solo (tronco+copa) como UM cluster — exatamente o
// que o clustering produz para uma árvore — e testa se a seleção de banda os corta.
//
// Saída JSON numa linha: dbh, gt, erro, reject, n_band, copa_na_banda, ...
//
// Build: ver run_probe.sh (g++ -I include -I /usr/include/pcl-1.14 -I /usr/include/eigen3)

#include <cstdio>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include "forest_3d_perception/cylinder_fit.hpp"

using forest_3d_perception::CylinderObservation;
using forest_3d_perception::CylinderReject;
using forest_3d_perception::fit_vertical_cylinder;

static const char * reject_str(CylinderReject r)
{
  switch (r) {
    case CylinderReject::Accepted: return "Accepted";
    case CylinderReject::TooFewPoints: return "TooFewPoints";
    case CylinderReject::TooShort: return "TooShort";
    case CylinderReject::TooWide: return "TooWide";
    case CylinderReject::HighRmse: return "HighRmse";
    case CylinderReject::LowInliers: return "LowInliers";
  }
  return "?";
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "uso: %s <cloud.xyzl> <gt_diameter_m>\n", argv[0]);
    return 2;
  }
  const std::string path = argv[1];
  const double gt = std::stod(argv[2]);

  // --- params reais (config/lidar3d_experimental.yaml :: cyl_fit) ---
  const double min_height = 0.30, max_radius = 0.80, max_rmse = 0.05;
  const double min_inlier = 0.30, inlier_dist = 0.04, max_slice_h = 2.50;
  const double dbh_band_low = 0.15, dbh_band_high = 2.50;
  const double stem_grow = 1.8, stem_axis_jump = 0.20;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<int> labels;
  std::ifstream f(path);
  if (!f) { std::fprintf(stderr, "nao abriu %s\n", path.c_str()); return 2; }
  float x, y, z;
  int l;
  while (f >> x >> y >> z >> l) {
    if (l == 2) { continue; }  // solo: removido pela seg. de chão antes do clustering
    cloud.points.push_back(pcl::PointXYZ(x, y, z));
    labels.push_back(l);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;

  std::vector<std::size_t> idx(cloud.size());
  std::iota(idx.begin(), idx.end(), 0u);

  std::vector<std::size_t> band;
  CylinderObservation cyl;
  const auto reject = fit_vertical_cylinder(
    cloud, idx, cyl, min_height, max_radius, max_rmse, min_inlier, inlier_dist,
    max_slice_h, dbh_band_low, dbh_band_high, stem_grow, stem_axis_jump, &band);

  std::size_t band_trunk = 0, band_canopy = 0;
  for (std::size_t bi : band) {
    if (bi < labels.size()) {
      (labels[bi] == 0 ? band_trunk : band_canopy)++;
    }
  }

  // Dump opcional dos pontos selecionados para a banda (para visualização).
  if (argc >= 4) {
    std::ofstream bf(argv[3]);
    for (std::size_t bi : band) {
      if (bi < cloud.size()) {
        bf << cloud.points[bi].x << ' ' << cloud.points[bi].y << ' '
           << cloud.points[bi].z << ' ' << labels[bi] << '\n';
      }
    }
  }
  const double dbh = 2.0 * cyl.radius;
  const double err = dbh - gt;

  std::printf(
    "{\"reject\":\"%s\",\"dbh\":%.4f,\"gt\":%.4f,\"err\":%.4f,"
    "\"n_input\":%zu,\"n_band\":%zu,\"band_trunk\":%zu,\"band_canopy\":%zu,"
    "\"canopy_frac\":%.4f,\"radius\":%.4f,\"height\":%.3f,\"rmse\":%.4f,"
    "\"inlier_ratio\":%.3f,\"arc_cov\":%.3f,\"ref_radius\":%.3f,\"used_fallback\":%d,"
    "\"z_base\":%.3f,\"cx\":%.3f,\"cy\":%.3f}\n",
    reject_str(reject), dbh, gt, err, cloud.size(), band.size(),
    band_trunk, band_canopy,
    band.empty() ? 0.0 : double(band_canopy) / double(band.size()),
    cyl.radius, cyl.height, cyl.rmse, cyl.inlier_ratio, cyl.arc_coverage,
    cyl.ref_radius, cyl.used_fallback ? 1 : 0, cyl.z_base, cyl.cx, cyl.cy);
  return 0;
}

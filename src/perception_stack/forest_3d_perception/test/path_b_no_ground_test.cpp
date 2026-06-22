// Caminho B — deteção SEM solo (troncos atrás/ao lado do robô, onde o LiDAR
// inclinado não vê chão). Teste ISOLADO, sem ROS/sim. Exit 0 = passou.
//
// Reproduz a falha e valida a correção, reaproveitando o código REAL:
//   [baseline]     extract_band com solo VAZIO larga TUDO (HAG=NaN) -> tronco
//                  atrás invisível -> causa do LOST.
//   [filtro-copa]  SEM solo não há corte de banda, logo a COPA entra e funde-se
//                  com o tronco num blob largo. O filtro de colunas verticais
//                  remove a copa: o cluster resultante fica ESTREITO (vs largo
//                  sem filtro). É a contaminação que o utilizador viu.
//   [B-tronco]     o caminho B completo (filtro + gate estrito de cilindro/core/
//                  DBH/largura) EMITE um tronco limpo sem solo.
//   [B-rocha]      um blob baixo/largo/disperso NÃO é emitido (anti-lixo).
//   [B-copa-só]    uma COPA sozinha (sem fuste) NÃO gera falso positivo.
//
// `colcon test --packages-select forest_3d_perception`.
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <random>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/cylinder_fit.hpp"
#include "forest_3d_perception/experimental/cluster_classifier.hpp"
#include "forest_3d_perception/experimental/stem_band_clustering.hpp"

using forest_3d_perception::CylinderObservation;
using forest_3d_perception::CylinderReject;
using forest_3d_perception::fit_vertical_cylinder;
using forest_3d_perception::experimental::ClusterClassifier;
using forest_3d_perception::experimental::PointCluster;
using forest_3d_perception::experimental::StemBandClusterer;

namespace
{
// Tronco vertical visto pela frente (semicírculo esparso), SEM pontos de solo.
pcl::PointCloud<pcl::PointXYZ>::Ptr make_trunk(double cx, double cy, double r)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
  for (double z = 0.4; z <= 4.0; z += 0.10) {
    for (int i = 0; i < 6; ++i) {
      const double a = M_PI / 2 + M_PI * i / 5.0;  // arco virado ao sensor
      c->push_back({static_cast<float>(cx + r * std::cos(a)),
                    static_cast<float>(cy + r * std::sin(a)),
                    static_cast<float>(z)});
    }
  }
  c->width = c->size(); c->height = 1; c->is_dense = true;
  return c;
}

// Copa REALISTA: o LiDAR atinge a CASCA exterior da folhagem (superfície ~planar/
// tangente), não um volume uniformemente preenchido. Modela-se como casca de uma
// esfera (raio ~1.3 m, centro alto) com ruído radial e clumps de folha. A
// vizinhança local destes pontos é planar (linearidade baixa) e/ou sem eixo
// vertical dominante -> o filtro de colunas verticais deve largá-los. (Um box
// uniforme denso é IRREALISTA: fabrica "agulhas" verticais que nem o LiDAR vê.)
void add_canopy(pcl::PointCloud<pcl::PointXYZ> & c, double cx, double cy)
{
  std::mt19937 rng(11);
  std::normal_distribution<double> g(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const double Rc = 1.3, zc = 5.2;
  for (int i = 0; i < 400; ++i) {
    // direção aleatória na esfera; ruído radial fino (casca, não volume)
    double dx = g(rng), dy = g(rng), dz = g(rng);
    const double n = std::sqrt(dx * dx + dy * dy + dz * dz) + 1e-9;
    const double rad = Rc + 0.10 * g(rng);  // casca fina ±0.10 m
    dx = dx / n * rad; dy = dy / n * rad; dz = dz / n * rad;
    if (u(rng) < 0.05) { continue; }  // folhagem esparsa (oclusão/transparência)
    c.push_back({static_cast<float>(cx + dx), static_cast<float>(cy + dy),
                 static_cast<float>(zc + dz)});
  }
  c.width = c.size(); c.height = 1;
}

// Rocha: blob baixo, largo, volumétrico e disperso, SEM verticalidade.
pcl::PointCloud<pcl::PointXYZ>::Ptr make_rock(double cx, double cy)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 400; ++i) {
    const double dx = 0.5 * u(rng), dy = 0.5 * u(rng);
    const double z = 0.1 + 0.45 * (0.5 + 0.5 * u(rng));  // 0.1..0.55 m, baixo
    c->push_back({static_cast<float>(cx + dx), static_cast<float>(cy + dy),
                  static_cast<float>(z)});
  }
  c->width = c->size(); c->height = 1; c->is_dense = true;
  return c;
}

// Reproduz EXATAMENTE o gate de emissão do caminho B no node:
// extract_no_ground -> filter_vertical_columns -> cluster -> cilindro aceite +
// core vertical + DBH em gama + largura contida. Devolve nº de troncos emitidos.
int path_b_trunks_emitted(const pcl::PointCloud<pcl::PointXYZ> & non_ground)
{
  StemBandClusterer clusterer;
  pcl::PointCloud<pcl::PointXYZ> empty_ground;  // LiDAR inclinado: zero solo atrás
  const auto ng = clusterer.extract_no_ground(empty_ground, non_ground);
  const auto cols = clusterer.filter_vertical_columns(non_ground, ng.indices);
  if (cols.empty()) {
    return 0;
  }
  const auto clusters = clusterer.cluster_band_subset(non_ground, cols, 0);
  ClusterClassifier clf;  // defaults = config real do node
  int n = 0;
  for (const auto & c : clusters) {
    // Backstop de formação (igual ao node): exige extensão vertical mínima ->
    // mata "agulhas" curtas esculpidas de fragmentos de copa.
    float zlo = c.cloud->points.front().z, zhi = zlo;
    for (const auto & p : c.cloud->points) { zlo = std::min(zlo, p.z); zhi = std::max(zhi, p.z); }
    if ((zhi - zlo) < 1.50f) { continue; }
    const auto s = clf.score_cluster(c);
    std::vector<std::size_t> idx(c.cloud->size());
    std::iota(idx.begin(), idx.end(), 0u);
    CylinderObservation cyl;
    const auto rej = fit_vertical_cylinder(*c.cloud, idx, cyl, 0.3, 0.6, 0.08, 0.5, 0.05);
    const float dbh = (rej == CylinderReject::Accepted) ? 2.0f * cyl.radius : -1.0f;
    const bool pb = rej == CylinderReject::Accepted &&
      s.feat.trunk_core_height >= 1.50f &&
      dbh >= 0.05f && dbh <= 0.60f &&
      s.feat.horizontal_size <= 0.80f &&
      s.feat.verticality >= 0.70f &&
      s.feat.linearity >= 0.60f;
    if (pb) {
      ++n;
      if (std::getenv("PBDBG")) {
        std::printf("    EMIT n=%d hspan=%.2f hsize=%.2f vert=%.2f lin=%.2f "
          "scatter=%.3f surfvar=%.3f core=%.2f dbh=%.3f\n",
          s.feat.n_points, s.feat.height_span, s.feat.horizontal_size,
          s.feat.verticality, s.feat.linearity, s.feat.scatter,
          s.feat.surface_variation, s.feat.trunk_core_height, dbh);
      }
    }
  }
  return n;
}

// Maior largura horizontal entre os clusters obtidos do conjunto de índices dado.
float widest_cluster_hsize(
  const pcl::PointCloud<pcl::PointXYZ> & non_ground,
  const std::vector<std::size_t> & subset)
{
  StemBandClusterer clusterer;
  const auto clusters = clusterer.cluster_band_subset(non_ground, subset, 0);
  ClusterClassifier clf;
  float w = 0.0f;
  for (const auto & c : clusters) {
    const auto s = clf.score_cluster(c);
    w = std::max(w, s.feat.horizontal_size);
  }
  return w;
}
}  // namespace

int main()
{
  int failures = 0;
  StemBandClusterer clusterer;
  const auto trunk = make_trunk(5.0, 0.0, 0.18);

  // [baseline] com solo VAZIO, extract_band larga TUDO (HAG=NaN) -> causa do LOST.
  {
    pcl::PointCloud<pcl::PointXYZ> empty_ground;
    const auto band = clusterer.extract_band(empty_ground, *trunk);
    if (!band.indices.empty()) {
      std::printf("[FALHA baseline] extract_band devolveu %zu pts sem solo (esperado 0)\n",
        band.indices.size());
      ++failures;
    } else {
      std::printf("[ok baseline] sem solo, extract_band larga tudo (a falha que corrigimos)\n");
    }
  }

  // [filtro-copa] tronco + copa sem solo. Sem filtro -> blob LARGO (copa funde).
  // Com filtro de colunas verticais -> cluster ESTREITO (copa removida).
  {
    pcl::PointCloud<pcl::PointXYZ> tree = *trunk;  // tronco
    add_canopy(tree, 5.0, 0.0);                    // + copa por cima
    pcl::PointCloud<pcl::PointXYZ> empty_ground;
    const auto ng = clusterer.extract_no_ground(empty_ground, tree);
    const float w_raw = widest_cluster_hsize(tree, ng.indices);
    const auto cols = clusterer.filter_vertical_columns(tree, ng.indices);
    const float w_filt = widest_cluster_hsize(tree, cols);
    std::printf("   contaminacao copa: largura SEM filtro=%.2f m  COM filtro=%.2f m\n",
      w_raw, w_filt);
    if (!(w_raw > 1.5f)) {
      std::printf("[FALHA filtro-copa] sem filtro o blob devia ser LARGO (>1.5m), foi %.2f\n", w_raw);
      ++failures;
    } else if (!(w_filt < 0.8f)) {
      std::printf("[FALHA filtro-copa] com filtro o cluster devia ficar ESTREITO (<0.8m), foi %.2f\n",
        w_filt);
      ++failures;
    } else {
      std::printf("[ok filtro-copa] o filtro vertical remove a copa (%.2f -> %.2f m)\n",
        w_raw, w_filt);
    }
  }

  // [B-tronco] o caminho B completo EMITE o tronco (com ou sem copa por cima).
  {
    pcl::PointCloud<pcl::PointXYZ> tree = *trunk;
    add_canopy(tree, 5.0, 0.0);
    if (path_b_trunks_emitted(tree) < 1) {
      std::printf("[FALHA B-tronco] caminho B NAO emitiu o tronco sem solo (com copa)\n");
      ++failures;
    } else {
      std::printf("[ok B-tronco] caminho B emite o tronco limpo apesar da copa\n");
    }
  }

  // [B-rocha] uma rocha NAO pode ser emitida como tronco.
  {
    const auto rock = make_rock(5.0, 0.0);
    if (path_b_trunks_emitted(*rock) > 0) {
      std::printf("[FALHA B-rocha] caminho B emitiu uma ROCHA como tronco (lixo!)\n");
      ++failures;
    } else {
      std::printf("[ok B-rocha] caminho B rejeita a rocha\n");
    }
  }

  // [B-copa-só] uma COPA sozinha (sem fuste) NAO pode gerar falso positivo.
  {
    pcl::PointCloud<pcl::PointXYZ> canopy;
    add_canopy(canopy, 5.0, 0.0);
    if (path_b_trunks_emitted(canopy) > 0) {
      std::printf("[FALHA B-copa-so] caminho B emitiu uma COPA como tronco (falso positivo!)\n");
      ++failures;
    } else {
      std::printf("[ok B-copa-so] caminho B nao gera tronco a partir de copa sozinha\n");
    }
  }

  std::printf(failures == 0 ? "PASSOU\n" : "FALHOU (%d)\n", failures);
  return failures == 0 ? 0 : 1;
}

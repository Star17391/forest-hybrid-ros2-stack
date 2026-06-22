/**
 * Regression test (validação Fase 1): o scorer suave (P-A) não tem penhascos.
 *
 * O plano-mestre exige que uma feature a variar em torno do seu limiar mude a
 * pontuação de forma CONTÍNUA (sem salto) — é isso que mata o flip-flop. Aqui
 * varremos finamente `surface_variation` (a feature com a escala de sigmoide
 * mais apertada, s_s=0.02 em score_class_probs) através do limiar de rocha e
 * confirmamos que a variação de `s_rocha` entre passos adjacentes é pequena.
 * Um limiar duro daria um salto ~1.0 num único passo; um scorer suave dá <<1.
 */
#include <array>
#include <cmath>
#include <cstdio>

#include "forest_3d_perception/experimental/cluster_classifier.hpp"

using forest_3d_perception::experimental::ClassifierParams;
using forest_3d_perception::experimental::ClusterClassifier;
using forest_3d_perception::experimental::ClusterFeatures;
using forest_3d_perception::experimental::kScoreRock;

int main()
{
  ClassifierParams p{};

  // Geometria ambígua de objeto baixo/não-vertical: só varia a suavidade da
  // superfície, isolando a transição rocha↔obstáculo em torno do limiar.
  ClusterFeatures f{};
  f.verticality = 0.20f;
  f.linearity = 0.20f;
  f.trunk_core_height = 0.0f;
  f.local_roughness = 0.01f;
  f.height_span = 0.50f;

  constexpr float kStep = 0.0005f;   // passo fino em surface_variation
  constexpr float kStart = 0.02f;
  constexpr int kN = 400;            // varre 0.02..0.22, cobre thr=0.10
  constexpr float kMaxJump = 0.15f;  // muito abaixo do salto ~1.0 de um limiar duro

  float prev = -1.0f;
  float max_step = 0.0f;
  float sv_at_max = 0.0f;
  for (int i = 0; i <= kN; ++i) {
    f.surface_variation = kStart + kStep * static_cast<float>(i);
    const auto s = ClusterClassifier::score_class_probs(f, p, 0.0f);
    const float rock = s[kScoreRock];
    if (prev >= 0.0f) {
      const float d = std::fabs(rock - prev);
      if (d > max_step) {
        max_step = d;
        sv_at_max = f.surface_variation;
      }
    }
    prev = rock;
  }

  if (max_step > kMaxJump) {
    std::fprintf(
      stderr,
      "FAIL: scorer com penhasco residual — salto %.4f em s_rocha (passo=%.4f sv @ sv=%.3f)\n",
      max_step, kStep, sv_at_max);
    return 1;
  }

  std::printf(
    "PASS soft_scorer_continuity max_step=%.4f (sv=%.3f) << limiar duro (~1.0)\n",
    max_step, sv_at_max);
  return 0;
}

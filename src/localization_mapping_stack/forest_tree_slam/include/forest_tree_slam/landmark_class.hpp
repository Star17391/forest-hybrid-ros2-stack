#pragma once

#include <cstdint>

namespace forest_tree_slam
{

/** Índices em TreeLandmark.class_scores / TreeDetection.class_scores. */
constexpr std::size_t kScoreTrunk = 0;
constexpr std::size_t kScoreRock = 1;
constexpr std::size_t kScoreObstacle = 2;
constexpr std::size_t kNumClassScores = 3;

/** Classe comprometida no SLAM (0 = candidato/desconhecido). */
constexpr std::uint8_t kCommittedUnknown = 0;
constexpr std::uint8_t kCommittedTrunk = 1;
constexpr std::uint8_t kCommittedRock = 2;
constexpr std::uint8_t kCommittedObstacle = 3;

inline std::uint8_t committed_from_score_index(int index)
{
  return static_cast<std::uint8_t>(index + 1);
}

inline int score_index_from_committed(std::uint8_t committed)
{
  return static_cast<int>(committed) - 1;
}

inline bool is_promoted_class(std::uint8_t committed_class)
{
  return committed_class != kCommittedUnknown;
}

/** Tronco e rocha entram no pose-graph; obstáculo não. */
inline bool is_slam_graph_class(std::uint8_t committed_class)
{
  return committed_class == kCommittedTrunk || committed_class == kCommittedRock;
}

/** Saída `/slam/tree_map` — só landmarks SLAM (tronco/rocha). Obstáculo classifica-se
 *  internamente mas não entra no mapa deste nó (costmap usa a perceção). */
inline bool is_map_output_class(std::uint8_t committed_class)
{
  return is_slam_graph_class(committed_class);
}

}  // namespace forest_tree_slam

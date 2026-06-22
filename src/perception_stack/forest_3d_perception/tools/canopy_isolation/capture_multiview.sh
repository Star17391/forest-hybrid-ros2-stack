#!/usr/bin/env bash
# Grava um bag da camada SLAM (referência a crescer) + perceção, ENQUANTO o robô
# se move (chamar drive_pattern.sh em paralelo, ou usar run_multiview.sh que faz tudo).
#
# Acrescenta aos tópicos da perceção os do Tree-SLAM: o mapa seguido, o status e o
# pose_graph (que contém os PONTOS ACUMULADOS por landmark, ns tree_slam_landmark_points).
#
# Uso:  ./capture_multiview.sh <tag, ex. t1_d4_mv> [segundos=45]
set -euo pipefail

TAG="${1:?uso: capture_multiview.sh <tag> [segundos=45]}"
SECS="${2:-45}"
OUT_DIR="/home/star17391/Projetos/Tese/forest-hybrid-ros2-stack/src/perception_stack/forest_3d_perception/tools/canopy_isolation/bags"
BAG="${OUT_DIR}/mv_${TAG}"
mkdir -p "${OUT_DIR}"; rm -rf "${BAG}"

TOPICS=(
  /sensors/lidar/points
  /perception/lidar/tree_clusters
  /perception/lidar/tree_landmarks
  /perception/lidar3d/experimental/trunk_fit_points
  /slam/tree_map                 # mapa de troncos seguido (DBH multi-vista, n_obs)
  /slam/status
  /slam/pose_graph               # DEBUG: pontos acumulados por landmark + esferas + labels
  /state/pose_fused              # pose do EKF (IMU), frame map — usada pelo controlo da órbita
  /state/odometry
  /tf /tf_static
)

echo "A gravar ${SECS}s -> ${BAG}"
timeout --signal=SIGINT --kill-after=3 "${SECS}" \
  nice -n 10 ros2 bag record -o "${BAG}" "${TOPICS[@]}" || true
echo "feito: ${BAG}"
ros2 bag info "${BAG}" 2>/dev/null | grep -E "Duration|tree_map|pose_graph|tree_landmarks" || true

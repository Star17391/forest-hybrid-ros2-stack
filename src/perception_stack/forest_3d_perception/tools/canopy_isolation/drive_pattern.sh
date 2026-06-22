#!/usr/bin/env bash
# Move o robô em padrão controlado (Twist -> /forest_gen/cmd_vel) para dar MÚLTIPLAS
# VISTAS do mesmo tronco. O LiDAR é 360°, logo o que muda o arco visível do TRONCO é
# a POSIÇÃO do robô (não a orientação): tem de mudar de sítio para ver outros lados.
#
# Skid-steer: só linear.x (frente/trás) e angular.z (rodar). Não anda de lado.
# Padrão por defeito = arco suave contínuo (auto-círculo) -> a posição varre e o
# rumo ao tronco abre o arco acumulado. Termina sempre com Twist zero (pára o robô).
#
# Uso:  ./drive_pattern.sh <segundos> [v=0.30] [w=0.28]
set -uo pipefail

SECS="${1:-40}"; V="${2:-0.30}"; W="${3:-0.28}"
TOPIC=/forest_gen/cmd_vel

stop() { ros2 topic pub --once "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true; }
trap stop EXIT INT TERM

echo "drive: v=$V w=$W durante ${SECS}s (auto-círculo, raio~$(python3 -c "print(f'{$V/$W:.2f}')")m)"
timeout --signal=SIGINT "$SECS" \
  ros2 topic pub -r 10 "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: $V}, angular: {z: $W}}" >/dev/null 2>&1 || true
stop
echo "drive: fim (robô parado)"

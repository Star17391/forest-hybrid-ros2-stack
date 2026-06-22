#!/usr/bin/env bash
# Instala dependências Nav2 (ROS 2 Jazzy) para a camada de planeamento.
set -euo pipefail

PKGS=(
  ros-jazzy-nav2-bringup
  ros-jazzy-nav2-planner
  ros-jazzy-nav2-controller
  ros-jazzy-nav2-mppi-controller
  ros-jazzy-nav2-bt-navigator
  ros-jazzy-nav2-lifecycle-manager
  ros-jazzy-nav2-behaviors
  ros-jazzy-nav2-costmap-2d
  ros-jazzy-nav2-core
  ros-jazzy-nav2-common
  ros-jazzy-nav2-util
  ros-jazzy-nav2-smoother
  ros-jazzy-nav2-velocity-smoother
  ros-jazzy-nav2-map-server
  ros-jazzy-nav2-navfn-planner
  ros-jazzy-nav2-waypoint-follower
  ros-jazzy-nav2-behavior-tree
)

echo "A instalar Nav2 (${#PKGS[@]} pacotes)…"
sudo apt-get update -qq
sudo apt-get install -y "${PKGS[@]}"
echo "OK. Rebuild: cd forest-hybrid-ros2-stack && colcon build --packages-select nav2_dstar_lite_planner forest_nav2_bringup"

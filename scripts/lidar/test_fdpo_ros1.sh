#!/usr/bin/env bash
# Proven fdpo driver (ROS 1) for the same YDLidar X4 hardware — use if SDK turnOn keeps failing.
set -euo pipefail

FDPO_WS="${FDPO_CATKIN_WS:-$HOME/catkin_ws_fdpo}"
PORT="${1:-/dev/ttyUSB0}"

if [[ ! -f "$FDPO_WS/devel/setup.bash" ]]; then
  echo "ERROR: fdpo workspace not found at $FDPO_WS"
  echo "  Build fdpo-ros-stack in catkin, then:"
  echo "  export FDPO_CATKIN_WS=/path/to/catkin_ws"
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash
source "$FDPO_WS/devel/setup.bash"
set -u

echo "fdpo sdpo_driver_laser_2d (ROS 1) — port=$PORT"
echo "Check: rostopic hz /laser_scan_point_cloud"
echo "Hear motor spin? Then hardware is OK; tune ydlidar ROS2 params or use ros1_bridge later."
exec roslaunch sdpo_driver_laser_2d sdpo_driver_laser_2d_YDLIDARX4.launch \
  serial_port_name:="$PORT"

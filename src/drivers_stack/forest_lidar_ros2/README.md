# `forest_lidar_ros2` — YDLidar X4 (2D)

Same sensor family as **fdpo-ros-stack** package `sdpo_driver_laser_2d` (model `ydlidarx4`, `/dev/ttyUSB0`, **128000** baud, 0.12–10 m).

| fdpo (ROS 1) | forest-hybrid (ROS 2) |
|--------------|------------------------|
| `sdpo_driver_laser_2d` | `ydlidar_ros2_driver` |
| `/laser_scan_point_cloud` (`PointCloud`) | `/scan` (`LaserScan`) + optional `/sensors/lidar/points` (`PointCloud2`) |
| `base_link` → `laser` | static TF (yaw configurable) |

## Install driver (once)

There is **no** `ros-jazzy-ydlidar-ros2-driver` Debian package. Build from source:

```bash
bash scripts/lidar/install_driver.sh
```

This installs **YDLidar-SDK** to `/usr/local` and clones `src/external/ydlidar_ros2_driver` (branch `humble`, works on Jazzy).

Add user to `dialout`: `sudo usermod -aG dialout $USER` (re-login).

## Test

```bash
cd forest-hybrid-ros2-stack
source install/setup.bash
ls /dev/ttyUSB*    # confirm port
bash scripts/lidar/test_ydlidar_sdk.sh
```

**Success looks like:** `Lidar successfully connected`, `Model: X4`, `Lidar init success`, and `/scan -> /sensors/lidar/points`.

While it runs (before Ctrl+C), in another terminal:

```bash
ros2 topic hz /scan
ros2 topic hz /sensors/lidar/points
rviz2   # Fixed Frame: laser, add LaserScan + PointCloud2
```

### Motor not spinning / `Failed to start the lidar`

The **fdpo** stack (`sdpo_driver_laser_2d`) starts the X4 with stop + retry (`0xA5 0x60`); the **YDLidar SDK** path is different. Try in order:

1. Forest launch with updated yaml (`support_motor_dtr: false`):
   ```bash
   bash scripts/lidar/test_ydlidar_sdk.sh
   ros2 topic hz /scan --qos-profile sensor_data
   ```
2. Upstream params only:
   ```bash
   bash scripts/lidar/test_ydlidar_upstream.sh
   ```
3. Confirm hardware with **your existing fdpo driver** (ROS 1):
   ```bash
   bash scripts/lidar/test_fdpo_ros1.sh
   ```

If (3) spins but (1–2) do not, keep fdpo for bringup short-term or we port the fdpo start sequence into a thin ROS 2 node later.

### QoS on `/scan` (RViz must use Best Effort)

Drivers publish with **sensor_data** (best effort). Default RViz uses **Reliable** → no points visible.

**Fix:** use the bundled RViz config:

```bash
bash scripts/lidar/view_rviz.sh
```

Or in RViz manually: LaserScan display → Topic → **Reliability Policy: Best Effort**.

Check rate (Jazzy — no `--qos-profile` on `topic hz`; use logs or):

```bash
ros2 topic list | grep scan
# driver logs: "scan NNN: 400 pts"
```

### Ctrl+C crash on `ydlidar_ros2_driver_node`

If you stop the launch quickly, the vendor node may print `Failed to start the lidar` and exit with code -6 **during shutdown only**. The other two nodes exit cleanly; this is a known quirk of `ydlidar_ros2_driver` when interrupted — not a hardware fault. Ignore if you already saw **health status good** and topics publishing.

If the scan is reversed vs your mount (fdpo used 180° yaw on some robots):

```bash
ros2 launch forest_lidar_ros2 ydlidar_x4_test.launch.py laser_yaw_rad:=3.14159
```

## ROS 1 fdpo (reference)

```bash
source ~/catkin_ws_fdpo/devel/setup.bash
roslaunch sdpo_driver_laser_2d sdpo_driver_laser_2d_YDLIDARX4.launch
```

See [docs/FORESTRY_LIDAR_PRACTICES.md](../../../docs/FORESTRY_LIDAR_PRACTICES.md).

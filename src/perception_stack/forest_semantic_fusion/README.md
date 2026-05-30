# `forest_semantic_fusion`

Late-fusion node for forest perception:

- input points: `/sensors/lidar/points`
- input semantics: `/perception/semantic_mask`
- input camera intrinsics: `/camera/camera_info`
- output: `/perception/semantic_points` (`PointCloud2` with field `label`)

The node projects each LiDAR point into the semantic mask and copies class ID to the point.

## Run

```bash
colcon build --packages-select forest_semantic_fusion
source install/setup.bash
ros2 launch forest_semantic_fusion semantic_point_fusion.launch.py
```

## Notes

- Set `extrinsics_lidar_to_camera_row_major` in `config/semantic_point_fusion.yaml` with calibrated lidar->camera transform.
- Current output contains fields `x,y,z,label`.


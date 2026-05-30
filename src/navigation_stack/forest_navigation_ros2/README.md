# Forest Navigation MVP

## Build (NMPC)

Requires acados + generated solver:

```bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
bash scripts/generate_nmpc_solver.sh
cd /path/to/forest-hybrid-ros2-stack
colcon build --packages-select forest_navigation_ros2 --symlink-install
```

`controller_type`: `nmpc` (default) or `pure_pursuit`.

Pipeline modular:

```
/planning/mission_goal
  → global_planner (path esparso)
  → local_planner (refine)
  → trajectory_sampler (densificação)
  → NMPC ou Pure Pursuit → /forest_gen/cmd_vel
  → feedback: /planning/progress, /planning/goal_reached, /planning/path_blocked
```

## Sim (dois terminais)

**Terminal 1 — ForestGen**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Projetos/Gazebo/ForestGen/install/setup.bash
ros2 launch forest_gen_bringup sim_rviz.launch.py world:=mvp_empty_flat.sdf
# Play no Gazebo
```

**Terminal 2 — stack tese**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Projetos/Tese/forest-hybrid-ros2-stack/install/setup.bash
ros2 launch forest_hybrid_conf navigation_mvp.launch.py use_sim_time:=true
```

**Terminal 3 — GOTO**
```bash
ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \
  "{command_type: 1, frame_type: 0, command_id: 'goto1', source: 'cli', target_x: 5.0, target_y: 0.0, target_z: 0.0}"
```

RViz debug (opcional): `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix forest_navigation_ros2)/share/forest_navigation_ros2/config/navigation_debug.rviz`

Plots: `ros2 run forest_navigation_ros2 plot_navigation_metrics.py`

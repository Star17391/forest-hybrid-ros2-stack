# Optional third-party binaries for NMPC codegen (not committed).

## Option A — download binary (fastest)

```bash
mkdir -p tools/third_party/bin
curl -fsSL -o tools/third_party/bin/t_renderer \
  https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64
chmod +x tools/third_party/bin/t_renderer
```

## Option B — build from acados bundled source (offline-friendly)

Requires Rust (`cargo`). Source lives in `$ACADOS_SOURCE_DIR/interfaces/acados_template/tera_renderer`.

```bash
export ACADOS_SOURCE_DIR=$HOME/acados
bash scripts/generate_nmpc_solver.sh
```

The script tries Option B automatically when the binary is missing.

## Generate solver + rebuild

```bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
bash scripts/generate_nmpc_solver.sh
cd /path/to/forest-hybrid-ros2-stack
colcon build --packages-select forest_navigation_ros2 --symlink-install
```

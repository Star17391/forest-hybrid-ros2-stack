# Canonical forest CLI words for bash Tab completion (sync_completions.sh).

FOREST_CLI_TOP_LEVEL="status down cleanup up world profile panel teleop attach test diag logs completion help -h --help"
FOREST_CLI_DIAG_SUBS="imu tf imu-check imu-analyze tf-audit lidar lidar-classify tf-frames lidar3d-stack pose pose-benchmark ekf-latency phase0-compare world"
FOREST_CLI_COMPLETION_SUBS="refresh sync"
FOREST_CLI_ATTACH_SUBS="panel teleop logs"
FOREST_CLI_LOGS_OPTS="-f --follow -n --grep -h --help"
FOREST_CLI_PROFILE_SUBS="list validate"
FOREST_CLI_WORLD_SUBS="list"
FOREST_CLI_UP_OPTS="-d --detach --panel-only --headless --no-rviz --lidar2d --lidar3d --lidar3d-experimental --lidar3d-experimental-only --world -w --trunk-slice --trunk-column --timeout -h --help"

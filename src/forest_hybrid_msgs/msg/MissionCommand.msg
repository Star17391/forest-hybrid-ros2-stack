# Mission command issued by operator/UI.
# GNSS coordinates must be converted to FRAME_MAP before sending (e.g. in GUI / gateway).

uint8 CMD_HOLD=0
uint8 CMD_GOTO_XYZ=1
uint8 CMD_PATROL_WAYPOINTS=2
uint8 CMD_TRACK_TARGET=3
uint8 CMD_RETURN_HOME=4
uint8 CMD_EMERGENCY_STOP=5
# After EMERGENCY_STOP the stack is latched until the operator sends this (policy: only after safe reset / inspection).
uint8 CMD_CLEAR_EMERGENCY_LATCH=6

uint8 FRAME_MAP=0
uint8 FRAME_RELATIVE=1

uint8 command_type
uint8 frame_type
string command_id
string source

# Target for GOTO_XYZ / TRACK_TARGET / RETURN_HOME
float64 target_x
float64 target_y
float64 target_z

# Final heading in map frame (yaw about +Z), radians — for GOTO_XYZ / TRACK_TARGET / RETURN_HOME when use_target_yaw is true.
bool use_target_yaw
float64 target_yaw_rad

# Waypoints for PATROL_WAYPOINTS (same frame_type).
float64[] waypoint_x
float64[] waypoint_y
float64[] waypoint_z
# Optional per-waypoint final yaw (rad, map). Empty => tangent / approach rules in mission_manager; if set, must match waypoint_x length.
float64[] waypoint_yaw

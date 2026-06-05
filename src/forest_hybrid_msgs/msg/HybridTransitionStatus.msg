# Hybrid ground↔aerial reconfiguration FSM (sim + future hardware).
uint8 STATE_GROUND_DRIVE=0
uint8 STATE_TRANSITION_LOCK=1
uint8 STATE_TRACKS_ROTATING=2
uint8 STATE_AERIAL_READY=3
uint8 STATE_AERIAL_FLY=4
uint8 STATE_AERIAL_HOVER=5
uint8 STATE_LEGS_EXTENDING=6
uint8 STATE_LEGS_RETRACTING=7
uint8 STATE_FAILED=255

uint8 state
string state_name
string detail
float32 left_track_yaw_rad
float32 right_track_yaw_rad
float32 leg_extension_m
float32 base_z_m
bool multicopter_enabled
bool airborne

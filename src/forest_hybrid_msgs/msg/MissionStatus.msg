# Mission manager runtime state.

uint8 STATE_IDLE=0
uint8 STATE_EXECUTING=1
uint8 STATE_WAITING_ACK=2
uint8 STATE_HOLDING=3
uint8 STATE_EMERGENCY=4
uint8 STATE_COMPLETED=5
uint8 STATE_FAILED=6

uint8 state
string active_command_id
uint8 active_command_type
uint32 retries_used
string detail
float32 progress

# Current high-level motion state published by uniubi_motion_bridge.
uint8 DISCONNECTED=0
uint8 CONNECTED=1
uint8 CONTROLLED=2

builtin_interfaces/Time stamp
uint8 control_state
string current_action
float32 line_velocity_x
float32 line_velocity_y
float32 angular_velocity
int32 last_error_code
string last_error_message

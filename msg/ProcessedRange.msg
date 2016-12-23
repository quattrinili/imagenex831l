# Processed echo data from sonar.
std_msgs/Header header
uint8[] intensity # intensity.

bool range_error
bool frequency_error
bool internal_error
bool switches_accepted

float32 head_position
bool step_direction # 0 ccw, 1 cw.
float32 max_range
uint16 profile_range

float32 roll_angle
float32 pitch_angle
float32 roll_acceleration
float32 pitch_acceleration


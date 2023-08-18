# Offboard control command message

# control mode flags
uint8 MODE_PASS_THROUGH = 0
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2

# ignore field bitmasks
uint8 IGNORE_NONE = 0
uint8 IGNORE_X = 1
uint8 IGNORE_Y = 2
uint8 IGNORE_Z = 4
uint8 IGNORE_F = 8

std_msgs/Header header
uint8 mode # offboard control mode for interpreting value fields
uint8 ignore # bitmask for ignore specific setpoint values
float32 x
float32 y
float32 z
float32 f

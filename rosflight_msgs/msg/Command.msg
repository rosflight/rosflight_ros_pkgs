# Offboard control command message

# control mode flags
uint8 MODE_PASS_THROUGH = 0
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2

# ignore field bitmasks
uint8 IGNORE_NONE = 0
uint8 IGNORE_QX = 1
uint8 IGNORE_QY = 2
uint8 IGNORE_QZ = 4
uint8 IGNORE_FX = 8
uint8 IGNORE_FY = 16
uint8 IGNORE_FZ = 32

std_msgs/Header header
uint8 mode # offboard control mode for interpreting value fields
uint8 ignore # bitmask for ignore specific setpoint values
float32 qx
float32 qy
float32 qz
float32 fx
float32 fy
float32 fz

# Offboard control command message

# control mode flags
uint8 MODE_PASS_THROUGH = 0
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2

# ignore field bitmasks
uint16 IGNORE_NONE = 0x0
uint16 IGNORE_VALUE0 = 0x1
uint16 IGNORE_VALUE1 = 0x2
uint16 IGNORE_VALUE2 = 0x4
uint16 IGNORE_VALUE3 = 0x8
uint16 IGNORE_VALUE4 = 0x10
uint16 IGNORE_VALUE5 = 0x20
uint16 IGNORE_VALUE6 = 0x40
uint16 IGNORE_VALUE7 = 0x80
uint16 IGNORE_VALUE8 = 0x100
uint16 IGNORE_VALUE9 = 0x200

std_msgs/Header header
uint8 mode      # offboard control mode for interpreting value fields
uint8 ignore    # bitmask for ignore specific setpoint values
float32[10] u   # Vector of control commands

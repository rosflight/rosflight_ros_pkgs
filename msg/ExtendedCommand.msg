# Offboard control command message

# control mode flags
uint8 MODE_PASS_THROUGH = 0
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2
uint8 MODE_ROLL_PITCH_YAWRATE_ALTITUDE = 3

# ignore field bitmasks
uint8 IGNORE_NONE = 0
uint8 IGNORE_VALUE1 = 1
uint8 IGNORE_VALUE2 = 2
uint8 IGNORE_VALUE3 = 4
uint8 IGNORE_VALUE4 = 8

uint8 mode # offboard control mode for interpreting value fields
uint8 ignore # bitmask for ignore specific setpoint values
float32 value1
float32 value2
float32 value3
float32 value4

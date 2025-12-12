# Flight controller status message

std_msgs/Header header

# Options for RC override reasons
uint16 OVERRIDE_NO_OVERRIDE = 0x00
uint16 OVERRIDE_ATT_SWITCH = 0x01
uint16 OVERRIDE_THR_SWITCH = 0x02
uint16 OVERRIDE_X = 0x04
uint16 OVERRIDE_Y = 0x08
uint16 OVERRIDE_Z = 0x10
uint16 OVERRIDE_T = 0x20
uint16 OVERRIDE_OFFBOARD_X_INACTIVE = 0x40
uint16 OVERRIDE_OFFBOARD_Y_INACTIVE = 0x80
uint16 OVERRIDE_OFFBOARD_Z_INACTIVE = 0x100
uint16 OVERRIDE_OFFBOARD_T_INACTIVE = 0x200

bool armed         # True if armed
bool failsafe      # True if in failsafe
uint16 rc_override   # Bitfield of RC override reasons
bool offboard      # True if offboard control is active
uint8 control_mode # Onboard control mode
uint8 error_code   # Onboard error code
int16 num_errors   # Number of errors
int16 loop_time_us # Loop time in microseconds

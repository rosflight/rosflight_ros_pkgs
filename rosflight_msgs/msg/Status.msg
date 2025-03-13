# Flight controller status message

std_msgs/Header header

bool armed         # True if armed
bool failsafe      # True if in failsafe
uint8 rc_override  # 3 = attitude and throttle rc override, 2 = throttle rc override, 1 = attitude rc override, 0 = no rc override
bool offboard      # True if offboard control is active
uint8 control_mode # Onboard control mode
uint8 error_code   # Onboard error code
int16 num_errors   # Number of errors
int16 loop_time_us # Loop time in microseconds

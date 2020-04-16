# GNSS

# TODO: does this message need header if Time is in it?
# the ros header time is going to be latent to moment of measurement
# consider removing `header` everywhere, and just using a call to system time
# this will help make everything more consistent, especially since ROS2
# does not have a ros-core running. All machines system time should be synced
# with chrony, or something similar, at the system level.

std_msgs/Header header # Estimated ROS time at moment of measurement
# header.frame_id may take value of __ or __

uint8 fix # fix type, as defined in the UBX protocol, enums defined below
builtin_interfaces/Time time # GPS time at moment of measurement
float64[3] position # m, ECEF frame
float64 horizontal_accuracy # m
float64 vertical_accuracy # m
float64[3] velocity # m/s, ECEF frame
float64 speed_accuracy # m/s

uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_FIX = 1
uint8 FIX_TYPE_RTK_FLOAT = 2
uint8 FIX_TYPE_RTK_FIXED = 3

# GNSS

std_msgs/Header header # Estimated ROS time at moment of measurement

uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_FIX = 1
uint8 FIX_TYPE_RTK_FLOAT = 2
uint8 FIX_TYPE_RTK_FIXED = 3

uint8 fix_type # fix type, as defined in the UBX protocol, enums defined above

uint16 year
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
uint32 nano

uint8 num_sat
int32 lat
int32 lon
int32 alt                   # m above ellipsoid
float64 horizontal_accuracy # m
float64 vertical_accuracy   # m

float64 vel_n                 # m/s
float64 vel_e                 # m/s
float64 vel_d                 # m/s
float64 speed_accuracy      # m/s


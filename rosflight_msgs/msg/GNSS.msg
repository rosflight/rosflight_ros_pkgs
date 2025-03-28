# GNSS

std_msgs/Header header # Estimated ROS time at moment of measurement

uint8 GNSS_FIX_TYPE_NO_FIX = 0
uint8 GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 GNSS_FIX_TYPE_2D_FIX = 2
uint8 GNSS_FIX_TYPE_3D_FIX = 3
uint8 GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4
uint8 GNSS_FIX_TYPE_TIME_FIX_ONLY = 5

uint8 fix_type # fix type, as defined in the UBX protocol, enums defined above

uint16 year
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec

uint8 num_sat
int32 lat                   # deg*10e7
int32 lon                   # deg*10e7
float32 alt                 # m above ellipsoid
float32 horizontal_accuracy # m
float32 vertical_accuracy   # m

float32 vel_n                 # m/s
float32 vel_e                 # m/s
float32 vel_d                 # m/s
float32 velocity_accuracy     # m/s


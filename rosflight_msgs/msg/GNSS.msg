# GNSS

std_msgs/Header header # Estimated ROS time at moment of measurement, timestamp of the last received packet

uint8 GNSS_FIX_TYPE_NO_FIX = 0
uint8 GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 GNSS_FIX_TYPE_2D_FIX = 2
uint8 GNSS_FIX_TYPE_3D_FIX = 3
uint8 GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4
uint8 GNSS_FIX_TYPE_TIME_FIX_ONLY = 5

uint8 fix_type # fix type, as defined in the UBX protocol, enums defined above

uint8 num_sat
float64 lat                   # deg DDS
float64 lon                   # deg DDS
float32 alt                 # m above mean sea level
float32 horizontal_accuracy # m
float32 vertical_accuracy   # m

float32 vel_n                 # m/s
float32 vel_e                 # m/s
float32 vel_d                 # m/s
float32 speed_accuracy        # m/s

int64 gnss_unix_seconds # Reported GNSS time (aka UNIX time), seconds
int32 gnss_unix_nanos   # Reported GNSS fractional time (aka UNIX time), nanoseconds

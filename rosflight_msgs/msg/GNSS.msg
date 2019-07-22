Header header # Estimated ROS time at moment of measurement
uint8 fix # fix type, as defined in the UBX protocol
time time # GPS time at moment of measurement
float64[3] position # m, ECEF frame
float64 horizontal_accuracy # m
float64 vertical_accuracy # m
float64[3] velocity # m/s, ECEF frame
float64 speed_accuracy # m/s


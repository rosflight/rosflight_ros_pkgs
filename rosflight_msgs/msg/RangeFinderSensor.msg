# RangeFinderSensor message
# Contains timestamp, count of distances and angles, and their measurements

# Timestamp in nanoseconds
int64 timestamp

# Number of distance measurements
int32 count

# Distance measurements in meters
float32[] distances

# Angle measurements in radians
float32[] angles

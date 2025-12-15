# RGBCamera message
# Contains timestamp, image dimensions, and image data

# Timestamp in nanoseconds
int64 timestamp

# Image width in pixels
int32 width

# Image height in pixels
int32 height

# Number of color channels
int32 channels

# Flattened image data as a 1D array
# Should be a 3D array but ROS does not support
int16[] image


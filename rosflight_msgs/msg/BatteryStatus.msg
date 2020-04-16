# Simple battery telemetry
# Values may be 0 if not supported

# TODO: does this message really need a header?

std_msgs/Header header
# header.frame_id may take value of __ or __

float32 voltage
float32 current

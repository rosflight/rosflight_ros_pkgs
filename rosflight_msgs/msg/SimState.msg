# Message containing the elements of the state needed to create sensor information
# Only used by the sim

geometry_msgs/Transform pose # [xyzw], NED
float32[3] velocity   # NED in body frame, with respect to inertial, m/s
float32[3] acceleration # linear, globla NED, m/s2
float32[3] angular_velocity
float32[3] air_velocity # NED in body frame, with respect to the air, m/s


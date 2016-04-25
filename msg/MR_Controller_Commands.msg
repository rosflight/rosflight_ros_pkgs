# Controller commands to the controller of a multirotor

Header header
geometry_msgs/Vector3 acceleration # commanded acceleration
geometry_msgs/Vector3 velocity # commanded velocity
geometry_msgs/Vector3 position # commanded position
geometry_msgs/Vector3 alpha # commanded angular acceleration
geometry_msgs/Vector3 omega # commanded angular velocity
geometry_msgs/Vector3 theta # commanded angle
float64[4] aux		# Optional auxiliary commands

bool position_valid
bool velocity_valid
bool acceleration_valid

# Controller commands to the controller of a multirotor

Header header
geometry_msgs/Vector3 acceleration # commanded acceleration
geometry_msgs/Vector3 velocity # commanded velocity
geometry_msgs/Vector3 position # commanded position
float64[4] aux		# Optional auxiliary commands

bool position_valid
bool velocity_valid
bool acceleration_valid

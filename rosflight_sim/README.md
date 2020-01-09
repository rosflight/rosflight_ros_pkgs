# ROSflight Sim Package

This package runs the ROSflight firmware as a gazebo plugin.  It simulates all the connected sensors
and communicates the same way that the hardware does in real life.

## Parameters
Parameters should be set in the same namespace as specified in the `xacro` file that declares the
plugin.  If you are unsure what this is, try running gazebo with the `--verbose` setting on and look 
for a message that looks like `loading parameters from <namespace> ns`

- `gazebo_host`: host name or IP address of the machine running the gazebo instance
- `gazebo_port`: port of rosflight_sim plugin. (default = `14525`)  This only needs to change if you are trying to simulate multiple agents
- `ROS_host`: host name or IP address of the machine running `rosflight_io`
- `ROS_port`: port of `rosflight_io` only needs to change if simulating multiple agents

- `gyro_stdev`: default: `0.13`
- `gyro_bias_range`: default: `0.15`
- `gyro_bias_walk_stdev`: default: `0.001`
- `acc_stdev`: default: `1.15`
- `acc_bias_range`: default: `0.15`
- `acc_bias_walk_stdev`: default: `0.001`
- `mag_stdev`: default: `1.15`
- `mag_bias_range`: default: `0.15`
- `mag_bias_walk_stdev`: default: `0.001`
- `baro_stdev`: default: `1.15`
- `baro_bias_range`: default: `0.15`
- `baro_bias_walk_stdev`: default: `0.001`
- `airspeed_stdev`: default: `1.15`
- `airspeed_bias_range`: default: `0.15`
- `airspeed_bias_walk_stdev`: default: `0.001`
- `sonar_stdev`: default: `1.15`
- `sonar_min_range`: (m) default: `0.25`
- `sonar_max_range`: (m) default: `8.0`
- `imu_update_rate`: (Hz) default: `1000.0`
- `inclination`: (rad) default: `1.14316156541`
- `declination`: (rad) default: `0.198584539676`
- `origin_altitude`: (m) default: `1387.0`
- `origin_latitude`: (deg) default: `40.2463724`
- `origin_longitude`: (deg) default: `-111.6474138.0`
- `horizontal_gps_stdev`: (m) default: `3.0`
- `vertical_gps_stdev`: (m) default: `1.0`
- `gps_velocity_stdev`: (m/s) default: `0.1`
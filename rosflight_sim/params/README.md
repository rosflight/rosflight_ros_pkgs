# ROSflight Sim Package

This package runs the ROSflight firmware as a gazebo plugin. It simulates all the connected sensors
and communicates the same way that the hardware does in real life.

## Parameters

Parameters can be set in the "rosflight/params/multirotor.yaml" files and "rosflight_sim/params/fixedwing.yaml".

## General Parameters

These parameters apply to both multirotors and fixedwings, and are used by sil_board. They have default values and don't 
need to be set in the .yaml files.

- `gazebo_host`: host name or IP address of the machine running the gazebo instance
- `gazebo_port`: port of rosflight_sim plugin. (default = `14525`)  This only needs to change if you are trying to
  simulate multiple agents
- `ROS_host`: host name or IP address of the machine running `rosflight_io`
- `ROS_port`: port of `rosflight_io` only needs to change if simulating multiple agents

- `serial_delay_ns`: (nanoseconds) default `0.006 * 1e9`
- `gyro_stdev`: default: `0.00226`
- `gyro_bias_range`: default: `0.25`
- `gyro_bias_walk_stdev`: default: `0.00001`

- `acc_stdev`: default: `0.025`
- `acc_bias_range`: default: `0.6`
- `acc_bias_walk_stdev`: default: `0.00001`

- `mag_stdev`: default: `0.10`
- `mag_bias_range`: default: `0.10`
- `mag_bias_walk_stdev`: default: `0.001`

- `baro_stdev`: default: `4.0`
- `baro_bias_range`: default: `500`
- `baro_bias_walk_stdev`: default: `0.1`

- `airspeed_stdev`: default: `1.15`
- `airspeed_bias_range`: default: `0.15`
- `airspeed_bias_walk_stdev`: default: `0.001`

- `sonar_stdev`: default: `0.03`
- `sonar_min_range`: (m) default: `0.25`
- `sonar_max_range`: (m) default: `8.0`

- `imu_update_rate`: (Hz) default: `1000.0`

- `inclination`: (rad) default: `1.14316156541`
- `declination`: (rad) default: `0.198584539676`

- `origin_altitude`: (m) default: `1387.0`
- `origin_latitude`: (deg) default: `40.2463724`
- `origin_longitude`: (deg) default: `-111.6474138.0`

- `horizontal_gps_stdev`: (m) default: `1.0`
- `vertical_gps_stdev`: (m) default: `3.0`
- `gps_velocity_stdev`: (m/s) default: `0.1`

## Multirotor and Fixedwing params

All parameters for multirotors and fixedwings must be defined in their respective .yaml files. ROS will give a warning 
for any missing parameters. Refer to the existing parameter files to know what parameters are required.
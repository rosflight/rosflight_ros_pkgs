# fcu_io2
ROS Driver for [ROSflight2](https://github.com/BYU-MAGICC/ROSflight2 "ROSflight2").  This package provides high-bandwidth access to sensors and actuators on an embedded flight controller as ROS topics, and the ability to change flight controller parameters via the ROS service API.  This is _incompatible_ with ROSflight1.0.  While ROSflight was originally based on the popular [cleanflight](https://github.com/cleanflight/cleanflight) and [baseflight](https://github.com/multiwii/baseflight)  We switched from using MultiWii Serial Protocol (MSP) to MAVLink to get more robust integrity checking and the ability to support streams.  This new system has demonstrated sensor stream rates of more than 10 times those achieved by the MSP system.

While we have leveraged baseflight in particular quite extensively for hardware drivers, we also have completely re-vamped the actual control code to be more research-friendly.  We have used only peer-reviewed sources for our estimation and control algorithms, prioritized offboard control and offboard sensor streaming, and deeply integrated safety-pilot RC control.  Our goal is to have a lean, modular system with well documented, easy to understand coding style based on established control and estimation algorithms.  The vision is that a researcher could easily develop code in ROS, simulate using Gazebo, and quickly run on hardware, without extensive embedded programming.  ROSflight is _not_ intended for racing, or pleasure-flying.  It is not as easy to set up as cleanflight or baseflight, and will not have support for non-essentials, such as LED strips and on-screen displays. 

## Dependencies
This package depends on [BYU-MAGICC/fcu_common](https://github.com/BYU-MAGICC/fcu_common "fcu_common"), the package for common messages and libraries required for flight control.  Currently, this package has only been tested in ROS indigo on Ubuntu 14.04.  It is not yet officially supported on 16.04 or ROS Kinetic.  If you are unfamiliar with ROS, you may want to first complete the [Official ROS Tutorials](http://wiki.ros.org/ROS/Tutorials "Official ROS Tutorials") or the [MAGICC Lab ROS Tutorials](https://magiccvs.byu.edu/wiki/ROS_Tutorials "MAGICC Lab ROS Tutorials")

## Installation
#### Begin a catkin workspace (assuming you have already [installed ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment))
``` bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
```
* Note, this adds a line to your `~/.bashrc` file.  If you are unfamiliar with this file, read about it [here](http://unix.stackexchange.com/questions/129143/what-is-the-purpose-of-bashrc-and-how-does-it-work)

#### Clone this repository and dependencies into the workspace
```bash
cd ~/catkin_ws/src
git clone https://github.com/BYU-MAGICC/fcu_io2.git
git clone https://github.com/BYU-MAGICC/fcu_common.git
```
#### Clone the MAVlink git submodule within `fcu_io2`
```bash
cd fcu_io2
git submodule update --init --recursive
```
#### Build
```bash
cd ~/catkin_ws
catkin_make
```

## Running the Node
This package contains a single node, `fcu_io_node`.  To run it, first run a `roscore`, then
```bash
rosrun fcu_io fcu_io_node
```
## Topics
__Subscriptions__

* __extended_command__ - `fcu_common::ExtendedCommand` - Commands sent to the flight controller to be executed according to the mode and ignore field.

__Publications__ - These are only published if information is being received from MAVlink.  The publisher is registered upon the first message receveived over MAVlink.  If a sensor is missing, or the stream rate of a particular stream is set to `0` on boot-up, then the corresponding publication may not occur.
* __imu/data__ - `sensor_msgs::Imu` - IMU measurement (orientation and covariance is currently not being populated)
* __imu/temperature__ - `sensor_msgs::Temperature` - Temperature of onboard IMU sensor
* __baro/data__ - `std_msgs::Float32` - Barometer measurement in meters
* __sonar/data__ - `sensor_msgs::Range` - Ultrasonic Sonar measurement
* __attitude__ - `fcu_common::Attitude` - Internal Attitude estimate of the flight controller
* __diff_pressure__ - `sensor_msgs::FluidPressure` - Differential pressure from pitot tube sensor
* __temperature__ - `sensor_msgs::Temperature` - Temperature of differential pressure sensor

* __servo_output_raw__ - `fcu_common::ServoOutputRaw` - Raw outputs in us to actuators from flight controller (for debugging)
* __rc_raw__ - `fcu_common::ServoOutputRaw - Raw inputs in us from RC (for debugging)
* __named_value/float/<name>__ - `std_msgs::Float32` - Dynamic publication automatically created from MAVlink.  This is generally used for debugging code on the flight controller.
* __named_value/int/<name>__ - `std_msgs::Int32` - Dynamic publication automatically created from MAVlink.  This is generally used for debugging code on the flight controller.
* __named_value/command_struct/<name>__ - `fcu_common::ExtendedCommand` - Dynamic publication automatically created from MAVlink.  This is generally used for debugging the muxing of command structs on the flight controller.

## Services
* __param_get__ - Retrieves a parameter from the flight controller.
* __param_set__ - Sets a parameter on the flight controller.  Changes take place immediately and take place in RAM (volatile memory), but parameters regarding hardware setup may require reboot to take effect.
* __param_write__ - Writes the current parameter configuration to the EEPROM (non-volatile memory).  This is required for parameter changes to persist after reboot.
* __calibrate_imu_bias__ - Sets IMU biases to be equal to current measurements.  This is fast, but does not take into account temperature compensation.  Do not move the FCU within 1 second of calling this service.
* __calibrate_imu_temp__ - Calculates linear temperature compensation coefficients for all Accelerometer axes, gyroscope biases and accelerometer offset biases.  This should be performed as closely to startup as possible, so as to get sufficient variance in temperature to perform appropriate compensation.  Do not move the flight controller within 10 seconds of calling this service.
* __calibrate_rc_trim__ - Calibrates the trim on an RC transmitter, and offsets future commands from ROS to be with respect to trims from RC.  Prompts are sent via command line to perform operations on the RC transmitter during calibration. To calibrate RC, first fly the UAV on RC control and trim to a stable equilibrium. Land, disarm, and call this service, which will ask you to first move all axes of the RC transmitter to the fullest extents, then leave the sticks centered for a few seconds.  This will then set the commands from the onboard comptuer to be with respect to the stable trim condition, as opposed to some non-equilibrium, by applying equilibrium control to the output of the control loops.  RC commands are not modified within the flight controller, so trims must be left on the transmitter following this service (As opposed to the method used by the PixHawk)

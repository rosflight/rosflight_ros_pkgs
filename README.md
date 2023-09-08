# ROSflight

[![ROS2 CI](https://github.com/rosflight/rosflight/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/rosflight/rosflight/actions/workflows/ros2-ci.yml)

This repository contains a ROS2 stack for interfacing with an autopilot running the ROSflight firmware.
For more information on the ROSflight autopilot firmware stack, visit http://rosflight.org.

# Packages Overview

## rosflight_io

This package contains the rosflight_io node, which provides the core functionality for interfacing an onboard computer
with the autopilot. This node streams autopilot sensor and status data to the onboard computer, streams control
setpoints to the autopilot, and provides an interface for configuring the autopilot. It also contains a mag_cal node
that is used to calibrate the UAV's magnetometer parameters.

rosflight_io utilizes two libraries: mavrosflight and mavlink. Mavlink is the communication protocol used for serial
communication between rosflight_io and the firmware. It exists as a separate Github repository, utilized by both
rosflight_io and the firmware. Mavrosflight is what handles the actual serial communication in rosflight and is largely
ROS independent. rosflight_io mostly just manages the interactions between mavrosflight and ROS.

## rosflight_firmware

This package contains an udp_board implementation of the ROSflight firmware and a copy of the firmware itself as a git 
submodule. udp_board can be used to run instances of the firmware on a device with UDP communication and is used by 
rosflight_sim for its sil_board.

## rosflight_msgs

This package contains the ROSflight message and service definitions.

## rosflight_pkgs

This is a metapackage for grouping the other packages into a ROS stack.

## rosflight_sim

This package contains files for running ROSflight in the Gazebo simulator. It has a SIL board implementation, forces and 
moments calculations for both fixedwing and multirotors, launch files for launching the sim, and model and world files 
for the Gazebo visualization. Dynamics for both multirotors and fixedwings can be modified in the .yaml files found in 
the `rosflight_sim/params` folder. To launch the sim, use `ros2 launch rosflight_sim fixedwing.launch.py` for a 
fixedwing simulation and `ros2 launch rosflight_sim multirotor.launch.py` for a multirotor simulation. You can also add
`gui:=false` to launch a sim without visualization. See the launch files under `rosflight_sim/launch` for additional 
parameters.

## rosflight_utils

This package contains additional supporting scripts and libraries that are not part of the core ROSflight package
functionality. These include the following:

### Attitude and magnetometer visualizer

This utility uses RViz and the viz node to allow easy visualization of the attitude of flight controller (as 
determined by the firmware's onboard estimator) and the magnetometer data. These can be launched with `ros2 launch 
rosflight_utils viz_att.launch.py` and `ros2 launch rosflight_utils viz_mag.launch.py`.

### rc_joy

This script contains a node that allows for connecting a gamepad or transmitter to the simulator. To use it, plug in 
a controller and launch `ros2 run rosflight_utils rc_joy.py --ros-args --remap RC:=/fixedwing/RC` for a fixedwing sim or
`ros2 run rosflight_utils rc_joy.py --ros-args --remap RC:=/multirotor/RC` for a multirotor sim. Currently 
supported devices are Taranis Q-X7 transmitters, XBox controllers, RealFlight InterLink controllers, and 
RadioMaster TX16S transmitters. Adding addition devices can be done easily, so long as the device has USB gamepad 
support.

### command_joy

This script contains a node that allows for using a gamepad or transmitter for the command input to rosflight. To use 
it, plug in a controller and launch `ros2 run rosflight_utils command_joy.py --ros-args --remap RC:=/fixedwing/RC` for a
fixedwing sim or `ros2 run rosflight_utils command_joy.py --ros-args --remap RC:=/multirotor/RC` for a multirotor sim. 
Currently supported devices are Taranis Q-X7 transmitters, XBox controllers, RealFlight InterLink controllers, and
RadioMaster TX16S transmitters. Adding addition devices can be done easily, so long as the device has USB gamepad
support.

### rc_sim

This script contains a node that allows for arming/disarming the firmware in the simulator as well as enabling/disabling
rc_override as if flipping switches on an actual controller. Currently, this node expects the arm switch to be on 
channel 4 and the override switch to be on channel 5. Launch it with `ros2 run rosflight_utils rc_sim.py --ros-args 
--remap RC:=/fixedwing/RC` for a fixedwing sim or `ros2 run rosflight_utils rc_sim.py --ros-args --remap 
RC:=/multirotor/RC` for a multirotor sim. Arm with `ros2 service call /arm std_srvs/srv/Trigger`, disarm with `ros2 
service call /disarm std_srvs/srv/Trigger`, enable override with `ros2 service call /enable_override 
std_srvs/srv/Trigger`, and disable override with `ros2 service call /disable_override std_srvs/srv/Trigger`.

### Simulator, rosflight_io, and rc_joy launch files

The gazebo simulator with the sil_node can be launched alongside rosflight_io and rc_joy, for convenience. Use 
`ros2 launch rosflight_utils fixedwing_sim_io_joy.launch.py` for a fixedwing sim and 
`ros2 launch rosflight_utils multirotor_sim_io_joy.launch.py` for a multirotor sim.

### Firmware parameter files

Basic parameter files for setting up a multirotor or fixedwing UAV have been provided, under the
`rosflight/rosflight_utils/params` directory. Use 
`ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{filename: "/path_to_rosflight/rosflight_utils/params/fixedwing_firmware.yaml"}"` for fixedwings and 
`ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{filename:"/path_to_rosflight/rosflight_utils/params/multirotor_firmware.yaml"}"` for multirotors.

### Firmware initialization launch files

To make setting up the firmware with initial calibrations and parameters easier, launch files have been provided to 
automate this process. Use `ros2 launch rosflight_utils fixedwing_init_firmware.launch.py` for fixedwings and `ros2 
launch rosflight_utils multirotor_init_firmware.launch.py` for multirotors. These launch files reference the parameter
files found in the `rosflight_utils/params` directory mentioned above.

# Building/Running Instructions

## Building the workspace

1. Before installing any new packages, update your system with `sudo apt update` and`sudo apt upgrade`.
2. Install ROS2 Humble. Follow the directions on
   the [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), making sure to
   install both the`ros-humble-desktop` and the `ros-dev-tools` packages.
3. Before ROS can be used, the setup file will need to be sourced in every terminal that you want to use ROS in. This
   can be done with `source /opt/ros/humble/setup.bash`, or you can set bash to source it automatically when opened
   with `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`. Re-open your terminal so that this echo command can
   take effect.
4. Create a rosflight workspace folder and cd into it with `mkdir rosflight_ws && cd rosflight_ws`.
5. Clone the rosflight repository and its submodules
   with `git clone --recursive https://github.com/rosflight/rosflight.git`.
6. Install all required dependencies with rosdep. To do so, initialize rosdep with `sudo rosdep init`, update
   with `rosdep update`, and install the dependencies
   with `rosdep install -i --from-path ./ -y --ignore-src`.
7. Rosdep will install Gazebo for the rosflight_sim packaged, which has a setup file that will need to be sourced. Set
   it to be sourced automatically with `echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc`.
8. Build the repository with `colcon build`. Once built, set the rosflight setup file to be sourced automatically
   with `echo "source ~/rosflight_ws/install/setup.bash" >> ~/.bashrc`. If your workspace folder is in a different
   location than your home directory, updated the path in the command to reflect its location.

## Running the rosflight_io node

To run the rosflight_io node when connected to real hardware, use the
command `ros2 run rosflight_io rosflight_io --ros-args -p port:=/dev/ttyACM0`, replacing `/dev/ttyACM0` with the location
of serial port connected to the flight controller. This will launch a ROS2 node on your computer that will publish all
sensor topics and create all command subscriptions needed to communicated with the firmware.

## Running the Gazebo simulation

All instructions in this section are for a fixedwing simulation, but a multirotor simulation can be launched by
replacing all occurrences of `fixedwing` with `multirotor`.

### Launch gazebo with SIL

To run the ROSflight firmware in the Gazebo simulator, launch Gazebo and the rosflight_sil node
with `ros2 launch rosflight_sim fixedwing.launch.py`. This will launch a rosflight_sil node that contains the full
ROSflight firmware as if it was running on an actual flight computer, the only difference being that instead of calling
real sensors it calls Gazebo sensors.

### Launch rosflight_io node

To run the rosflight_io node with the simulator, use the
command `ros2 run rosflight_io rosflight_io --ros-args -p udp:=true`.

### Launch RC controller interface node

To connect an RC controller to the simulator, plug a controller into your computer and launch the rc_joy node
with `ros2 run rosflight_utils rc_joy.py --ros-args --remap /RC:=/fixedwing/RC`. This will launch a node that receives
RC controller commands and publishes them to be received by rosflight_sil. It currently only has support for the
following: Xbox controllers, Taranis QX7 transmitters, RadioMaster TX16s transmitters, and RealFlight controllers. To
add more, edit rosflight_utils/src/rc_joy.py and rebuild the workspace.

### Launch Gazebo, rosflight_io, and RC interface all at once

To launch the rosflight_sil, rosflight_io node, and rc_joy nodes all at once rather than individually, use the
command `ros2 launch rosflight_utils fixedwing_sim_io_joy.launch.py`.

### Setup firmware parameters for flying in simulation

Note that in order to actually arm and fly the UAV in the simulator, you still need to set the proper parameters on the
flight controller. To do so, launch both the rosflight_sil and rosflight_io nodes. Set all necessary parameters
with `ros2 launch rosflight_utils fixedwing_init_firmware.launch.py`. Wait until launch file completes.
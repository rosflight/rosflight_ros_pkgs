# ROSflight2

This repository contains an in-development ROS2 stack for interfacing with an autopilot running the ROSflight firmware. For more information on the ROSflight autopilot firmware stack, visit http://rosflight.org. Please note that the documentation currently is written for ROS1 and while most of it is still relevant, some of the details for building and using the ROS2 companion-computer stack (not the ROSflight firmware) has changed. See below for updated instructions.

The following sections describe each of the packages contained in this stack.

## rosflight_pkgs

This is a metapackage for grouping the other packages into a ROS stack.

## rosflight_msgs

This package contains the ROSflight message and service definitions.

## rosflight

This package contains the `rosflight_io` node, which provides the core functionality for interfacing an onboard computer with the autopilot. This node streams autopilot sensor and status data to the onboard computer, streams control setpoints to the autopilot, and provides an interface for configuring the autopilot.

## rosflight_utils

This package contains additional supporting scripts and libraries that are not part of the core ROSflight package functionality, including visualization tools for the attitude estimate and magnetometer.

# Updated ROS2 Instructions

Currently, this repo has only been tested with Ubuntu 22.04 LTS and ROS2 Humble.

## Building the full development workspace

1. Before installing any new packages, update your system with `sudo apt update` and`sudo apt upgrade`.
2. Install ROS2 Humble. Follow the directions on the [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), making sure to install both the`ros-humble-desktop` and the `ros-dev-tools` packages.
3. Before ROS can be used, the setup file will need to be sourced in every terminal that you want to use ROS in. This can be done with `source /opt/ros/humble/setup.bash`, or you can set bash to source it automatically when opened with `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`. You'll need to re-open your terminal for the echo command to take effect.
4. Install Gazebo for ROS with the following packages: `ros-humble-ros-gz`, `ros-humble-gazebo-plugins`. These packages will not be available until after you've added the ROS apt repositories to your system as instructed in the ROS installation guide. Once installed, set the gazebo setup file to be sourced automatically with `echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc`.
5. Install the packages `libeigen-stl-containers-dev` and `python3-pygame`.
6. Clone the ROSflight repository and it's submodules with `git clone --recursive https://github.com/byu-magicc/rosflight2.git`. Enter that directory with `cd rosflight2`.
7. Build the repository with `colcon build`. Once built, set the rosflight setup file to be sourced automatically with `echo "source ~/rosflight2/install/setup.bash" >> ~/.bashrc`. If you cloned the repository in a different location than your home directory, updated the path in the command to reflect its location.

## Building the rosflight_io node only

1. Before installing any new packages, update your system with `sudo apt update` and`sudo apt upgrade`.
2. Install ROS2 Humble. Follow the directions on the [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), installing only the `ros-humble-ros-base` package.
3. Before ROS can be used, the setup file will need to be sourced in every terminal that you want to use ROS in. This can be done with `source /opt/ros/humble/setup.bash`, or you can set bash to source it automatically when opened with `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`. You'll need to re-open your terminal for the echo command to take effect.
4. Clone the ROSflight repository and it's submodules with `git clone --recursive https://github.com/byu-magicc/rosflight2.git`. Enter that directory with `cd rosflight2`.
5. Build the repository with `colcon build --packages-select rosflight rosflight_msgs`. Once built, set the rosflight setup file to be sourced automatically with `echo "source ~/rosflight2/install/setup.bash" >> ~/.bashrc`. If you cloned the repository in a different location other than your home directory, updated the path in the command to reflect its location.

## Running the rosflight_io node

To run the rosflight_io node when connected to real hardware, use the command `ros2 run rosflight rosflight_io --ros-args -p port:=/dev/ttyACM0`, replacing `/dev/ttyACM0` with the location of serial port connected to the flight controller. This will launch a ROS2 node on your computer that will publish all sensor topics and create all command subscriptions needed to communicated with the firmware.

## Running the Gazebo simulation

All instructions in this section are for a fixedwing simulation, but a multirotor simulation can be launched by replacing all occurrences of `fixedwing` with `multirotor`.

### Launch gazebo with SIL

To run the ROSflight firmware in the Gazebo simulator, launch Gazebo and the rosflight_sil node with `ros2 launch rosflight_sim fixedwing.launch.py`. This will launch a rosflight_sil node that contains the full ROSflight firmware as if it was running on an actual flight computer, the only difference being that instead of calling real sensors it calls Gazebo sensors.

### Launch rosflight_io node

To run the rosflight_io node with the simulator, use the command `ros2 run rosflight rosflight_io --ros-args -p udp:=true`.

### Launch RC controller interface node

To connect an RC controller to the simulator, plug a controller into your computer and launch the rc_joy node with `ros2 run rosflight_utils rc_joy.py --ros-args --remap /RC:=/fixedwing/RC`. This will launch a node that receives RC controller commands and publishes them to be received by rosflight_sil. It currently only has support for the following: Xbox controllers, Taranis QX7 transmitters, Radiomaster TX16s transmitters, and Realflight controllers. To add more, edit rosflight_utils/src/rc_joy.py and rebuild the workspace.

### Launch Gazebo, rosflight_io, and RC interface all at once

To launch the rosflight_sil, rosflight_io node, and rc_joy nodes all at once rather than individually, use the command `ros2 launch rosflight_utils fixedwing_sil_io_joy.launch.py`. 

### Setup firmware parameters for flying in simulation

Note that in order to actually arm and fly the UAV in the simulator, you still need to set the proper parameters on the flight controller. To do so, launch both the rosflight_sil and rosflight_io nodes. Set all necessary parameters with `ros2 launch rosflight_utils fixedwing_sim_init_firmware.launch.py`. Wait until launch file completes.
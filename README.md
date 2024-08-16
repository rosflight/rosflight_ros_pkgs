# ROSflight

[![ROS2 CI](https://github.com/rosflight/rosflight_ros_pkgs/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/rosflight/rosflight_ros_pkgs/actions/workflows/ros2-ci.yml)

This repository contains a ROS2 stack for interfacing with an autopilot running the ROSflight firmware. For instructions on building this repository, see the [ROS2 setup page](https://docs.rosflight.org/git-main/user-guide/ros2-setup/) on the [ROSflight docs](https://docs.rosflight.org/git-main/).

## Packages Overview

### rosflight_io

This package contains the rosflight_io node, which provides the core functionality for interfacing an onboard computer
with the autopilot. This node streams autopilot sensor and status data to the onboard computer, streams control
setpoints to the autopilot, and provides an interface for configuring the autopilot. It also contains a mag_cal node
that is used to calibrate the UAV's magnetometer parameters.

rosflight_io utilizes two libraries: mavrosflight and mavlink. Mavlink is the communication protocol used for serial
communication between rosflight_io and the firmware. It exists as a separate Github repository, utilized by both
rosflight_io and the firmware. Mavrosflight is what handles the actual serial communication in rosflight and is largely
ROS independent. rosflight_io mostly just manages the interactions between mavrosflight and ROS.

### rosflight_gcs

This package contains utilities that will be used to support the ground control station experience. Currently this is under development and only contains a couple of former rosflight_utils packages.

### rosflight_msgs

This package contains the ROSflight message and service definitions.

### rosflight_pkgs

This is a metapackage for grouping the other packages into a ROS stack.

### rosflight_rqt_plugins

This package contains plugins that can be used within rqt for tuning or ground station purposes. They are built to be autopilot agnostic and can be easily configured for any specific needs.

### rosflight_sim

This package contains files for running ROSflight in the Gazebo simulator. It has a SIL board implementation, forces and 
moments calculations for both fixedwing and multirotors, launch files for launching the sim, and model and world files 
for the Gazebo visualization. Dynamics for both multirotors and fixedwings can be modified in the .yaml files found in 
the `rosflight_sim/params` folder.


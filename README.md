# ROSflight

[![Build Status](https://travis-ci.org/rosflight/rosflight.svg?branch=master)](https://travis-ci.org/rosflight/rosflight)

This repository contains the ROS stack for interfacing with an autopilot running the ROSflight firmware. For more information on the ROSflight autopilot firmware stack, visit http://rosflight.org.

The following sections describe each of the packages contained in this stack.

## rosflight_pkgs

This is a metapackage for grouping the other packages into a ROS stack.

## rosflight_msgs

This package contains the ROSflight message and service definitions.

## rosflight

This package contains the `rosflight_io` node, which provides the core functionality for interfacing an onboard computer with the autopilot. This node streams autopilot sensor and status data to the onboard computer, streams control setpoints to the autopilot, and provides an interface for configuring the autopilot.

## rosflight_utils

This package contains additional supporting scripts and libraries that are not part of the core ROSflight package functionality, including visualization tools for the attitude estimate and magnetometer. This package also helps support the [ROSplane](https://github.com/byu-magicc/rosplane) and [ROScopter](https://github.com/byu-magicc/roscopter) projects.

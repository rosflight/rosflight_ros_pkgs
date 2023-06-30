^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosflight_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (pre-release)
------------------
* Updated package to ROS2 and Gazebo 11
* Removed unused parameters from forces and moments c++ code and yaml param files
* Updated mass and inertia params in xacro files to match aerodynamic params
* Removed broken color param from xacro files
* Changed multirotor to use mesh file (rather than a cylinder) for collision when available
* Rewrote launch files using ROS2 python launch api
* Added UAV model files to resources folder
* Created worlds for fixedwing and multirotor simulations
* Added error message for missing UAV params
* Removed sensor noise and bias params from multirotor param file and updated defaults in sil_board
* Contributors: Brandon Sutherland, Ian Reid, Brady Anderson

1.4.0 (2020-10-06)
------------------
* GNSS improvements
* Contributors: BillThePlatypus

1.3.1 (2020-03-27)
------------------

1.3.0 (2020-03-19)
------------------
* Fixes for multiple simulators running simultaneously
* Battery monitor support
* Hard fault handling support
* Contributors: BillThePlatypus, Cameron McQuinn, Daniel Koch, Jacob Willis, James Jackson, Parker Lusk, Trey Henrichsen

1.0.0 (2018-03-13)
------------------
* Update firmware to v1.1.0
* improved acceleration calculation in simulation
* some tweaks to make multi-agent possible
* fixed some issues in SIL
* working SIL multirotor
* Cleaned up simulation launch and parameter files
* updated board layer and python joy nodes
* fixed a memory file write bug
* able to arm, and disarm in simulation, IMU noise activated with motor spinning
* fixed timestamping issue with simulator (need to test on hardware)
* updated firmware
* Got UDP comms working with SIL in Gazebo
* launch files, xacro files, and aircraft parameters for generic multirotor and fixedwing aircraft are added.  They currently use a .dae that comes with gazebo, which might be confusing.  The best solution would be to create our own and include them here
* added some noise with no bias
* added sim, not tested, lacks udp and sensor noise
* Contributors: Daniel Koch, Gary Ellingson, James Jackson, Skyler Tolman, superjax08@gmail.com

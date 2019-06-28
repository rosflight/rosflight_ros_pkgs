^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosflight_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

FCU Common
===============

is a common package for the MAGICC fcu_io, ROSplane and ROScopter framework.

Its main purpose is to hold things that other packages depend on.  This includes message generation and common peripheral tools for dealing with joy sticks, gps, plotting, etc.

File Descriptions
---------------------------

Messages are named with a leading FW or MR to identify if the are for fixed wing or multirotor respectively. 

* msg folder is the meat of the package.  Most all other packages use the message defined here. Again FW identify messages for fixed wing aircraft and MR for multirotor.  

* fcu_common_joy
	This node translages from the joy node and the command message type.  Its purpose is to make joy stick reconfiguring a little easyer.  

* gps
	This node is a copy of the files at https://github.com/ros-drivers/nmea_navsat_driver with changes to combine the velocity and course messurements into the regular message.

* FW_est_plot
	This node plots the fixed wing estimate and truth side by side for estimator tunning. It should only work when the fcu is being simulated, which is the only case where truth is available.

* FW_cont_plot
	This node plots fixed wing commands next to the responce and can be used to tunne the controller loops.  It should work durring simulation and actual flying. 

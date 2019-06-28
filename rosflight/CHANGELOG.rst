^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosflight
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-03-13)
------------------
* Fixed Eigen build errors for debian
* Fixed eigen_stl_containers dependency
* Added timer to throttle rate at which params are sent
* made time retrieval 32 bit for millisecond and 64 bit for microsecond
* add includes for std libs to fix build
* use one timer with 10 Hz update
* Cleaned up rosconsole output on startup
* added declarations to rosflight_io header. Successfully tested on a board
* added callback and service advertisement for reboot_to_bootloader
* added header stamp
* some tweaks to the calibration routine
* got rid of stupid eigen size format warning
* added viz_mag to rosflight_utils
* Increased UDP buffer size for sim communications
* changed deque to STL containers in EigenSTL for proper memory alignment
* CMakelists update and include the boost library
* removed sensors from mavrosflight
* Print error codes by name
* Updated mavlink and status message
* updated Jerel's mag and made it a node
* fixed timestamping issue with simulator (need to test on hardware)
* updated firmware
* Got UDP comms working with SIL in Gazebo
* Added UDP support
* Abstracted serial communication layer
* Variable name change to be in accordance with ROS c++ style guide. Changed handle_small_range function name to handle_small_range_msg, to be consistent with the other message handler functions. Changed the if within handle_small_range to a switch. Compiled and run on my machine, works.
* Updated mavlink submodule to latest commit: 5f399ef
* More user-friendly autopilot error-code printing
* Contributors: Cameron McQuinn, Daniel Koch, Devon, Erich Nygaard, James Jackson, Jerel Nielsen, tyler

0.1.3 (2017-06-02)
------------------
* Temporarily removed magnetometer calibration
* Updated package.xml files
* Renamed sonar/data topic to sonar
* Changed yaml-cpp dependency to a PkgConfig module
* Contributors: Daniel Koch

0.1.2 (2017-05-24)
------------------
* Removed OpenMP compile flag for now
* Added missing tf dependency
* Contributors: Daniel Koch

0.1.1 (2017-05-24)
------------------
* Added missing dependencies
* Contributors: Daniel Koch

0.1.0 (2017-05-22)
------------------
* Added BSD license statements to source files
  Closes `#1 <https://github.com/rosflight/rosflight/issues/1>`_
* Added git as build dependency for rosflight
* Fixed system dependencies. Closes `#10 <https://github.com/rosflight/rosflight/issues/10>`_.
* cleanup of CMakeLists.txt
* cleanup of CMakeLists.txt
* automatic git submodule cloning
* Replaced outdated package README files with simpler top-level README
  The information that used to be in the package README files is now on the ROS wiki (http://wiki.ros.org/rosflight_pkgs, http://wiki.ros.org/rosflight, etc.)
  Closes `#7 <https://github.com/rosflight/rosflight/issues/7>`_
* Fixed rosflight_io runtime name
* Created the rosflight_msgs package and updated dependencies
* Restructured rosflight package include structure
* Renamed rosflight_io package to rosflight
* Contributors: Daniel Koch, James Jackson

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosflight_gcs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (pre-release)
------------------
* !! Renamed package to rosflight_gcs and moved sim-related utilites to rosflight_sim
* Updated viz files, rc_joy.py, rc_sim.py, and command_joy.py to ROS2
* Rewrote launch files using ROS2 python launch api
* Added launch and param files to initialize firmware for simulation flying
* Removed unused simple_pid, turbomath, WaypointConvert utils
* Removed postprocess utils due to lack of required functionality in ROS2
* Removed joy c++ node in favor of similar command_joy python node
* Removed outdated GPS driver in favor of external GPS libraries
* Re-implemented linter build checking using github actions
* Fixed airspeed reporting incorrect values
* Added doxygen documentation to almost all functions
* Contributors: Brandon Sutherland, Ian Reid, Brady Anderson

1.4.0 (2020-10-06)
------------------

1.3.1 (2020-03-27)
------------------

1.3.0 (2020-03-19)
------------------
* Cleaned up rc_joy node
* Cleaned up mag visualization
* Added attitude visualization
* Contributors: BillThePlatypus, Cameron McQuinn, Daniel Koch, Jacob Willis, James Jackson, Parker Lusk

1.0.0 (2018-03-13)
------------------
* parse GLONASS nmea sentences
* headless joy node
* added xbox to command mappings
* added Taranis mappings
* Fixed gps reporting of the number of satellites to report the actual number of satellites instead of a fixed number of 0 when not connected or 4 when connected.
* Contributors: Daniel Koch, Devon, Gary Ellingson, James Jackson, Jesse Wynn, pmarke, superjax08@gmail.com

0.1.3 (2017-06-02)
------------------
* Updated package.xml files
* Contributors: Daniel Koch

0.1.2 (2017-05-24)
------------------

0.1.1 (2017-05-24)
------------------
* Added missing dependencies
* Contributors: Daniel Koch

0.1.0 (2017-05-22)
------------------
* Added BSD license statements to source files
  Closes `#1 <https://github.com/rosflight/rosflight/issues/1>`_
* Replaced outdated package README files with simpler top-level README
  The information that used to be in the package README files is now on the ROS wiki (http://wiki.ros.org/rosflight_pkgs, http://wiki.ros.org/rosflight, etc.)
  Closes `#7 <https://github.com/rosflight/rosflight/issues/7>`_
* Renamed rosflight_common to rosflight_utils
* Contributors: Daniel Koch

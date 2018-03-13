^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosflight_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

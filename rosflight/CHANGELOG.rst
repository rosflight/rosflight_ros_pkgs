^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosflight
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

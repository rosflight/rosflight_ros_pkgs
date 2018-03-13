^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosflight_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#56 <https://github.com/rosflight/rosflight/issues/56>`_ from rosflight/gps_fix
  parse GLONASS nmea sentences
* parse GLONASS nmea sentences
* Merge pull request `#54 <https://github.com/rosflight/rosflight/issues/54>`_ from rosflight/fix_53
  compile without gazebo - gets warnings but works
* compile without gazebo - gets warnings but works
* Merge branch 'master' into reboot_to_bootloader
* Merge pull request `#45 <https://github.com/rosflight/rosflight/issues/45>`_ from rosflight/rc1.0
  Rc1.0
* Merge branch 'rc1.0' of github.com:rosflight/rosflight into rc1.0
* changed the gps driver gps topic and rc_joy error, potential fix for `#44 <https://github.com/rosflight/rosflight/issues/44>`_
* Merge branch 'rc1.0' of github.com:rosflight/rosflight into rc1.0
* improved acceleration calculation in simulation
* realized the error of my ways with these rc reverses
* Merge branch 'rc1.0' of github.com:rosflight/rosflight into rc1.0
* launch file for complete sim
* working SIL multirotor
* working SIL with relative nav stack
* Merge branch 'mag_calibration' of github.com:rosflight/rosflight into mag_calibration
* added viz_mag to rosflight_utils
* headless joy node
* added xbox to command mappings
* Merge pull request `#29 <https://github.com/rosflight/rosflight/issues/29>`_ from rosflight/remove_sensors
  Remove sensors
* repeat of `#18 <https://github.com/rosflight/rosflight/issues/18>`_
* merged changes
* Merge pull request `#28 <https://github.com/rosflight/rosflight/issues/28>`_ from rosflight/taranis
  Fixed Taranis mapping direction
* Fixed Taranis mapping direction
* Merge branch 'rc1.0' into error_printing
* Merge pull request `#25 <https://github.com/rosflight/rosflight/issues/25>`_ from rosflight/joystick_changes
  added Taranis mappings
* added Taranis mappings
* Merge branch 'rosflight_sim_pkg' into rc1.0
* updated board layer and python joy nodes
* fixed joy nodes
* added rc spoofing python nodes
* Merge branch 'master' into rosflight_sim_pkg
* Merge branch 'rosflight_sim_pkg' into udp_board
* fixed printf issue
* Merge pull request `#19 <https://github.com/rosflight/rosflight/issues/19>`_ from pmarke/undergrad
  Fixed gps reporting of the number of satellites to report the actual …
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

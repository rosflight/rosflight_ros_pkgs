# Changelog

## [2.0.0](https://github.com/rosflight/rosflight_ros_pkgs/compare/v2.0.0-beta...v2.0.0) (2026-02-03)


### Features

* add holoocean simulation files ([8018ecc](https://github.com/rosflight/rosflight_ros_pkgs/commit/8018ecc03d86b0b954fbfed7885f83e23aa58af1))
* Added realtime scheduling to rosflight_io and ability to import realtime configuration for other packages. ([e91bfeb](https://github.com/rosflight/rosflight_ros_pkgs/commit/e91bfebe74300d96f8f54816c280a3c5c80569d9))
* changed /sonar topic to /range ([57ffd47](https://github.com/rosflight/rosflight_ros_pkgs/commit/57ffd475550ad7b91ff4f064009b634f8c0378e7))
* update offboard command message to reflect new vector notation ([9cfc773](https://github.com/rosflight/rosflight_ros_pkgs/commit/9cfc773b7ae92087f42d17236ad14fe77f60fa9d))
* update rosflight_io mag param names to reflect new naming conventions ([#247](https://github.com/rosflight/rosflight_ros_pkgs/issues/247)) ([c5650f5](https://github.com/rosflight/rosflight_ros_pkgs/commit/c5650f5099a253970120ac922250c42a32f95965))


### Bug Fixes

* add docker_user to plugdev and dialout groups for USB access ([#240](https://github.com/rosflight/rosflight_ros_pkgs/issues/240)) ([1ca7715](https://github.com/rosflight/rosflight_ros_pkgs/commit/1ca7715d08b89c70f1d5989805344ab399a245b2))

## [2.0.0-beta](https://github.com/rosflight/rosflight_ros_pkgs/compare/v1.3.1...v2.0.0-beta) (2025-12-13)


### ⚠ BREAKING CHANGES

* update to ROS2

### Features

* add versioning ci using release-please ([0faaba4](https://github.com/rosflight/rosflight_ros_pkgs/commit/0faaba455c5ae46cc3cafd0fc9ad11419b23d4c6))
* added a service callback that reports whether or not all params are received from firmware. Also added publisher to publish when params change ([8592c4f](https://github.com/rosflight/rosflight_ros_pkgs/commit/8592c4fcd38df6cdee8c8d1df00e6c0a602bea29))
* added compose and Dockerfile for rosflight users ([bff5ef5](https://github.com/rosflight/rosflight_ros_pkgs/commit/bff5ef5363f3c738a1496ee42c4b9a93da48c7bf))
* Added getting mixer parameters on boot and when rosflight_io publishes that params have changed. Added mixer inversion as well ([eef1e1a](https://github.com/rosflight/rosflight_ros_pkgs/commit/eef1e1a780a2ee107e98aba5c72bc55a8c2fb207))
* added params file to the launch file to make sure mass parameters are synchronized between nodes ([58c53e5](https://github.com/rosflight/rosflight_ros_pkgs/commit/58c53e5d84f7d3727a5c256cf15e7673cac7ae9d))
* changed floating point multiplication to integer multiplication as per Copilot's suggestion ([37e7ec2](https://github.com/rosflight/rosflight_ros_pkgs/commit/37e7ec29c07efd85d4d9eb7653c6ab08a244558a))
* exposed firmware control gains to ROS2 parameter interface ([5e24f33](https://github.com/rosflight/rosflight_ros_pkgs/commit/5e24f3304b5df5db873d4a8c667cce503e79d1ec))
* IMU bias truth publishing ([#229](https://github.com/rosflight/rosflight_ros_pkgs/issues/229)) ([a188754](https://github.com/rosflight/rosflight_ros_pkgs/commit/a1887547a9be88ae5079daaacf1c998c9d957577))
* moved rosflight_firmware submodule to highest level so rosflight_io and sim can access it ([5291ac6](https://github.com/rosflight/rosflight_ros_pkgs/commit/5291ac6301fae29006e70d93db5031ef9262244b))
* Readme for docker files ([e71fb77](https://github.com/rosflight/rosflight_ros_pkgs/commit/e71fb771fed5879f1af2cbd15214e53cdf84dedd))
* removed pitch cancellation on collision with ground, since it helps with takeoff ([78074b8](https://github.com/rosflight/rosflight_ros_pkgs/commit/78074b825ae711d7231be1d9c3c808a21cfc4623))
* Small fixes to comments based on code review discussion, renaming variables and files, and converting transformations to Eigen transformations ([21bb5e6](https://github.com/rosflight/rosflight_ros_pkgs/commit/21bb5e6ae1db5099d7adf4028b7b65e45416e7a6))
* updated CI from iron ([fd53c3c](https://github.com/rosflight/rosflight_ros_pkgs/commit/fd53c3c395fedeb8c0db7143de916bdbf33be646))
* updated sil_board to reflect firmware board.h changes ([b4b123c](https://github.com/rosflight/rosflight_ros_pkgs/commit/b4b123ca04613c9802323356f22435b30cee3ecb))


### Bug Fixes

* changed angular and linear accels to compute derivatives in correct frame ([caef4ec](https://github.com/rosflight/rosflight_ros_pkgs/commit/caef4ec4408a5560f7c99e4866d46e8c4e5df250))
* enabled compiling on Jazzy ([18cb899](https://github.com/rosflight/rosflight_ros_pkgs/commit/18cb899ba89d136b8ff7ddd21fef078e0306cc77))
* enabled compiling on Jazzy ([eac7b0f](https://github.com/rosflight/rosflight_ros_pkgs/commit/eac7b0ffb8753a33488451d61f85cdd6eb7a3b23))
* fixed convenience launch script path to multirotor firmware file ([dee4172](https://github.com/rosflight/rosflight_ros_pkgs/commit/dee41721ea4ae4911c09605e2188f321569b8572))
* fixed convenience launch script path to multirotor firmware file ([09c3e2b](https://github.com/rosflight/rosflight_ros_pkgs/commit/09c3e2bf508bdf96e83a514dfc09295ff772a719))
* Fixed the declination rotation to be the correct sign. ([4f93a9a](https://github.com/rosflight/rosflight_ros_pkgs/commit/4f93a9a8c6f2b4bcbb3a8090682d1d0af7c582c9))
* Fixed the declination rotation to be the correct sign. ([4d6320e](https://github.com/rosflight/rosflight_ros_pkgs/commit/4d6320e2bd8761ba29c25dc2f8397a3d360230d3))
* moved set_sim_state function call to interface and added gazebo implementation ([d8cbf37](https://github.com/rosflight/rosflight_ros_pkgs/commit/d8cbf37a2ec3430dde748397bbc4f9134f88c88f))
* reorganized launch files to correctly load mass and inertia parameters on launch ([c875f46](https://github.com/rosflight/rosflight_ros_pkgs/commit/c875f462050c280ca12b11cc4e3ec3d1919be382))
* rosflight_sil_manager previously couldn't change params ([a559232](https://github.com/rosflight/rosflight_ros_pkgs/commit/a5592320ef364f426bda56dce3bcc0bcd5103722))
* viz transcriber path length publishes correct path length ([8d40a02](https://github.com/rosflight/rosflight_ros_pkgs/commit/8d40a02a6613cfd146c2da2e2b6f9102b02ed92e))
* wind can be changed by ros2 parameters. Renamed param to avoid double negative ([e9feb24](https://github.com/rosflight/rosflight_ros_pkgs/commit/e9feb2428ccb7b6a0349a1cf379b723cdde3656f))

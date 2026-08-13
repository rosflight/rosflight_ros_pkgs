cmake_minimum_required(VERSION 3.20...3.28)
project(rosflight_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# declare the message files to generate code for
set(msg_files
  "msg/Airspeed.msg"
  "msg/Attitude.msg"
  "msg/AuxCommand.msg"
  "msg/Barometer.msg"
  "msg/BatteryStatus.msg"
  "msg/Command.msg"
  "msg/Error.msg"
  "msg/GNSS.msg"
  "msg/OutputRaw.msg"
  "msg/PwmOutput.msg"
  "msg/RangeFinderSensor.msg"
  "msg/RCRaw.msg"
  "msg/RGBCamera.msg"
  "msg/Status.msg"
  "msg/SimState.msg"
)

# declare the service files to generate code for
set(srv_files
  "srv/ParamFile.srv"
  "srv/ParamGet.srv"
  "srv/ParamSet.srv"
  "srv/SetSimState.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

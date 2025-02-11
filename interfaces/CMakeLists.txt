cmake_minimum_required(VERSION 3.5)

project(interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ImuReset.srv"
  "srv/AngularPositionMove.srv"
  "srv/LinearPositionMove.srv"
  "srv/ParameterRead.srv"
  "srv/ParameterWrite.srv"
  "srv/SetMoveMode.srv"
  "srv/ConveyorAutoMovement.srv"
  "srv/ConveyorDataRead.srv"
  "srv/ConveyorManualMovement.srv"
  "srv/ConveyorParameterRead.srv"
  "srv/ConveyorParameterWrite.srv"
  "srv/Integrallog.srv"
  "srv/LedControl.srv"
  "srv/LedToggleControl.srv"
  "srv/PowerAdcRead.srv"
  "srv/PowerDataRead.srv"
  "srv/PowerGetIoStatus.srv"
  "srv/PowerParameterRead.srv"
  "srv/PowerParameterWrite.srv"
  "srv/PowerSetEnable.srv"
  "srv/PowerSetOutport.srv"
  "srv/PowerSetSingleEnable.srv"
  "srv/PowerSetSingleOutport.srv"
  "srv/PowerSonarCmd.srv"
  "srv/PowerSonarRead.srv"
  "srv/PowerVersionRead.srv"
  "srv/PowerWheelEnable.srv"
  "srv/ToggleOn.srv"
  "srv/DockingControl.srv"
  "srv/SaveMaker.srv"
  "srv/DockingStop.srv"
  "srv/SaveMap.srv"
  "srv/RunMapping.srv"
  "srv/RunNavigation.srv"
  "srv/GetInformation.srv"
  "srv/GotoLocation.srv"
  "srv/GotoLocation2.srv"
  "srv/SetLocation.srv"
  "srv/GotoCancel.srv"
  "srv/RunCollisionMonitor.srv"
  "srv/SetMaxspeed.srv"
  "msg/Gpio.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs std_srvs
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()

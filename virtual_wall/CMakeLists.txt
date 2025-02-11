cmake_minimum_required(VERSION 3.5)
project(virtual_wall)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Ensure rosidl_generate_interfaces runs first
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddVirtualWall.srv"
  DEPENDENCIES geometry_msgs
)

# Link the generated interfaces before building executables
find_package(rosidl_default_runtime REQUIRED)

# Ensure generated headers are included
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

# Add Executable
add_executable(virtual_wall_node src/virtual_wall.cpp)

# Set dependencies for the executable
set(dependencies
  rclcpp
  rmw_implementation
  std_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(virtual_wall_node ${dependencies})

# Ensure message generation completes before building the node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(virtual_wall_node ${cpp_typesupport_target})

# Install
install(TARGETS
  virtual_wall_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

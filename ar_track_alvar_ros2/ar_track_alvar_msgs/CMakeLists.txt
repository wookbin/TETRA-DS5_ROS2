cmake_minimum_required(VERSION 3.8)
project(ar_track_alvar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ar_track_alvar_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
set(SOURCE
    include/ar_track_alvar/tinystr.cpp
    include/ar_track_alvar/tinystr.h
    include/ar_track_alvar/tinyxml.cpp
    include/ar_track_alvar/tinyxml.h
    include/ar_track_alvar/tinyxmlerror.cpp
    include/ar_track_alvar/tinyxmlparser.cpp
    src/Alvar.cpp
    src/Bitset.cpp
    src/Camera.cpp
    src/CaptureDevice.cpp
    src/CaptureFactory.cpp
    src/CaptureFactory_unix.cpp
    src/ConnectedComponents.cpp
    src/CvTestbed.cpp
    src/DirectoryIterator.cpp
    src/DirectoryIterator_unix.cpp
    src/Draw.cpp
    # src/FileFormatUtils.cpp
    src/Filter.cpp
    src/Kalman.cpp
    src/kinect_filtering.cpp
    src/Line.cpp
    src/Marker.cpp
    src/MarkerDetector.cpp
    src/medianFilter.cpp
    src/MultiMarker.cpp
    src/MultiMarkerBundle.cpp
    src/MultiMarkerInitializer.cpp
    src/Mutex.cpp
    src/Mutex_unix.cpp
    src/Optimization.cpp
    src/Platform.cpp
    src/Plugin.cpp
    src/Plugin_unix.cpp
    src/Pose.cpp
    src/Ransac.cpp
    src/Rotation.cpp
    src/Threads.cpp
    src/Threads_unix.cpp
    src/Util.cpp
)

add_executable(individual_markers_no_kinect src/IndividualMarkersNoKinect.cpp ${SOURCE})
ament_target_dependencies(individual_markers_no_kinect rclcpp tf2_geometry_msgs resource_retriever tf2 tf2_ros geometry_msgs std_msgs visualization_msgs sensor_msgs ar_track_alvar_msgs pcl_conversions image_transport cv_bridge pcl_ros Eigen3 OpenCV)

install(TARGETS
  individual_markers_no_kinect
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  	launch
  	DESTINATION share/${PROJECT_NAME})

ament_package()

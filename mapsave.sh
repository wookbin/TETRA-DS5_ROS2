#!/bin/bash
map_name=$1

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd /home/tetra/ros2_ws/src/tetra_navigation2/maps/

ros2 run nav2_map_server map_saver_cli -f ${map_name}
#ros2 run nav2_map_server map_saver_cli -f ${map_name} --ros-args -p save_map_timeout:=10000


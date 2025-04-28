#!/bin/bash
map_name=$1

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

while pgrep -f mapping.sh > /dev/null; do
 echo "wait for mapping die";
 sleep 1
done
# pkill -f cartographer.launch.py

#ros2 launch tetra_bringup test.navigation.launch.py map_name:=${map_name}
ros2 launch tetra_navigation2 tetra_navigation.launch.py map_name:=${map_name}
sleep 2

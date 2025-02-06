#!/bin/bash

map_name="office" #$1

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo $map_name

#pkill -9 -f nav2_controller
#sleep 1

#ros2 launch tetra_bringup test.navigation.launch.py map_name:=${map_name}
ros2 launch tetra_navigation2 tetra_navigation.launch.py map_name:=${map_name}
sleep 2


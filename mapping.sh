#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

#pkill -9 -f nav2_controller
#sleep 1

ros2 launch tetra_cartographer cartographer.launch.py

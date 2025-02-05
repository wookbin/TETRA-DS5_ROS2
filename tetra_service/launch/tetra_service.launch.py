#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    tetra_service_node = Node(
        package='tetra_service', 
        executable='tetra_service',
        output='screen'
    )

    # create and return launch description object
    return LaunchDescription(
        [
            tetra_service_node
        ]
    )

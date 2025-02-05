#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    tetra_interface_node = Node(
        package='tetra_interface', 
        executable='tetra_interface',
        output='screen',
        parameters=[
            {"m_bConveyor_option": False},
            {"m_bUltrasonic_option": False}
        ]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            tetra_interface_node
        ]
    )

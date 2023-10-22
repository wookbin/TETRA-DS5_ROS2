#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
  
  ar_track_alvar_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'ar_track_alvar_parameter.yaml'
  )
  ar_track_alvar_node = Node(
    package="ar_track_alvar",
    executable="individual_markers_no_kinect",
    name="ar_track_alvar",
    output="screen",
    parameters=[{"output_frame": "camera"},
                ar_track_alvar_parameter],
  )
  
  return LaunchDescription([
    ar_track_alvar_node,
	])

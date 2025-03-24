import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bme_ros2_navigation = get_package_share_directory('tetra_navigation2')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Flag to enable use_sim_time'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='office.yaml',
        description='Name of the map file to use'
    )


    # Path to the Slam Toolbox launch file
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'launch',
        'navigation_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'params',
        'amcl_localization.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('tetra_navigation2'),
        'params',
        'navigation.yaml'
    )

    #map_file_path = os.path.join(
    #    get_package_share_directory('tetra_navigation2'),
    #    'maps',
    #    'office.yaml'
    #)
    
    # Dynamically constructed paths
    map_file_path = PathJoinSubstitution([
        pkg_bme_ros2_navigation, 'maps', LaunchConfiguration('map_name')
    ])

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
                'map': map_file_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject

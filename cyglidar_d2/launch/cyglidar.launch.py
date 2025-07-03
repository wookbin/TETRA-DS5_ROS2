from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
import launch_ros.actions

def generate_launch_description():

    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate", default_value = TextSubstitution(text="0"),
        description = "baud rate value [0:(3,000,000), 1:(921,600), 2:(115,200), 3:(57,600)]")

    run_mode_arg = DeclareLaunchArgument(
        "run_mode", default_value = TextSubstitution(text="2"),
        description = "version type [0(2D), 1(3D), 2(2D/3D)]")

    frequency_channel_arg = DeclareLaunchArgument(
        "frequency_channel", default_value = TextSubstitution(text="0"),
        description = "frequency Ch. [0 to 15]")

    duration_mode_arg = DeclareLaunchArgument(
        "duration_mode", default_value = TextSubstitution(text="0"),
        description = "pulse mode [0 (Auto), 1(Fixed)]")

    duration_value_arg = DeclareLaunchArgument(
        "duration_value", default_value = TextSubstitution(text="10000"),
        description = "pulse duration [0 to 10000] ")

    color_mode_arg = DeclareLaunchArgument(
        "color_mode", default_value = TextSubstitution(text="0"),
        description = "color mode [0 (HUE), 1 (RGB), 2 (GRAY)]")

    data_type_3d_arg = DeclareLaunchArgument(
        "data_type_3d", default_value = TextSubstitution(text="0"),
        description = "3D data type [0 (DISTANCE), 1 (AMPLITUDE)]")

    filter_mode_arg = DeclareLaunchArgument(
        "filter_mode", default_value = TextSubstitution(text="0"),
        description = "New 3D Data Filtering [0 (None), 1 (Median Filter), 2 (Average Filter)]")

    edge_filter_value_arg = DeclareLaunchArgument(
        "edge_filter_value", default_value = TextSubstitution(text="0"),
        description = "Edge Filtering Threshold")

    enable_kalmanfilter_arg = DeclareLaunchArgument(
        "enable_kalmanfilter", default_value = TextSubstitution(text="True"),
        description = "Kalman Filtering [False (Raw Distance Data), True (Kalman Filter Applied Distance Data)]")

    enable_clahe_arg = DeclareLaunchArgument(
        "enable_clahe", default_value = TextSubstitution(text="False"),
        description = "Ampliutde 3D type [False (Raw Amplitude Data), True (CLAHE Applied Amplitude Data)]")

    clahe_cliplimit_arg = DeclareLaunchArgument(
        "clahe_cliplimit", default_value = TextSubstitution(text="40"),
        description = "ClipLimit from 0 to 255 / Default : 40")

    clahe_tiles_grid_size_arg = DeclareLaunchArgument(
        "clahe_tiles_grid_size", default_value = TextSubstitution(text="8"),
        description = "Tiles Grid Size from 1 to 15")


    lidar_node = launch_ros.actions.Node(
        package = 'cyglidar_d2_ros2',
        executable = 'cyglidar_d2_publisher',
        output = 'screen',
        parameters=[
           {"port_number": "/dev/cyglidar"},
           {"baud_rate": LaunchConfiguration("baud_rate")},
           {"frame_id": "laser_link2"},
           {"fixed_frame": "/map"},
           {"run_mode": LaunchConfiguration("run_mode")},
           {"frequency_channel": LaunchConfiguration("frequency_channel")},
           {"duration_mode": LaunchConfiguration("duration_mode")},
           {"duration_value": LaunchConfiguration("duration_value")},
           {"color_mode": LaunchConfiguration("color_mode")},
           {"data_type_3d": LaunchConfiguration("data_type_3d")},
           {"filter_mode": LaunchConfiguration("filter_mode")},
           {"edge_filter_value": LaunchConfiguration("edge_filter_value")},
           {"enable_kalmanfilter": LaunchConfiguration("enable_kalmanfilter")},
           {"enable_clahe": LaunchConfiguration("enable_clahe")},
           {"clahe_cliplimit": LaunchConfiguration("clahe_cliplimit")},
           {"clahe_tiles_grid_size": LaunchConfiguration("clahe_tiles_grid_size")}
        ],
        remappings=[
            ('/scan', '/scan2'),
        ]
    )

    # tf_node = launch_ros.actions.Node(
    #     package = 'tf2_ros', executable = "static_transform_publisher", name="to_laserframe",
    #     arguments = ["0", "0", "0", "0", "0", "0", "map", "laser_frame"]
    # )

    ld = LaunchDescription()

    ld.add_action(baud_rate_arg)
    ld.add_action(run_mode_arg)
    ld.add_action(frequency_channel_arg)
    ld.add_action(duration_mode_arg)
    ld.add_action(duration_value_arg)
    ld.add_action(color_mode_arg)
    ld.add_action(data_type_3d_arg)
    ld.add_action(filter_mode_arg)
    ld.add_action(edge_filter_value_arg)
    ld.add_action(enable_kalmanfilter_arg)
    ld.add_action(enable_clahe_arg)
    ld.add_action(clahe_cliplimit_arg)
    ld.add_action(clahe_tiles_grid_size_arg)
    ld.add_action(lidar_node)
    #ld.add_action(tf_node)

    return ld

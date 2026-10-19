"""
Mapping launch — builds a map from live LiDAR + odometry.

Usage (from the PC, with the robot already running on the Pi):
    ros2 launch jidenna_bringup mapping.launch.py

Requirements on the Pi:
    ros2 launch jidenna_bringup real.launch.py   # in another terminal

What this does:
    - Starts slam_toolbox (online async mode)
    - Starts RViz with mapping-friendly displays
    - Saves the map when you press Ctrl+C? No — see map_save instructions below.

To save the map after driving:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("jidenna_controller")
    slam_share = get_package_share_directory("slam_toolbox")

    default_slam_params = os.path.join(pkg, "config", "slam.yaml")
    default_rviz        = os.path.join(pkg, "config", "mapping.rviz")

    slam_params = LaunchConfiguration("slam_params_file")
    rviz_cfg    = LaunchConfiguration("rviz_config")

    declare_slam = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_slam_params,
        description="Path to slam_toolbox params YAML",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Path to RViz config file",
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([
        declare_slam,
        declare_rviz,
        slam,
        rviz,
    ])
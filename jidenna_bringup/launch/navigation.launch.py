"""
Navigation launch — drives autonomously on a saved map.

Usage (from the PC, with the robot already running on the Pi):
    ros2 launch jidenna_bringup navigation.launch.py \
        map:=$HOME/maps/my_map.yaml

Requirements on the Pi:
    ros2 launch jidenna_bringup real.launch.py   # in another terminal

What this does:
    - Loads the saved map
    - Runs AMCL to localize the robot on that map
    - Runs Nav2 (planner, controller, costmaps, BT navigator)
    - Relays Nav2's cmd_vel output to the Pi's cmd_vel_unstamped
    - Opens RViz with navigation displays

After launch:
    - In RViz, click "2D Pose Estimate" to set the robot's initial pose on the map
    - Click "Nav2 Goal" to send a goal
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
    nav2_share = get_package_share_directory("nav2_bringup")

    default_map    = os.path.expanduser("~/maps/jidenna_office.yaml")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")
    default_rviz   = os.path.join(pkg, "config", "navigation.rviz")

    map_yaml    = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_cfg    = LaunchConfiguration("rviz_config")
    use_sim     = LaunchConfiguration("use_sim_time")

    declare_map = DeclareLaunchArgument(
        "map", default_value=default_map,
        description="Full path to the map YAML file",
    )
    declare_params = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Full path to nav2_params.yaml",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz,
        description="Path to RViz config file",
    )
    declare_sim = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation time",
    )

    # --- Nav2 bringup (map + AMCL + planners + controllers + BT) ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    relay = Node(
        package="topic_tools",
        executable="relay",
        name="nav2_cmd_vel_relay",
        arguments=[
            "/cmd_vel_nav",
            "/jidenna_ros_controller/cmd_vel_unstamped",
        ],
        output="screen",
    )


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_rviz,
        declare_sim,
        nav2,
        relay,
        rviz,
    ])
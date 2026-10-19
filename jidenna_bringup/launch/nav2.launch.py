import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the saved map (EDIT THIS)
    default_map = os.path.expanduser("~/maps/jidenna_office.yaml")

    # Path to the params file (EDIT THIS)
    default_params = os.path.expanduser(
        "~/jidenna_ws/src/jidenna_controller/config/nav2_params.yaml"
    )

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_map = DeclareLaunchArgument(
        "map", default_value=default_map,
        description="Full path to the map YAML file"
    )
    declare_params = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Full path to the Nav2 params YAML file"
    )
    declare_use_sim = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation time"
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )
    relay = Node(
        package="topic_tools",
        executable="relay",
        name="nav2_cmd_vel_relay",
        arguments=[
            "/cmd_vel_nav",                              # Nav2's output
            "/jidenna_ros_controller/cmd_vel_unstamped", # Pi's input
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim,
        bringup,
        relay,
    ])
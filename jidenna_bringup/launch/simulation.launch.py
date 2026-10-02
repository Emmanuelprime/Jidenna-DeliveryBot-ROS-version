import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory("jidenna_description")
    pkg_ctrl = get_package_share_directory("jidenna_controller")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    xacro_file = os.path.join(pkg_desc, "urdf", "main.urdf.xacro")
    ctrl_yaml  = os.path.join(pkg_ctrl, "config", "jidenna_controllers.yaml")


    robot_description = Command([
        "xacro ", xacro_file,
        " is_sim:=true",
        " controllers_yaml:=", ctrl_yaml,
    ])


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": "empty"}.items(),
    )


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )


    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "jidenna",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
        output="screen",
    )


    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ctrl, "launch", "controller.launch.py")
        ),
    )

    start_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[controller],
        )
    )


    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_relay",
        arguments=[
            "/cmd_vel",
            "/jidenna_ros_controller/cmd_vel_unstamped",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        start_controllers_after_spawn,
        cmd_vel_relay,
    ])
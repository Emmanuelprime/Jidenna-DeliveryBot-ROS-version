import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



def generate_launch_description():

    pkg_desc = get_package_share_directory("jidenna_description")
    pkg_ctrl = get_package_share_directory("jidenna_controller")

    xacro_file = os.path.join(pkg_desc, "urdf", "main.urdf.xacro")
    ctrl_yaml  = os.path.join(pkg_ctrl, "config", "jidenna_controllers.yaml")

    # Expand xacro with is_sim:=false so the ROS 2 control block emits
    # the JidennaArduinoHardware plugin instead of the Gazebo one.
    robot_description = Command([
        "xacro ", xacro_file,
        " is_sim:=false",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            ctrl_yaml,
            {"use_sim_time": False},
        ],
        output="screen",
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )


    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "jidenna_ros_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    delay_diff_drive_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    start_jsb_after_control = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(pkg_desc, "config", "display.rviz")],
        )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jidenna_controller"),
                "launch",
                "localization.launch.py",
            )
        ),
    )

    start_localization_after_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[localization],
        )
    )


    return LaunchDescription([
        robot_state_publisher,
        control_node,
        start_jsb_after_control,
        delay_diff_drive_after_jsb,
        rviz_node,
        start_localization_after_ctrl,
    ])
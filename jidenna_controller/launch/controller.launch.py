import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "jidenna_ros_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # velocity_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "jidenna_controller",
    #         "--controller-manager", "/controller_manager",
    #     ],
    #     output="screen",
    # )

    joy_node =  Node(
            package="jidenna_controller",
            executable="teleop_joy.py",
            name="joystick_udp",
            output="screen",
            parameters=[{
                "udp_port": 4210,
                "listen_address": "0.0.0.0",
                "cmd_vel_topic": "/jidenna_ros_controller/cmd_vel_unstamped",
                "publish_rate": 50.0,
                "timeout_s": 0.5,
                "max_linear": 0.6,       # m/s
                "max_angular": 1.5,      # rad/s
                "deadzone": 0.05,
                "invert_linear": False,
                "invert_angular": False,
            }],
        )

    delay_diff_drive_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        delay_diff_drive_after_jsb,
        # velocity_controller_spawner,
        # joy_node,
    ])
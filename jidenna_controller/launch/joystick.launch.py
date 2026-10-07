from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="jidenna_controller",
            executable="teleop_joy",
            name="teleop_joy",
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
        ),
    ])
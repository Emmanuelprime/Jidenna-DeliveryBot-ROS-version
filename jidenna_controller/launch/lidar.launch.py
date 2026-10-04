from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jidenna_controller',
            executable='ydlidar_node',
            name='ydlidar_prime_node',
            output='screen',
        ),
    ])

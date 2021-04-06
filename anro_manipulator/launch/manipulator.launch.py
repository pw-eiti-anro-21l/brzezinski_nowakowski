from launch import LaunchDescription
import math

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                    package='anro_manipulator',
                    executable='state_publisher',
                    name='state_publisher',
                    output='screen'),
    ])

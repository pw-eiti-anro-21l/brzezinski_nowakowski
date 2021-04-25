from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                    package='anro_manipulator',
                    executable='nonkdl_dkin',
                    name='NONKDL_DKIN',
                    output='screen'),
    ])
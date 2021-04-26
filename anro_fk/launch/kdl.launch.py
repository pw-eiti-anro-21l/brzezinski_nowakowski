from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                    package='anro_fk',
                    executable='kdl_dkin',
                    name='KDL_DKIN',
                    output='screen'),
    ])
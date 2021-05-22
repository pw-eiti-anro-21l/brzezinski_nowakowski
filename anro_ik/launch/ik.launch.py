from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                    package='anro_interpolation',
                    executable='oint_control_srv',
                    name='OINT_CONTROL_SRV',
                    output='screen'),
            Node(
                    package='anro_ik',
                    executable='state_publisher',
                    name='state_publisher',
                    output='screen')
    ])
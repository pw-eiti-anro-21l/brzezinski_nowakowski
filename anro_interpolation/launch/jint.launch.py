from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
            Node(
                    package='anro_interpolation',
                    executable='jint_control_srv',
                    name='JINT_CONTROL_SRV',
                    output='screen')
    ])
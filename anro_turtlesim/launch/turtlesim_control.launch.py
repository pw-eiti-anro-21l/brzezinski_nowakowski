from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            # namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        # Node(
        #     package='anro_turtlesim',
        #     executable='turtlesim_control',
        #     name='control',
        #     output='screen'
        # )
        Node(
            package='anro_turtlesim',
            executable='turtlesim_control',
            name='control',
            # output='screen',
            prefix=['gnome-terminal -- ']
        )
    ])
# <launch>

#  <!-- Turtlesim Node-->
#   <node pkg="turtlesim" type="turtlesim_node" name="turtlesim"/>

#  <!-- Axes -->
#   <node pkg="anro_turtlesim" type="turtlesim_control" name="turtlesim_control" output="screen"/>

# </launch>
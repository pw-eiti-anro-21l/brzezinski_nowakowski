import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_file_name = 'manipulator.rviz'
    urdf_file_name = 'manipulator.urdf.xml'

    rviz = os.path.join(
            get_package_share_directory('anro_interpolation'),
            rviz_file_name)
    urdf = os.path.join(
            get_package_share_directory('anro_manipulator'),
            urdf_file_name)

    return LaunchDescription([
            Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=[urdf]
                    ),
            Node(
                    package='rviz2',
                    executable='rviz2',
                    name='anro_interpolation_rviz2_jint',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-d', rviz])
    ])
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
            get_package_share_directory('anro_fk'),
            rviz_file_name)
    urdf = os.path.join(
            get_package_share_directory('anro_manipulator'),
            urdf_file_name)

    return LaunchDescription([
            Node(
                    package='anro_fk',
                    executable='kdl_dkin',
                    name='KDL_DKIN',
                    output='screen'),
            Node(
                    package='anro_fk',
                    executable='nonkdl_dkin',
                    name='NONKDL_DKIN',
                    output='screen')
    ])
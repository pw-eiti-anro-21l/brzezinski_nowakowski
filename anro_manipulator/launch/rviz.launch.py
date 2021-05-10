import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node


def generate_launch_description():

    rviz = LaunchConfiguration('rviz', default='')
    fixed = LaunchConfiguration('fixed', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_file_name = 'manipulator.rviz'
    urdf_file_name = 'manipulator.urdf.xml'
    urdf_fixed_file_name = 'manipulator.fixed.urdf.xml'
    #if rviz == '':
    rviz = os.path.join(
            get_package_share_directory('anro_manipulator'),
            rviz_file_name)
    urdf = os.path.join(
            get_package_share_directory('anro_manipulator'),
            urdf_file_name)
    urdf_fixed = os.path.join(
            get_package_share_directory('anro_manipulator'),
            urdf_fixed_file_name)
    return LaunchDescription([
            DeclareLaunchArgument(
                    'use_sim_time',
                    default_value='false',
                    description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument(
                    'fixed',
                    default_value='false',
                    description='Use fixed model'),
            DeclareLaunchArgument(
                    'rviz',
                    default_value='',
                    description='Rviz config path'),
            Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=[urdf_fixed],
                    condition=IfCondition(fixed)
                    ),
            Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=[urdf],
                    condition=UnlessCondition(fixed)
                    ),
            Node(
                    package='rviz2',
                    executable='rviz2',
                    name='anro_manipulator_rviz2',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-d', rviz]),
    ])
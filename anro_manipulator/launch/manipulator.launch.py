import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'manipulator_fixed.urdf.xml'
  xacro_file_name = 'manipulator_fixed.xacro.xml'

  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('anro_manipulator'),
      urdf_file_name)
  xacro = os.path.join(
      get_package_share_directory('anro_manipulator'),
      xacro_file_name)
  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
    #   ExecuteProcess(cmd=['/bin/bash', '-c', 'xacro ' + xacro + ' -o ' + urdf]),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
          arguments=[urdf]),
      Node(
          package='anro_manipulator',
          executable='state_publisher',
          name='state_publisher',
          output='screen'),
  ])
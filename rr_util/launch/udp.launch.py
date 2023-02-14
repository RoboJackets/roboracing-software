import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
  udp_pub = Node(
    package='rr_util', 
    executable='udp_pub.py',
    output='screen'
  )
  
  udp_sub = Node(
    package='rr_util',
    executable='udp_sub.py',
    output='screen'
  )

  return LaunchDescription([
    udp_pub,
    udp_sub
  ])
from distutils.spawn import spawn
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
  pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
  pkg_rr_gazebo = get_package_share_directory("rr_gazebo")
  world_path = os.path.join(pkg_rr_gazebo, "worlds/ev_grand_prix.world")
  print(world_path)
  gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
      ),
      launch_arguments={"world": world_path}.items()
  )

  xacro_file = os.path.join(get_package_share_directory('rr_description'), 'urdf/', 'rigatoni.urdf.xacro')
  assert os.path.exists(xacro_file), "rigatoni.urdf.xacro doesnt exist in "+str(xacro_file)

  robot_description_config = process_file(xacro_file)
  robot_desc = robot_description_config.toxml()

  spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'rigatoni', '-topic', '/robot_description']
  )
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[
        {"robot_description": robot_desc}],
    output='screen'
  )

  return LaunchDescription([
      gazebo,
      spawn_robot,
      robot_state_publisher
  ])
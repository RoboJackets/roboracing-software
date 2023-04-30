from distutils.spawn import spawn
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
  DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time')

  pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
  velodyne = get_package_share_directory('velodyne_description')
  pkg_rr_gazebo = get_package_share_directory("rr_gazebo")
  world_path = os.path.join(pkg_rr_gazebo, "worlds/ev_grand_prix.world")
  rviz_config_file = os.path.join(velodyne, 'rviz', 'example.rviz')
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
    arguments=['-entity', 'rigatoni', '-topic', '/robot_description'],
  
    remappings=[
            ('/cmd_vel', '/tricyle_controller/cmd_vel')
        ]
  )
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[
        {"robot_description": robot_desc}],
    output='screen'
  )

  joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
  )

  joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
  )
  load_tricycle_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'tricycle_controller'],
        output='screen'
  )
  # robot_localization_node = Node(
  #      package='robot_localization',
  #      executable='ekf_node',
  #      name='ekf_filter_node',
  #      output='screen',
  #      parameters=[os.path.join(pkg_rr_gazebo, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  # ) 

  cloud_to_laser =  Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', ['/velodyne_points']),
                        ('scan', ['/scan'])],
            parameters=[{'range_min': 2.0}],
            name='pointcloud_to_laserscan'
        )

  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )

  return LaunchDescription([
      gazebo,
      spawn_robot,
      robot_state_publisher,
      joint_state_publisher_node,
      joint_state_publisher_gui_node,
      load_tricycle_controller,
      # robot_localization_node,
      cloud_to_laser,
      start_rviz_cmd
  ])
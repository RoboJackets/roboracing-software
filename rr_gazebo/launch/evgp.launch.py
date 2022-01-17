# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#     pkg_rr_gazebo = get_package_share_directory('rr_gazebo')

#     # Gazebo launch
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
#         )
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'world',
#             default_value=[os.path.join(pkg_rr_gazebo, 'worlds', 'ev_grand_prix.world'), ''],
#             description='SDF world file'
#         ),
#         gazebo
#     ])

# import os
# import launch
# from launch import LaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess
# from ament_index_python.packages import get_package_share_directory, get_package_prefix

# def generate_launch_description():
#     world_path = os.sep.join(
#         [get_package_share_directory('rr_gazebo'), 'worlds', 'ev_grand_prix_cones.world']
#     )

#     gazebo_launch_path = os.sep.join(
#         [get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py']
#     )
    
#     gazebo_model_path = os.sep.join(
#         [get_package_share_directory('rr_gazebo'), 'models']
#     )

#     return LaunchDescription([
#         SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
#         SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(gazebo_launch_path),
#             launch_arguments={
#                 'world': world_path,
#                 'verbose': 'false',
#                 'gui_required': 'true',
#                 'server_required': 'false',
#                 'factory': 'false'
#             }.items()
#         )
#     ])

# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, LaunchConfiguration, PythonExpression
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
 
# def generate_launch_description():
 
#   # Set the path to the Gazebo ROS package
#   pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
   
#   # Set the path to this package.
#   pkg_share = FindPackageShare(package='rr_gazebo').find('rr_gazebo')
 
#   # Set the path to the world file
#   world_file_name = 'ev_grand_prix.world'
#   world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
#   # Set the path to the SDF model files.
#   gazebo_models_path = os.path.join(pkg_share, 'models')
#   os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
 
#   ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
#   # Launch configuration variables specific to simulation
#   headless = LaunchConfiguration('headless')
#   use_sim_time = LaunchConfiguration('use_sim_time')
#   use_simulator = LaunchConfiguration('use_simulator')
#   world = LaunchConfiguration('world')
 
#   declare_simulator_cmd = DeclareLaunchArgument(
#     name='headless',
#     default_value='False',
#     description='Whether to execute gzclient')
     
#   declare_use_sim_time_cmd = DeclareLaunchArgument(
#     name='use_sim_time',
#     default_value='true',
#     description='Use simulation (Gazebo) clock if true')
 
#   declare_use_simulator_cmd = DeclareLaunchArgument(
#     name='use_simulator',
#     default_value='True',
#     description='Whether to start the simulator')
 
#   declare_world_cmd = DeclareLaunchArgument(
#     name='world',
#     default_value=world_path,
#     description='Full path to the world model file to load')
    
#   # Specify the actions
   
#   # Start Gazebo server
#   start_gazebo_server_cmd = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
#     condition=IfCondition(use_simulator),
#     launch_arguments={'world': world}.items())
 
#   # Start Gazebo client    
#   start_gazebo_client_cmd = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
#     condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 
#   # Create the launch description and populate
#   ld = LaunchDescription()
 
#   # Declare the launch options
#   ld.add_action(declare_simulator_cmd)
#   ld.add_action(declare_use_sim_time_cmd)
#   ld.add_action(declare_use_simulator_cmd)
#   ld.add_action(declare_world_cmd)
 
#   # Add any actions
#   ld.add_action(start_gazebo_server_cmd)
#   ld.add_action(start_gazebo_client_cmd)
 
#   return ld


# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, LaunchConfiguration, PythonExpression
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
 
 
# def generate_launch_description():
#   pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#   pkg_rr_gazebo = get_package_share_directory('rr_gazebo')
#   pkg_rr_description = get_package_share_directory('rr_description')
 
 
#   # # Constants for paths to different files and folders
#   # gazebo_models_path = 'models'
#   # package_name = 'rr_gazebo'
#   robot_name_in_model = 'rigatoni'
#   # rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz'
#   # urdf_file_path = 'urdf/two_wheeled_robot_nav2.urdf'
#   world_file_path = 'worlds/ev_grand_prix_cones.world'
     
#   # Pose where we want to spawn the robot
#   spawn_x_val = '0.0'
#   spawn_y_val = '0.0'
#   spawn_z_val = '0.0'
#   spawn_yaw_val = '0.00'
 
#   ############ If you're lucky you shouldn't need to change anything below this line #############
   
#   # # Set the path to different files and folders.  
#   # pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
#   # pkg_share = FindPackageShare(package=package_name).find(package_name)
#   # default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
#   # default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
#   # world_path = os.path.join(pkg_share, world_file_path)
#   # gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
#   # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  

#   bot_urdf_path = 'urdf/rigatoni.urdf'

#   gazebo_models_path = 'models'
#   pkg_rr_gazebo = FindPackageShare(package='rr_gazebo').find('gazebo_ros')
#   pkg_rr_description = FindPackageShare(package='rr_description').find('rr_description')
#   default_urdf_model_path = os.path.join(pkg_rr_description, bot_urdf_path)
#   world_path = os.path.join(pkg_rr_gazebo, )

#   #todo add / find an rviz config file lolz
#   # default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)

   
#   # Launch configuration variables specific to simulation
#   gui = LaunchConfiguration('gui')
#   headless = LaunchConfiguration('headless')
#   namespace = LaunchConfiguration('namespace')
#   # rviz_config_file = LaunchConfiguration('rviz_config_file')
#   urdf_model = LaunchConfiguration('urdf_model')
#   use_namespace = LaunchConfiguration('use_namespace')
#   use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
#   use_rviz = LaunchConfiguration('use_rviz', default=False)
#   use_sim_time = LaunchConfiguration('use_sim_time')
#   use_simulator = LaunchConfiguration('use_simulator')
#   world = LaunchConfiguration('world')
   
#   # Declare the launch arguments  
#   declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
#     name='gui',
#     default_value='True',
#     description='Flag to enable joint_state_publisher_gui')
     
#   declare_namespace_cmd = DeclareLaunchArgument(
#     name='namespace',
#     default_value='',
#     description='Top-level namespace')
 
#   declare_use_namespace_cmd = DeclareLaunchArgument(
#     name='use_namespace',
#     default_value='false',
#     description='Whether to apply a namespace to the navigation stack')
             
#   # add if you want rviz to auto exist
#   # declare_rviz_config_file_cmd = DeclareLaunchArgument(
#   #   name='rviz_config_file',
#   #   default_value=default_rviz_config_path,
#   #   description='Full path to the RVIZ config file to use')
 
#   declare_simulator_cmd = DeclareLaunchArgument(
#     name='headless',
#     default_value='False',
#     description='Whether to execute gzclient')
 
#   declare_urdf_model_path_cmd = DeclareLaunchArgument(
#     name='urdf_model', 
#     default_value=default_urdf_model_path, 
#     description='Absolute path to robot urdf file')
     
#   declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
#     name='use_robot_state_pub',
#     default_value='True',
#     description='Whether to start the robot state publisher')
 
#   declare_use_rviz_cmd = DeclareLaunchArgument(
#     name='use_rviz',
#     default_value='True',
#     description='Whether to start RVIZ')
     
#   declare_use_sim_time_cmd = DeclareLaunchArgument(
#     name='use_sim_time',
#     default_value='true',
#     description='Use simulation (Gazebo) clock if true')
 
#   declare_use_simulator_cmd = DeclareLaunchArgument(
#     name='use_simulator',
#     default_value='True',
#     description='Whether to start the simulator')
 
#   declare_world_cmd = DeclareLaunchArgument(
#     name='world',
#     default_value=world_path,
#     description='Full path to the world model file to load')
   
#   # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
#   start_robot_state_publisher_cmd = Node(
#     package='robot_state_publisher',
#     executable='robot_state_publisher',
#     parameters=[{'robot_description': Command(['xacro ', urdf_model])}])
 
#   # Publish the joint states of the robot
#   start_joint_state_publisher_cmd = Node(
#     package='joint_state_publisher',
#     executable='joint_state_publisher',
#     name='joint_state_publisher',
#     condition=UnlessCondition(gui))
 
#   # Launch RViz
#   # start_rviz_cmd = Node(
#   #   package='rviz2',
#   #   executable='rviz2',
#   #   name='rviz2',
#   #   output='screen',
#   #   arguments=['-d', rviz_config_file])
 
#   # Start Gazebo server
#   # start_gazebo_server_cmd = IncludeLaunchDescription(
#   #   PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
#   #   condition=IfCondition(use_simulator),
#   #   launch_arguments={'world': world}.items())
 
#   # # Start Gazebo client    
#   # start_gazebo_client_cmd = IncludeLaunchDescription(
#   #   PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
#   #   condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
#   start_gazebo = DeclareLaunchArgument(
#     'world',
#     default_value=[os.path.join(pkg_gazebo_ros, world_file_path), ''],
#     description='SDF world file'
#   )
#   gazebo = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#       os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
#     )
#   )
 
#   # Launch the robot
#   spawn_entity_cmd = Node(
#     package='gazebo_ros', 
#     executable='spawn_entity.py',
#     arguments=[#'-file', default_urdf_model_path, 
#                 '-entity', robot_name_in_model,
#                 '-topic', 'robot_description',
#                     '-x', spawn_x_val,
#                     '-y', spawn_y_val,
#                     '-z', spawn_z_val,
#                     '-Y', spawn_yaw_val],
#                     output='screen')
 
#   # Create the launch description and populate
#   ld = LaunchDescription()
 
#   # Declare the launch options
#   ld.add_action(declare_use_joint_state_publisher_cmd)
#   ld.add_action(declare_namespace_cmd)
#   ld.add_action(declare_use_namespace_cmd)
#   # add if you want rviz to auto exist
#   # ld.add_action(declare_rviz_config_file_cmd)
#   ld.add_action(declare_simulator_cmd)
#   ld.add_action(declare_urdf_model_path_cmd)
#   ld.add_action(declare_use_robot_state_pub_cmd)  
#   ld.add_action(declare_use_rviz_cmd) 
#   ld.add_action(declare_use_sim_time_cmd)
#   ld.add_action(declare_use_simulator_cmd)
#   ld.add_action(declare_world_cmd)
 
#   # Add any actions
#   ld.add_action(start_gazebo)
#   ld.add_action(gazebo)
#   # ld.add_action(spawn_entity_cmd)
#   # ld.add_action(start_robot_state_publisher_cmd)
#   # ld.add_action(start_joint_state_publisher_cmd)
#   # ld.add_action(start_rviz_cmd)
 
#   return ld


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  robot_name = 'rigatoni'
  world_file_name = 'ev_grand_prix.world'

  pkg_rr_gazebo = get_package_share_directory('rr_gazebo')
  pkg_rr_descrp = get_package_share_directory('rr_description')
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

  gazebo_model_path = os.sep.join([pkg_rr_gazebo, 'models'])
  gazebo_media_path = os.sep.join([pkg_rr_descrp, 'media', 'materials'])

  world = os.path.join(pkg_rr_gazebo, 'worlds', world_file_name)
  urdf = os.path.join(pkg_rr_descrp, 'urdf', 'rigatoni.urdf')

  xml = open(urdf, 'r').read()
  xml = xml.replace('"', '\\"')

  spawn_args = '{name: \"rigatoni\", xml: \"' + xml + '\"}'

  return LaunchDescription([
    ExecuteProcess(
      cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_force_system.so'],
      output='screen'
    ),

    #todo this needs to append it not just set it.
    SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path + os.pathsep + os.environ['GAZEBO_MODEL_PATH']),
    # SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_media_path + os.pathsep + os.environ['GAZEBO_MODEL_PATH']),
    SetEnvironmentVariable('GAZEBO_MEDIA_PATH', gazebo_media_path),
    ExecuteProcess(
      cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
      output='screen'
    ),

    ExecuteProcess(
      cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_args],
      output='screen'
    )
  ])

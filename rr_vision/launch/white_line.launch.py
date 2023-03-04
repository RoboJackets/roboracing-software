from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  pkg_realsense = get_package_share_directory('realsense2_camera')

  realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py'),
        ),
        launch_arguments={'pointcloud.enable': 'false', 'rgb_camera.profile': '1920x1080x30'}.items()
    )

  line_detector = Node(
    package='rr_vision',
    executable='rr_vision_line_detector',
    output='screen',
    parameters=[{'min_white': 225}]
  )
  

  return LaunchDescription([
    realsense,
    line_detector
  ])
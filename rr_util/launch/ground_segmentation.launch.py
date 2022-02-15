import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_yaml
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

import yaml

def generate_launch_description():
    default_parameters_file_path = os.path.join(get_package_share_directory(
        'rr_util'), 'launch', 'ground_segmentation_params.yaml')
    
    with open("ground_segmentation_params.yaml", "r") as stream:
        params = yaml.load(stream)
    
    paramList = []

    for i, v in params["ground_segmentation"]["ros__parameters"].items():
        paramList.append({i: v})

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch.actions.DeclareLaunchArgument(
            "config_path",
            default_value=default_parameters_file_path
        ),
        launch_ros.actions.Node(
            package='rr_util', executable='ground_segmentation', output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'ground_segmentation'],
            parameters=paramList
        ),
    ])
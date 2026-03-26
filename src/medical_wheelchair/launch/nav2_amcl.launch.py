import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('medical_wheelchair')

    default_map = PathJoinSubstitution([pkg_share, 'maps', 'map.yaml'])
    default_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Run SLAM if true, localization if false'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map YAML file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'slam': LaunchConfiguration('slam'),
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_collision_monitor': 'False',
                'use_composition': 'True',
            }.items()
        ),
    ])

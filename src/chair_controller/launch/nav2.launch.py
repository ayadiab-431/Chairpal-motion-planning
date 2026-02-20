import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_chair_controller = get_package_share_directory('chair_controller')
    nav2_params_file = os.path.join(pkg_chair_controller, 'config', 'nav2_params.yaml')

    # SLAM Toolbox for Mapping and Localization
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Nav2 Bringup (Navigation only - Planner, Controller, BT Navigator)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    return LaunchDescription([
        slam_launch,
        nav2_launch
    ])

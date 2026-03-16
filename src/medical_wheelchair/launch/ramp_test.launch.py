from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the main spawn launch file but override the world argument
    spawn_wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('medical_wheelchair'),
                'launch',
                'spawn_wheelchair.launch.py'
            ])
        ]),
        launch_arguments={
            'world': '/home/salwa/chair_ws/src/medical_wheelchair/worlds/ramp.sdf'
        }.items()
    )

    return LaunchDescription([
        spawn_wheelchair_launch
    ])

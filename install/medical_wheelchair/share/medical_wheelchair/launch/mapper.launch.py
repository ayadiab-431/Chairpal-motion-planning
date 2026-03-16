from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'max_laser_range': 10.0,
                'minimum_time_interval': 0.1,
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping'
            }]
        )
    ])

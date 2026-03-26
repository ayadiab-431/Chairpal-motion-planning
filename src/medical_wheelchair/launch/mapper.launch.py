from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('medical_wheelchair')
    
    slam_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'mapper_params_online_async.yaml'
    ])

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}],
    )

    scan_fix = Node(
        package='medical_wheelchair',
        executable='scan_republisher',
        name='scan_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/scan',
            'output_topic': '/scan_fixed',
            'frame_id': 'lidar_link',
            'strip_prefix': 'wheelchair/',
        }],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='slam_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            # Allow extra time for TF/scan to appear before bond timeout
            'bond_timeout': 15.0,
        }],
    )

    return LaunchDescription([
        scan_fix,
        slam_node,
        lifecycle_mgr,
    ])

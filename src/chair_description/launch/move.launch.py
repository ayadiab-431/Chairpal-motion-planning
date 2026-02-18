import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Path to display.launch.py
    display_launch_file = os.path.join(
        get_package_share_directory('chair_description'),
        'launch',
        'display.launch.py'
    )

    return LaunchDescription([
        
        # Launch Rviz Display (Robot State Publisher + Joint State Publisher GUI + Rviz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file)
        ),

        # Fake Odom Publisher (Handles movement logic)
        Node(
            package='chair_controller',
            executable='fake_odom_publisher',
            output='screen'
        ),
        
        # Base Controller (Optional - converts Twist to Wheel Speeds)
        Node(
            package='chair_controller',
            executable='base_controller',
            output='screen'
        ),

        # Goal Navigator (Autonomous navigation to Rviz 2D Goal Pose)
        Node(
            package='chair_controller',
            executable='goal_navigator',
            output='screen'
        ),
    ])

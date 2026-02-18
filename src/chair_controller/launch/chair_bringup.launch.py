from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='chair_controller',
            executable='base_controller',
            name='base_controller',
            output='screen'
        ),

        Node(
            package='chair_controller',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),

        Node(
            package='chair_controller',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
    ])

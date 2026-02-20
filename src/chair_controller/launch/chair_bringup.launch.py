from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Include the wheelchair simulation launch file
    wheelchair_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('medical_wheelchair'),
                'launch',
                'spawn_wheelchair.launch.py'
            ])
        ])
    )

    # 2. Base Controller Node
    base_controller_node = Node(
        package='chair_controller',
        executable='base_controller',
        name='base_controller',
        output='screen'
    )

    # 3. Motor Driver Node
    motor_driver_node = Node(
        package='chair_controller',
        executable='motor_driver',
        name='motor_driver',
        output='screen'
    )

    # 4. Obstacle Detector Node
    obstacle_detector_node = Node(
        package='chair_controller',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen'
    )

    return LaunchDescription([
        wheelchair_sim_launch,
        base_controller_node,
        motor_driver_node,
        obstacle_detector_node
    ])

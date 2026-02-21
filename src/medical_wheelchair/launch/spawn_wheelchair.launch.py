import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_medical_wheelchair = get_package_share_directory('medical_wheelchair')
    xacro_file = os.path.join(pkg_medical_wheelchair, 'urdf', 'wheelchair.xacro')
    world_file = os.path.join(pkg_medical_wheelchair, 'worlds', 'house.sdf')

    # Ignition Gazebo spawn node
    spawn_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wheelchair',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}]
        ),

        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        ),

        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
            output='screen'
        ),

        # Spawn بعد 2 ثانية
        TimerAction(
            period=2.0,
            actions=[spawn_cmd]
        )
    ])

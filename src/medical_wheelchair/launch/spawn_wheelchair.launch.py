from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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
            parameters=[
                {'use_sim_time': True},
                {'robot_description': Command([
                    FindExecutable(name='xacro'), ' ',
                    PathJoinSubstitution([
                        FindPackageShare('medical_wheelchair'),
                        'urdf',
                        'wheelchair.xacro'
                    ])
                ])}
            ]
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
            launch_arguments={
                'gz_args': [
                    '-r ',
                    PathJoinSubstitution([
                        FindPackageShare('medical_wheelchair'),
                        'worlds',
                        'empty_with_sensors.sdf'
                    ])
                ]
            }.items()
        ),

        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/wheelchair/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/wheelchair/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            remappings=[
                ('/model/wheelchair/cmd_vel', '/cmd_vel'),
                ('/model/wheelchair/odom', '/odom'),
            ],
            output='screen'
        ),

        # Publish odom -> base_footprint TF from /odom to keep TF tree consistent
        Node(
            package='medical_wheelchair',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'odom_topic': '/odom'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_footprint'},
                {'strip_prefix': 'wheelchair/'},
            ]
        ),

        # Spawn بعد 2 ثانية
        TimerAction(
            period=2.0,
            actions=[spawn_cmd]
        ),

        # Launch Rviz بعد 3 ثواني
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'rviz2', '-d',
                        PathJoinSubstitution([
                            FindPackageShare('medical_wheelchair'),
                            'rviz',
                            'medical_wheelchair.rviz'
                        ]),
                        '--ros-args', '-p', 'use_sim_time:=true'
                    ],
                    output='screen'
                )
            ]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='/home/salwa/chair_ws/worlds/empty.sdf',
        description='Path to Gazebo world file'
    )

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
        world_arg,
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
                    '/home/salwa/chair_ws/src/medical_wheelchair/urdf/wheelchair.xacro'
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
            launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
        ),

        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/teleop_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            remappings=[
                ('/teleop_cmd_vel', '/cmd_vel'),
                ('/tf', '/tf_gazebo'),
            ],
            output='screen'
        ),

        # TF Republisher: strips 'wheelchair/' prefix from Gazebo TF frames
        Node(
            package='medical_wheelchair',
            executable='tf_republisher',
            name='tf_republisher',
            output='screen'
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
                        'rviz2', '-d', '/home/salwa/chair_ws/src/medical_wheelchair/rviz/medical_wheelchair.rviz',
                        '--ros-args', '-p', 'use_sim_time:=true'
                    ],
                    output='screen'
                )
            ]
        )
    ])

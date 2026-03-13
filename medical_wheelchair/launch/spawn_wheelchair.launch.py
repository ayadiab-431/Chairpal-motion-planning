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
                    '/home/ayadiab/chair_ws/src/medical_wheelchair/urdf/wheelchair.xacro'
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
            launch_arguments={'gz_args': '-r /home/ayadiab/chair_ws/worlds/medical_office.sdf'}.items()
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
            ],
            output='screen'
        ),

        # Bridge: connects wheelchair/base_footprint (Gazebo) to base_footprint (URDF)
        # This unifies the two disconnected TF trees into one complete chain
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_bridge_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'wheelchair/base_footprint', 'base_footprint']
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
                        'rviz2', '-d', '/home/ayadiab/chair_ws/src/medical_wheelchair/rviz/medical_wheelchair.rviz',
                        '--ros-args', '-p', 'use_sim_time:=true'
                    ],
                    output='screen'
                )
            ]
        )
    ])

from setuptools import setup

package_name = 'chair_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Toty Diab',
    maintainer_email='toty@example.com',
    description='Chair simulation with obstacle detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_controller = chair_controller.base_controller:main',
            'motor_driver = chair_controller.motor_driver:main',
            'obstacle_detector = chair_controller.obstacle_detector:main',
            'lidar_fake_publisher = chair_controller.lidar_fake_publisher:main',
            'path_planner = chair_controller.path_planner:main',
            'fake_odom_publisher = chair_controller.fake_odom_publisher:main',
            'goal_navigator = chair_controller.goal_navigator:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/chair_controller']),
        ('share/chair_controller', ['package.xml']),
        ('share/chair_controller/launch', ['launch/chair_bringup.launch.py']),
    ],

)

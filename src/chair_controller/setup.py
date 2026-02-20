from setuptools import setup
import os
from glob import glob

package_name = 'chair_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
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
        ],
    },
)

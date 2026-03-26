from setuptools import find_packages, setup

package_name = 'medical_wheelchair'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/spawn_wheelchair.launch.py',
            'launch/mapper.launch.py',
            'launch/view_wheelchair.launch.py',
            'launch/map_server.launch.py',
            'launch/nav2_amcl.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/medical_wheelchair.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/wheelchair.xacro']),
        ('share/' + package_name + '/config', [
            'config/mapper_params_online_async.yaml',
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/scripts', ['scripts/save_map.sh']),
        ('share/' + package_name + '/worlds', ['worlds/empty_with_sensors.sdf']),
        ('share/' + package_name + '/maps', ['maps/map.yaml', 'maps/map.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayadiab',
    maintainer_email='diabaya695@gmail.com',
    description='Autonomous Medical Wheelchair Simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_republisher = medical_wheelchair.tf_republisher:main',
            'odom_to_tf = medical_wheelchair.odom_to_tf:main',
            'scan_republisher = medical_wheelchair.scan_republisher:main',
        ],
    },
)
